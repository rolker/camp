/// Regression tests for CAMP's crash handlers (#217).
///
/// These are gtest *death tests*: each crashing statement runs in a forked
/// child, so the parent test process survives. What is asserted, per the work
/// plan, is both halves of the contract:
///
///   1. the exit status is preserved (the handler re-raises rather than
///      swallowing), so a supervisor still sees -11 / -6, and
///   2. the backtrace actually reached the file, with *resolved symbol names*
///      — not merely non-empty output. Bare addresses are what you get when
///      ENABLE_EXPORTS is dropped from the target, and a non-empty assertion
///      would pass in that state while covering nothing.

#include <gtest/gtest.h>

#include <fcntl.h>
#include <signal.h>
#include <unistd.h>

#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "crash_handler.h"

namespace
{

std::string temp_path(const char* tag)
{
  std::ostringstream oss;
  oss << "/tmp/camp_crash_test_" << tag << "_" << ::getpid() << ".log";
  return oss.str();
}

std::string read_file(const std::string& path)
{
  std::ifstream in(path);
  std::ostringstream oss;
  oss << in.rdbuf();
  return oss.str();
}

int open_temp(const std::string& path)
{
  const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  EXPECT_GE(fd, 0) << "could not open " << path;
  return fd;
}

} // namespace

// The two crashing helpers below are deliberately at file scope, NOT in the
// anonymous namespace above. Internal linkage keeps a symbol out of the
// dynamic symbol table, so backtrace_symbols_fd() renders such frames as bare
// `binary(+0xoffset)` even with ENABLE_EXPORTS set — which would make the
// resolved-symbol assertion below untestable. External linkage is what puts
// the name in the dump.

/// Named distinctly so the test can assert this symbol appears in the dump —
/// which is what makes the test cover ENABLE_EXPORTS rather than just
/// "something was written".
void camp_test_crashing_frame(int sig)
{
  ::raise(sig);
}

/// `noexcept` is load-bearing, not decoration.
///
/// gtest wraps death-test statements in try/catch when exceptions are enabled
/// (gtest-death-test-internal.h), so a plain `throw` inside ASSERT_EXIT is
/// caught by gtest and reported as TEST_THREW_EXCEPTION — std::terminate is
/// never reached and the set_terminate handler would ship untested. Throwing
/// across a noexcept boundary escapes that catch and forces the real
/// std::terminate path.
///
/// GCC warns `'throw' will always call 'terminate'` here. That is precisely
/// the intent — this function exists to reach std::terminate — so the warning
/// is silenced at this one site rather than repaired.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wterminate"
void camp_test_throwing_frame() noexcept
{
  throw std::runtime_error("boom");
}
#pragma GCC diagnostic pop

TEST(CrashHandler, SigsegvDumpsBacktraceAndPreservesExitStatus)
{
  const std::string path = temp_path("segv");

  ASSERT_EXIT(
    {
      camp_crash::install_crash_handlers(open_temp(path));
      camp_test_crashing_frame(SIGSEGV);
    },
    ::testing::KilledBySignal(SIGSEGV), "");

  const std::string dump = read_file(path);
  EXPECT_NE(dump.find("SIGSEGV"), std::string::npos) << dump;
  // Resolved symbol name, not just any output: this is the ENABLE_EXPORTS
  // coverage.
  EXPECT_NE(dump.find("camp_test_crashing_frame"), std::string::npos)
    << "no resolved symbol in backtrace — is ENABLE_EXPORTS still set on the "
       "target?\n" << dump;

  ::remove(path.c_str());
}

TEST(CrashHandler, SigabrtDumpsBacktraceAndPreservesExitStatus)
{
  const std::string path = temp_path("abrt");

  ASSERT_EXIT(
    {
      camp_crash::install_crash_handlers(open_temp(path));
      camp_test_crashing_frame(SIGABRT);
    },
    ::testing::KilledBySignal(SIGABRT), "");

  const std::string dump = read_file(path);
  EXPECT_NE(dump.find("SIGABRT"), std::string::npos) << dump;
  EXPECT_NE(dump.find("camp_test_crashing_frame"), std::string::npos) << dump;

  ::remove(path.c_str());
}

TEST(CrashHandler, UncaughtExceptionNamesItAndDumpsBacktrace)
{
  const std::string path = temp_path("throw");

  ASSERT_EXIT(
    {
      camp_crash::install_crash_handlers(open_temp(path));
      camp_test_throwing_frame();
    },
    ::testing::KilledBySignal(SIGABRT), "");

  const std::string dump = read_file(path);
  EXPECT_NE(dump.find("std::terminate"), std::string::npos) << dump;
  // The exception message is the single most useful line for the
  // abort-on-close class (#207).
  EXPECT_NE(dump.find("boom"), std::string::npos) << dump;

  // The terminate path ends in abort(), which re-enters the signal handler.
  // Exactly one backtrace must be emitted, not two.
  const std::string marker = "=== end CAMP backtrace ===";
  const size_t first = dump.find(marker);
  ASSERT_NE(first, std::string::npos) << dump;
  EXPECT_EQ(dump.find(marker, first + marker.size()), std::string::npos)
    << "second backtrace emitted via abort() — the already_dumped guard is "
       "not holding\n" << dump;

  ::remove(path.c_str());
}
