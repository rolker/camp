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
///
/// Note what this file does NOT cover: the `ENABLE_EXPORTS` property on the
/// *shipped* `CCOMAutonomousMissionPlanner` executable. These tests run against
/// `test_crash_handler`, which sets that property on itself, so they stay green
/// if the flag is dropped from the main target. The `check_camp_exports` CTest
/// (CMakeLists.txt) is the guard for that; it runs `readelf` against the real
/// binary.

#include <gtest/gtest.h>

#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

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
  ::remove(path.c_str());  // the handler creates it (O_APPEND, never O_TRUNC)

  ASSERT_EXIT(
    {
      camp_crash::install_crash_handlers(path);
      camp_test_crashing_frame(SIGSEGV);
    },
    // The matcher is against the child's STDERR — the half that lands in the
    // ros2 launch log next to "process has died". Asserting only against the
    // file would leave deleting every write(STDERR_FILENO, ...) green.
    ::testing::KilledBySignal(SIGSEGV), "CAMP caught SIGSEGV");

  const std::string dump = read_file(path);
  EXPECT_NE(dump.find("SIGSEGV"), std::string::npos) << dump;
  // Resolved symbol name, not just any output: this is the ENABLE_EXPORTS
  // coverage.
  EXPECT_NE(dump.find("camp_test_crashing_frame"), std::string::npos)
    << "no resolved symbol in backtrace — is ENABLE_EXPORTS still set on the "
       "target?\n" << dump;

  // This SIGSEGV was raise()d, not faulted: si_code is SI_TKILL (-6), so the
  // siginfo union holds si_pid/si_uid and there is no fault address to print.
  // Gating si_addr on the signal number alone printed (uid << 32 | pid) here.
  EXPECT_NE(dump.find("[si_code=-6"), std::string::npos) << dump;
  EXPECT_EQ(dump.find("si_addr"), std::string::npos)
    << "si_addr printed for a raise()d SIGSEGV\n" << dump;

  ::remove(path.c_str());
}

TEST(CrashHandler, SigabrtDumpsBacktraceAndPreservesExitStatus)
{
  const std::string path = temp_path("abrt");
  ::remove(path.c_str());  // the handler creates it (O_APPEND, never O_TRUNC)

  ASSERT_EXIT(
    {
      camp_crash::install_crash_handlers(path);
      camp_test_crashing_frame(SIGABRT);
    },
    ::testing::KilledBySignal(SIGABRT), "CAMP caught SIGABRT");

  const std::string dump = read_file(path);
  EXPECT_NE(dump.find("SIGABRT"), std::string::npos) << dump;
  EXPECT_NE(dump.find("camp_test_crashing_frame"), std::string::npos) << dump;

  // si_code must render SIGNED. raise() goes through tgkill(2), so the kernel
  // reports SI_TKILL (-6) — which an unsigned formatter printed as
  // 18446744073709551610 on every abort-path dump, i.e. on exactly the #207
  // abort-on-close class this feature exists to diagnose.
  EXPECT_NE(dump.find("[si_code=-6"), std::string::npos)
    << "si_code not rendered as a signed value\n" << dump;

  // ...and si_addr must NOT appear: at si_code <= 0 the siginfo union holds
  // si_pid/si_uid, not a fault address.
  EXPECT_EQ(dump.find("si_addr"), std::string::npos)
    << "si_addr printed for a user-generated signal — that is (uid << 32 | pid)"
       ", not an address\n" << dump;

  ::remove(path.c_str());
}

TEST(CrashHandler, UncaughtExceptionNamesItAndDumpsBacktrace)
{
  const std::string path = temp_path("throw");
  ::remove(path.c_str());  // the handler creates it (O_APPEND, never O_TRUNC)

  ASSERT_EXIT(
    {
      camp_crash::install_crash_handlers(path);
      camp_test_throwing_frame();
    },
    ::testing::KilledBySignal(SIGABRT), "CAMP std::terminate");

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

/// Distinct from `camp_test_crashing_frame` so the dump proves the backtrace
/// came from the worker thread, not from the main one.
void camp_test_thread_crashing_frame(int sig)
{
  ::raise(sig);
}

TEST(CrashHandler, CrashOnANonMainThreadIsStillReported)
{
  const std::string path = temp_path("thread");
  ::remove(path.c_str());

  // Where CAMP actually crashes: ROS callbacks run on the node thread and on
  // the executor's workers, never on the thread that installed the handlers.
  // sigaction() is process-wide, so this must work — and this is the test that
  // would have caught the alternate signal stack being main-thread-only.
  ASSERT_EXIT(
    {
      camp_crash::install_crash_handlers(path);
      std::thread worker([]
        {
          camp_crash::install_thread_alt_stack();
          camp_test_thread_crashing_frame(SIGSEGV);
        });
      worker.join();
    },
    ::testing::KilledBySignal(SIGSEGV), "CAMP caught SIGSEGV");

  const std::string dump = read_file(path);
  EXPECT_NE(dump.find("camp_test_thread_crashing_frame"), std::string::npos)
    << dump;

  ::remove(path.c_str());
}

/// Installed by the watchdog test to prove the bound does not depend on
/// SIGALRM's disposition being untouched.
extern "C" void camp_test_swallow_alarm(int /*sig*/)
{
}

TEST(CrashHandler, ATerminateDumpThatCannotDrainIsKilledByTheWatchdog)
{
  // The anti-hang bound on the terminate path. emit()/emit_backtrace() can block
  // indefinitely — on a glibc loader lock held by a thread inside dlopen() (the
  // real hazard), or, as staged here, on a destination fd nobody drains. A hang
  // is strictly worse than a crash: the supervisor never even logs "process has
  // died".
  //
  // The child also BLOCKS SIGALRM and installs a handler that swallows it, so
  // this fails unless arm_watchdog() restores SIG_DFL and unblocks the signal
  // itself. A bare alarm() — which is what the signal path's comment used to
  // claim was sufficient, on the grounds that SIG_DFL had been restored for the
  // faulting signal — leaves this child hanging forever.
  //
  // Costs kWatchdogSeconds (10 s) by construction. Removing the bound does not
  // make this test fail fast: it makes it hang until ctest's timeout.
  ASSERT_EXIT(
    {
      int fds[2];
      if (::pipe(fds) != 0)
        ::_exit(2);

      // Fill the pipe so the handler's first write() blocks. The read end stays
      // open (same process) and nothing ever drains it.
      const int flags = ::fcntl(fds[1], F_GETFL, 0);
      ::fcntl(fds[1], F_SETFL, flags | O_NONBLOCK);
      char block[4096];
      ::memset(block, 0, sizeof(block));
      while (::write(fds[1], block, sizeof(block)) > 0)
      {
      }
      ::fcntl(fds[1], F_SETFL, flags);

      ::signal(SIGALRM, &camp_test_swallow_alarm);
      sigset_t alarm_only;
      ::sigemptyset(&alarm_only);
      ::sigaddset(&alarm_only, SIGALRM);
      ::pthread_sigmask(SIG_BLOCK, &alarm_only, nullptr);

      camp_crash::install_crash_handlers(fds[1]);
      camp_test_throwing_frame();
    },
    ::testing::KilledBySignal(SIGALRM), "");
}

TEST(CrashHandler, NoCrashFileMeansNoFile)
{
  // The zero-byte-file regression: installing the handlers must not create
  // anything. An operator scanning ~/.ros/log has to be able to read the
  // presence of a camp_crash_*.log as "this run crashed".
  const std::string path = temp_path("nofile");
  ::remove(path.c_str());

  camp_crash::install_crash_handlers(path);

  std::ifstream probe(path);
  EXPECT_FALSE(probe.good()) << path << " was created without a crash";

  // Leave the process without a stale crash path pointing at a temp file.
  camp_crash::install_crash_handlers(-1);
}

TEST(CrashHandler, UnopenableCrashPathStillDumpsToStderr)
{
  // The degradation contract in crash_handler.h: a crash log that cannot be
  // opened must cost the file, never the report. Covers the fd == -1 branch of
  // emit()/emit_backtrace() as well.
  ASSERT_EXIT(
    {
      camp_crash::install_crash_handlers(
        std::string("/nonexistent-directory-camp217/crash.log"));
      camp_test_crashing_frame(SIGSEGV);
    },
    ::testing::KilledBySignal(SIGSEGV), "CAMP caught SIGSEGV");
}

TEST(CrashHandler, EmptyCrashPathIsStderrOnly)
{
  // What main() gets when the ROS logging directory cannot be resolved, and
  // what the pre-rclcpp::init() install pass uses.
  ASSERT_EXIT(
    {
      camp_crash::install_crash_handlers(std::string());
      camp_test_throwing_frame();
    },
    ::testing::KilledBySignal(SIGABRT), "CAMP std::terminate");
}

TEST(CrashHandler, CrashLogPathIsUnderTheRosLoggingDirectory)
{
  // crash_log_path() had no coverage at all, including the documented
  // "returns empty rather than throwing" degradation.
  ::setenv("ROS_LOG_DIR", "/tmp/camp_crash_test_logdir", 1);
  const std::string path = camp_crash::crash_log_path();
  ::unsetenv("ROS_LOG_DIR");

  ASSERT_FALSE(path.empty()) << "crash_log_path() resolved nothing";
  EXPECT_EQ(path.find("/tmp/camp_crash_test_logdir/"), 0u) << path;
  EXPECT_NE(path.find("camp_crash_"), std::string::npos) << path;
  EXPECT_NE(path.find(std::to_string(::getpid())), std::string::npos) << path;
}
