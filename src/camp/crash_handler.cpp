#include "crash_handler.h"

#include <execinfo.h>
#include <fcntl.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>

#include <cstdlib>
#include <exception>
#include <new>
#include <string>

#include "rclcpp/logger.hpp"

namespace camp_crash
{

namespace
{

/// Destination fds. Written once at install time, read from signal handlers,
/// hence `volatile sig_atomic_t` rather than plain int.
volatile sig_atomic_t g_crash_fd = -1;

/// Set by whichever handler emits first.
///
/// The terminate path ends in `std::abort()`, which raises SIGABRT and
/// re-enters the signal handler; without this guard every uncaught exception
/// would print a second backtrace rooted in `abort()` itself, burying the
/// useful one. The signal handler still re-raises when this is set — only the
/// output is suppressed, never the exit status.
volatile sig_atomic_t g_already_dumped = 0;

/// Alternate signal stack. A stack-overflow SIGSEGV leaves no room to push a
/// handler frame on the faulting stack, so without `sigaltstack` + `SA_ONSTACK`
/// that entire class of crash stays as silent as it is today.
char* g_alt_stack = nullptr;

constexpr int kMaxFrames = 128;

/// `write()` to both destinations. Async-signal-safe: no allocation, no
/// buffered I/O, no locks. Short writes and errors are ignored deliberately —
/// there is nothing useful to do about them from inside a crash.
void emit(const char* text, size_t len)
{
  if (len == 0)
    return;
  ssize_t ignored = ::write(STDERR_FILENO, text, len);
  (void)ignored;
  if (g_crash_fd >= 0)
  {
    ignored = ::write(static_cast<int>(g_crash_fd), text, len);
    (void)ignored;
  }
}

void emit(const char* text)
{
  emit(text, ::strlen(text));
}

/// Backtrace to both destinations. `backtrace_symbols_fd` writes straight to
/// an fd; `backtrace_symbols` would allocate, which is not safe here — heap
/// corruption is a suspected cause of the crashes this exists to diagnose
/// (#215).
void emit_backtrace()
{
  void* frames[kMaxFrames];
  const int count = ::backtrace(frames, kMaxFrames);
  ::backtrace_symbols_fd(frames, count, STDERR_FILENO);
  if (g_crash_fd >= 0)
    ::backtrace_symbols_fd(frames, count, static_cast<int>(g_crash_fd));
}

const char* signal_name(int sig)
{
  switch (sig)
  {
    case SIGSEGV: return "SIGSEGV (invalid memory reference)";
    case SIGABRT: return "SIGABRT (abort)";
    case SIGBUS:  return "SIGBUS (bus error)";
    case SIGFPE:  return "SIGFPE (arithmetic exception)";
    case SIGILL:  return "SIGILL (illegal instruction)";
    default:      return "fatal signal";
  }
}

void on_fatal_signal(int sig)
{
  // FIRST statement, before anything that could itself fault: restore the
  // default action. If this handler crashes — entirely possible when the heap
  // is already corrupt — the process dies immediately instead of recursing.
  ::signal(sig, SIG_DFL);

  if (!g_already_dumped)
  {
    g_already_dumped = 1;
    emit("\n=== CAMP caught ");
    emit(signal_name(sig));
    emit(" — backtrace follows (mangled; pipe through c++filt) ===\n");
    emit_backtrace();
    emit("=== end CAMP backtrace ===\n");
  }

  // Re-raise so the exit status is exactly what it would have been, and any
  // host-level core handling still applies.
  ::raise(sig);
}

void on_terminate()
{
  if (!g_already_dumped)
  {
    g_already_dumped = 1;
    emit("\n=== CAMP std::terminate ");

    // Name the active exception if there is one. This is the single most
    // useful line for the abort-on-close class (#207). Kept to write()+strlen
    // for symmetry with the signal path: if the reason for terminating is
    // heap corruption, allocating here would just fail a second time.
    if (std::exception_ptr active = std::current_exception())
    {
      try
      {
        std::rethrow_exception(active);
      }
      catch (const std::exception& e)
      {
        emit("(uncaught exception: ");
        emit(e.what());
        emit(")");
      }
      catch (...)
      {
        emit("(uncaught non-std exception)");
      }
    }
    else
    {
      // e.g. "pure virtual method called", which reaches terminate with no
      // active exception.
      emit("(no active exception)");
    }

    emit(" — backtrace follows (mangled; pipe through c++filt) ===\n");
    emit_backtrace();
    emit("=== end CAMP backtrace ===\n");
  }

  std::abort();
}

} // namespace

int open_crash_log_fd()
{
  std::string dir;
  try
  {
    // Throws rclcpp::exceptions::RCLError if the directory cannot be
    // resolved. Caught here so a bad log dir degrades to stderr-only rather
    // than terminating CAMP before it has a window — a diagnostics feature
    // must not become a startup failure on the hosts it serves.
    dir = rclcpp::get_logging_directory().string();
  }
  catch (const std::exception&)
  {
    return -1;
  }

  if (dir.empty())
    return -1;

  const std::string path =
    dir + "/camp_crash_" + std::to_string(::getpid()) + ".log";

  // O_CLOEXEC so the fd does not leak into any child process camp spawns.
  const int fd = ::open(path.c_str(),
                        O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC,
                        0644);
  return fd;  // -1 on failure is fine; the caller installs handlers anyway.
}

void install_crash_handlers(int backtrace_fd)
{
  g_crash_fd = backtrace_fd;
  g_already_dumped = 0;

  // Force backtrace()'s lazy initialization now. Its first call may dlopen
  // libgcc and allocate — exactly what must not happen inside a handler
  // entered from a corrupted heap.
  {
    void* warmup[4];
    (void)::backtrace(warmup, 4);
  }

  // Alternate signal stack, so a stack-overflow SIGSEGV is still catchable.
  if (g_alt_stack == nullptr)
  {
    g_alt_stack = new (std::nothrow) char[SIGSTKSZ];
    if (g_alt_stack != nullptr)
    {
      stack_t ss;
      ss.ss_sp = g_alt_stack;
      ss.ss_size = SIGSTKSZ;
      ss.ss_flags = 0;
      ::sigaltstack(&ss, nullptr);
    }
  }

  struct sigaction sa;
  ::memset(&sa, 0, sizeof(sa));
  sa.sa_handler = &on_fatal_signal;
  ::sigemptyset(&sa.sa_mask);
  sa.sa_flags = SA_ONSTACK | SA_RESTART;

  // All five are the same class of fatal, silent death; the handler is
  // identical for each.
  ::sigaction(SIGSEGV, &sa, nullptr);
  ::sigaction(SIGABRT, &sa, nullptr);
  ::sigaction(SIGBUS, &sa, nullptr);
  ::sigaction(SIGFPE, &sa, nullptr);
  ::sigaction(SIGILL, &sa, nullptr);

  std::set_terminate(&on_terminate);
}

} // namespace camp_crash
