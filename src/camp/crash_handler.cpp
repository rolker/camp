#include "crash_handler.h"

#include <execinfo.h>
#include <fcntl.h>
#include <limits.h>
#include <signal.h>
#include <string.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <new>
#include <string>

#include "rclcpp/logger.hpp"

namespace camp_crash
{

namespace
{

/// Durable destination fd. Written at install time (fd form) or by the handler
/// itself (path form), read from signal handlers, hence `volatile
/// sig_atomic_t` rather than plain int.
volatile sig_atomic_t g_crash_fd = -1;

/// Crash-log path, resolved at install time and opened only when a crash
/// actually happens.
///
/// **The file is deliberately NOT pre-opened.** Pre-opening with
/// `O_CREAT|O_TRUNC` left a zero-byte `camp_crash_<pid>.log` behind after every
/// clean run — in a `~/.ros/log` that already holds 10k+ entries, that makes
/// "no crash" indistinguishable from "crashed before the first write" and
/// buries the real reports. Worse, on a host with the stock `pid_max` a
/// recycled pid silently truncated an earlier genuine crash report.
///
/// Resolving the path needs rclcpp and allocates, so that still happens at
/// install time; only the `open()` moves into the handler, and `open(2)` is on
/// the async-signal-safe list.
char g_crash_path[PATH_MAX] = {0};
volatile sig_atomic_t g_crash_path_valid = 0;

/// Open the crash log, once, from inside a handler. Async-signal-safe.
///
/// `O_APPEND` (never `O_TRUNC`): if a recycled pid lands on an existing file,
/// appending keeps both reports where truncating destroyed the older one.
/// `O_NOFOLLOW` + 0600: the path is derived from `$ROS_LOG_DIR`/`$ROS_HOME`,
/// which are environment-controlled, so a pre-planted symlink in a shared log
/// dir must not turn this into a write into someone else's file — and the dump
/// embeds full install paths, which nothing else needs to read.
void open_crash_fd()
{
  if (g_crash_fd >= 0 || !g_crash_path_valid)
    return;
  g_crash_fd = ::open(g_crash_path,
                      O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC | O_NOFOLLOW,
                      0600);
}

/// Claimed by whichever handler emits first, via an atomic test-and-set.
///
/// The terminate path ends in `std::abort()`, which raises SIGABRT and
/// re-enters the signal handler; without this guard every uncaught exception
/// would print a second backtrace rooted in `abort()` itself, burying the
/// useful one. The signal handler still re-raises when this is set — only the
/// output is suppressed, never the exit status.
///
/// It must be an **atomic** test-and-set, not `if (!flag) flag = 1;`:
/// `volatile` prevents tearing but not a race, so two threads faulting in the
/// same window would both pass the test and interleave two dumps across ~5
/// `write()` calls each. `__atomic_test_and_set` is lock-free and
/// async-signal-safe.
volatile unsigned char g_already_dumped = 0;

/// Returns true for the first caller only.
bool claim_dump()
{
  return !__atomic_test_and_set(&g_already_dumped, __ATOMIC_ACQ_REL);
}

/// Alternate signal stack — **per thread**. A stack-overflow SIGSEGV leaves no
/// room to push a handler frame on the faulting stack, so without
/// `sigaltstack` + `SA_ONSTACK` that class of crash stays as silent as it is
/// today.
///
/// `sigaltstack(2)` is a per-thread attribute and `pthread_create(3)`
/// explicitly does not inherit it, so installing one on the main thread covers
/// only the main thread. Hence `thread_local` plus the exported
/// `install_thread_alt_stack()`, which every thread CAMP creates itself calls
/// on entry. Threads created inside rclcpp (the `MultiThreadedExecutor`
/// workers, the `tf2_ros::TransformListener` thread) cannot be hooked and so
/// have no alternate stack — see the scoping note in `crash_handler.h`.
thread_local char* t_alt_stack = nullptr;

constexpr int kMaxFrames = 128;

/// `write()` to both destinations. Async-signal-safe: no allocation, no
/// buffered I/O, no locks. Short writes and errors are ignored deliberately —
/// there is nothing useful to do about them from inside a crash.
///
/// **The durable file is written FIRST, stderr second.** Under `ros2 launch`
/// stderr is a pipe to the launch parent, and in the abort-on-close class
/// (#207) that parent may already be gone or no longer draining. Writing
/// stderr first would mean a full pipe blocks the handler forever (CAMP hangs
/// instead of dying, and no "process has died" line is ever logged) or SIGPIPE
/// kills the process mid-handler — either way costing the crash file this
/// feature exists to produce. The file is a regular fd: it never blocks and
/// never raises SIGPIPE.
void emit(const char* text, size_t len)
{
  if (len == 0)
    return;
  ssize_t ignored;
  if (g_crash_fd >= 0)
  {
    ignored = ::write(static_cast<int>(g_crash_fd), text, len);
    (void)ignored;
  }
  ignored = ::write(STDERR_FILENO, text, len);
  (void)ignored;
}

void emit(const char* text)
{
  emit(text, ::strlen(text));
}

/// Async-signal-safe unsigned formatter (no snprintf: it is not on the
/// async-signal-safe list and glibc's may take a lock).
void emit_ulong(unsigned long value, int base)
{
  char buf[32];
  char* p = buf + sizeof(buf);
  const char* digits = "0123456789abcdef";
  do
  {
    *--p = digits[value % static_cast<unsigned long>(base)];
    value /= static_cast<unsigned long>(base);
  } while (value != 0 && p > buf);
  emit(p, static_cast<size_t>(buf + sizeof(buf) - p));
}

/// Backtrace to both destinations. `backtrace_symbols_fd` writes straight to
/// an fd; `backtrace_symbols` would allocate, which is not safe here — heap
/// corruption is a suspected cause of the crashes this exists to diagnose
/// (#215).
/// Durable file first, stderr second, for the reason documented on `emit()`.
void emit_backtrace()
{
  void* frames[kMaxFrames];
  const int count = ::backtrace(frames, kMaxFrames);
  if (g_crash_fd >= 0)
    ::backtrace_symbols_fd(frames, count, static_cast<int>(g_crash_fd));
  ::backtrace_symbols_fd(frames, count, STDERR_FILENO);
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

/// C language linkage: `sa_sigaction` is a C function pointer, and a handler
/// with C++ linkage is only conventionally compatible.
extern "C" void on_fatal_signal(int sig, siginfo_t* info, void* /*ucontext*/)
{
  // FIRST statement, before anything that could itself fault: restore the
  // default action. SA_RESETHAND has already done this at delivery; the
  // explicit call keeps the guarantee if the flag is ever dropped. If this
  // handler crashes — entirely possible when the heap is already corrupt — the
  // process dies immediately instead of recursing.
  ::signal(sig, SIG_DFL);

  // Bound the handler's own lifetime. backtrace()/backtrace_symbols_fd() take
  // glibc's loader locks; if another thread is inside a GDAL or Qt-plugin
  // dlopen() holding one, the handler deadlocks and a silent death becomes a
  // silent HANG — strictly worse, because the supervisor never even logs
  // "process has died". SIGALRM's default action terminates the process, and
  // the alarm is set after SIG_DFL is restored so nothing intercepts it.
  ::alarm(10);

  if (claim_dump())
  {
    open_crash_fd();
    emit("\n=== CAMP caught ");
    emit(signal_name(sig));
    emit(" on thread ");
    emit_ulong(static_cast<unsigned long>(::syscall(SYS_gettid)), 10);

    // si_code / si_addr often identify the fault class before anyone reads the
    // stack: a null `this->member` (addr near 0), a use-after-free, a wild
    // write. SEGV_MAPERR=1 (unmapped), SEGV_ACCERR=2 (permissions).
    if (info != nullptr)
    {
      emit(" [si_code=");
      emit_ulong(static_cast<unsigned long>(info->si_code), 10);
      if (sig == SIGSEGV || sig == SIGBUS || sig == SIGFPE || sig == SIGILL)
      {
        emit(" si_addr=0x");
        emit_ulong(reinterpret_cast<unsigned long>(info->si_addr), 16);
      }
      emit("]");
    }

    emit(" — backtrace follows (mangled; pipe through c++filt) ===\n");
    emit_backtrace();
    emit("=== end CAMP backtrace ===\n");
  }

  // Re-raise so the exit status is exactly what it would have been.
  //
  // Note this is the raise() site, not the faulting instruction: if core dumps
  // are ever enabled on a host (they are not today — CAMP is unpackaged, so
  // apport drops the report, and `ulimit -c` is 0 by default) the core's
  // faulting frame is this handler rather than the real fault. Returning
  // instead would re-execute the faulting instruction and dump at the right
  // place for the four synchronous faults, but not for SIGABRT, and it changes
  // the death path for a diagnostics-only feature. Deliberately not done:
  // the backtrace above is what this feature delivers, and it is unaffected.
  ::raise(sig);
}

void on_terminate()
{
  if (claim_dump())
  {
    open_crash_fd();

    // ORDER IS LOAD-BEARING: backtrace first, exception introspection second.
    //
    // current_exception()/rethrow_exception() allocate and run the unwinder —
    // exactly the machinery that can fault when the reason for terminating is
    // the heap corruption this exists to diagnose. If it faults, the SIGSEGV
    // handler finds g_already_dumped set (it must, or abort() would print a
    // second useless stack), suppresses its output, and the crash produces a
    // header line and NO STACK AT ALL. Emitting the stack first means the
    // worst case degrades to "stack without a reason line" instead of
    // "reason-less header without a stack".
    emit("\n=== CAMP std::terminate — backtrace follows "
         "(mangled; pipe through c++filt) ===\n");
    emit_backtrace();
    emit("=== end CAMP backtrace ===\n");

    // Name the active exception if there is one. This is the single most
    // useful line for the abort-on-close class (#207). Kept to write()+strlen
    // for symmetry with the signal path: if the reason for terminating is
    // heap corruption, allocating here would just fail a second time.
    emit("=== CAMP terminate reason: ");
    if (std::exception_ptr active = std::current_exception())
    {
      try
      {
        std::rethrow_exception(active);
      }
      catch (const std::exception& e)
      {
        emit("uncaught exception: ");
        emit(e.what());
      }
      catch (...)
      {
        emit("uncaught non-std exception");
      }
    }
    else
    {
      // e.g. "pure virtual method called", which reaches terminate with no
      // active exception.
      emit("no active exception");
    }
    emit(" ===\n");
  }

  std::abort();
}

} // namespace

std::string crash_log_path()
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
    return std::string();
  }

  if (dir.empty())
    return std::string();

  return dir + "/camp_crash_" + std::to_string(::getpid()) + ".log";
}

void install_thread_alt_stack()
{
  if (t_alt_stack != nullptr)
    return;

  // SIGSTKSZ is sysconf(_SC_SIGSTKSZ) on glibc >= 2.34, i.e. a function call
  // rather than a constant. Evaluate it once, so the allocation and the size
  // handed to the kernel cannot disagree.
  const size_t stack_size = static_cast<size_t>(SIGSTKSZ);

  t_alt_stack = new (std::nothrow) char[stack_size];
  if (t_alt_stack == nullptr)
    return;

  stack_t ss;
  ss.ss_sp = t_alt_stack;
  ss.ss_size = stack_size;
  ss.ss_flags = 0;
  if (::sigaltstack(&ss, nullptr) != 0)
  {
    // Nothing to do about it from here, but don't leave the pointer looking
    // like a live alternate stack: a later retry on this thread may succeed.
    delete[] t_alt_stack;
    t_alt_stack = nullptr;
  }
}

void install_crash_handlers(const std::string& crash_log_path)
{
  install_crash_handlers(-1);

  // The file is opened lazily, by the handler — see g_crash_path. An empty or
  // over-long path is not an error: the handlers stay installed and write to
  // stderr only.
  if (!crash_log_path.empty() && crash_log_path.size() < sizeof(g_crash_path))
  {
    ::memcpy(g_crash_path, crash_log_path.c_str(), crash_log_path.size() + 1);
    g_crash_path_valid = 1;
  }
}

void install_crash_handlers(int backtrace_fd)
{
  g_crash_fd = backtrace_fd;
  g_crash_path_valid = 0;
  __atomic_clear(&g_already_dumped, __ATOMIC_RELEASE);

  // Ignore SIGPIPE for the process. Without this, a write to a stderr pipe
  // whose reader has exited kills CAMP outright — changing the exit status the
  // supervisor sees from the real fault (-11) to 13, and truncating the dump
  // partway. CAMP writes nothing else to a pipe it needs SIGPIPE for.
  ::signal(SIGPIPE, SIG_IGN);

  // Force backtrace()'s lazy initialization now. Its first call may dlopen
  // libgcc and allocate — exactly what must not happen inside a handler
  // entered from a corrupted heap.
  {
    void* warmup[4];
    (void)::backtrace(warmup, 4);
  }

  // Alternate stack for the installing (main) thread. Other threads must call
  // install_thread_alt_stack() themselves — see that function's contract.
  install_thread_alt_stack();

  struct sigaction sa;
  ::memset(&sa, 0, sizeof(sa));
  sa.sa_sigaction = &on_fatal_signal;
  ::sigemptyset(&sa.sa_mask);
  // SA_SIGINFO: si_code/si_addr, see on_fatal_signal.
  // SA_RESETHAND: the kernel restores SIG_DFL atomically at delivery, which is
  //   what the handler's first statement asks for.
  // No SA_RESTART: it governs restarting interrupted syscalls after the
  //   handler RETURNS, and none of these handlers return normally — it read as
  //   a claim about behavior that cannot happen.
  sa.sa_flags = SA_ONSTACK | SA_SIGINFO | SA_RESETHAND;

  // All five are the same class of fatal, silent death; the handler is
  // identical for each.
  const int fatal[] = {SIGSEGV, SIGABRT, SIGBUS, SIGFPE, SIGILL};
  for (int sig : fatal)
  {
    if (::sigaction(sig, &sa, nullptr) != 0)
    {
      // Not fatal — the other signals are still covered — but it must not be
      // silent, or a crash class quietly stops being reported. Safe to use
      // stderr formatting here: install time, not handler time.
      ::fprintf(stderr,
                "[camp #217] could not install crash handler for signal %d; "
                "crashes of that kind will not be reported\n", sig);
    }
  }

  std::set_terminate(&on_terminate);
}

} // namespace camp_crash
