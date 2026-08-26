#include "crash_handler.h"

#include <execinfo.h>
#include <fcntl.h>
#include <limits.h>
#include <pthread.h>
#include <signal.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <new>
#include <string>

namespace camp_crash
{

namespace
{

/// Durable destination fd. Written at install time (fd form) or by the handler
/// itself (path form), read from signal handlers, hence `volatile
/// sig_atomic_t` rather than plain int.
volatile sig_atomic_t g_crash_fd = -1;

/// Set when `g_crash_fd` was opened by `open_crash_fd()` rather than handed to
/// us by a caller. A reinstall may close what we opened; it must never close a
/// caller's fd, which we do not own.
volatile sig_atomic_t g_crash_fd_owned = 0;

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
/// Resolving the path allocates (and, in CAMP's case, asks rcl for the ROS
/// logging directory — see `src/camp/crash_log_path.h`), so that still happens
/// at install time, in the caller; only the `open()` moves into the handler,
/// and `open(2)` is on the async-signal-safe list.
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
  if (g_crash_fd >= 0 ||
      !__atomic_load_n(&g_crash_path_valid, __ATOMIC_ACQUIRE))
    return;

  const int fd = ::open(g_crash_path,
                        O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC | O_NOFOLLOW,
                        0600);
  if (fd < 0)
    return;

  // O_NOFOLLOW rejects a symlink at the FINAL component only. A pre-planted
  // hardlink, or a symlink anywhere in the parent path, still lands this append
  // — which carries full install paths — in a file someone else chose. Confirm
  // after the fact that what we hold is a plain, unshared file we own, and give
  // up (stderr-only) rather than write if it is not. fstat/getuid/close are all
  // on the async-signal-safe list.
  struct stat st;
  if (::fstat(fd, &st) != 0 || !S_ISREG(st.st_mode) || st.st_nlink != 1 ||
      st.st_uid != ::getuid())
  {
    ::close(fd);
    return;
  }

  g_crash_fd = fd;
  g_crash_fd_owned = 1;
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
/// `install_thread_alt_stack()`, which every thread CAMP's own sources start
/// calls on entry — the ROS node thread, `camp::ros::GraphThread`, and each
/// QtConcurrent worker entry point. What remains uncovered is rclcpp's own
/// internal threads, which offer no entry hook; see the enumerated gap on
/// `install_thread_alt_stack()` in `crash_handler.h`.
///
/// **Deliberately leaked at thread exit** — do NOT "fix" this into a
/// `unique_ptr` or give it a destructor. The kernel keeps the `ss_sp` pointer
/// this hands it for the lifetime of the thread, and a `thread_local`
/// destructor runs while the thread is still alive and still able to take a
/// signal. Freeing there would leave the kernel a dangling alternate stack — a
/// use-after-free reachable only from a signal handler, i.e. the least
/// debuggable kind there is. The leak is one buffer per CAMP-started thread,
/// reclaimed at process exit.
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

/// Async-signal-safe SIGNED decimal formatter.
///
/// `siginfo_t::si_code` is an `int` and is **negative** for every
/// user-generated signal (`SI_USER` 0, `SI_QUEUE` -1, `SI_TKILL` -6). Rendering
/// it through `emit_ulong()` printed `18446744073709551610` for exactly the
/// abort-on-close class (#207) this feature targets.
void emit_long(long value)
{
  if (value < 0)
  {
    emit("-");
    // Negate in unsigned space: -LONG_MIN is undefined in signed arithmetic.
    emit_ulong(0UL - static_cast<unsigned long>(value), 10);
  }
  else
  {
    emit_ulong(static_cast<unsigned long>(value), 10);
  }
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

/// Seconds the dump is allowed to take before the process is killed anyway.
constexpr unsigned int kWatchdogSeconds = 10;

/// Bound the handler's own lifetime. Async-signal-safe.
///
/// `backtrace()` / `backtrace_symbols_fd()` take glibc's loader locks, and the
/// durable `write()` can block on a pipe nobody is draining. If another thread
/// is inside a GDAL or Qt-plugin `dlopen()` holding a loader lock, the dump
/// deadlocks and a silent death becomes a silent HANG — strictly worse,
/// because the supervisor never even logs "process has died".
///
/// The bound is only real if SIGALRM can actually terminate us, so this makes
/// that true rather than assuming it. The signal handler's
/// `signal(sig, SIG_DFL)` restores the *faulting* signal's disposition and says
/// nothing about SIGALRM; `on_terminate()` restores nothing at all. So: put
/// SIGALRM back on its default action (terminate the process) and unblock it in
/// this thread's mask, then arm it. Every call here is on the POSIX
/// async-signal-safe list.
void arm_watchdog()
{
  ::signal(SIGALRM, SIG_DFL);
  sigset_t alarm_only;
  ::sigemptyset(&alarm_only);
  ::sigaddset(&alarm_only, SIGALRM);
  ::pthread_sigmask(SIG_UNBLOCK, &alarm_only, nullptr);
  ::alarm(kWatchdogSeconds);
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

  // Bound the handler's own lifetime — see arm_watchdog().
  arm_watchdog();

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
      emit_long(static_cast<long>(info->si_code));

      // si_addr is a fault address ONLY when the kernel generated the signal,
      // which is what `si_code > 0` means. At si_code <= 0 the signal came
      // from kill()/raise()/sigqueue() and the same union member holds
      // si_pid/si_uid — printing that rendered (uid << 32 | pid) as a
      // plausible-looking fault address on every raise()d SIGSEGV. Gating on
      // the signal number alone could not tell the two apart.
      if (info->si_code > 0 &&
          (sig == SIGSEGV || sig == SIGBUS || sig == SIGFPE || sig == SIGILL))
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

  // Re-raise so the exit status is exactly what it would have been — unless the
  // watchdog above fired first, in which case the process dies of SIGALRM (14)
  // instead of the real fault. That is the deliberate trade: a wrong exit status
  // the operator can see beats a hang the supervisor never reports.
  //
  // Note this is the raise() site, not the faulting instruction. Where a core IS
  // written — and it is, on any host whose core ulimit is non-zero; measured
  // `unlimited` on the dev workstation — that core's faulting frame is this
  // handler rather than the real fault. That is a real cost, not a hypothetical
  // one, and it is accepted for two reasons that do not depend on the ulimit:
  //
  //  - Returning instead of re-raising only works for a HARDWARE fault, where
  //    the faulting instruction re-executes and traps again against the
  //    now-restored SIG_DFL. For a signal that was DELIVERED rather than
  //    faulted — kill(1), or the raise() inside abort() on the #207 path —
  //    returning simply resumes the interrupted code. CAMP would survive a
  //    SIGSEGV or SIGABRT in an undefined state instead of dying: strictly worse
  //    than a core whose top frame is wrong.
  //  - Doing it correctly therefore means branching on si_code in the death path
  //    of a diagnostics-only feature.
  //
  // The backtrace above is what this feature delivers, it names the real fault
  // site, and it is identical either way.
  ::raise(sig);
}

void on_terminate()
{
  // The terminate path runs the SAME emit_backtrace() as the signal path and is
  // exposed to the same loader-lock deadlock — but it is NOT downstream of the
  // signal handler's watchdog: it reaches abort() (and hence on_fatal_signal)
  // only *after* the dump it can hang in. Bound it here, at the top, or an
  // uncaught exception thrown while another thread holds a loader lock hangs
  // CAMP forever.
  arm_watchdog();

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

void install_thread_alt_stack()
{
  if (t_alt_stack != nullptr)
    return;

  // SIGSTKSZ is sysconf(_SC_SIGSTKSZ) on glibc >= 2.34, i.e. a function call
  // rather than a constant. Evaluate it once, so the allocation and the size
  // handed to the kernel cannot disagree — and treat its -1 failure return as a
  // failure: a bare cast to size_t turns it into SIZE_MAX, new[] then fails,
  // and the thread silently loses stack-overflow coverage.
  //
  // Floor it too. The pre-2.34 constant was 8 KB, which is tight against this
  // handler's own buffers (a 128-entry frame array plus backtrace_symbols_fd's
  // formatting) and the unwinder that fills them. 64 KB is negligible per
  // thread and always >= MINSIGSTKSZ.
  constexpr size_t kMinAltStackBytes = 64u * 1024u;
  const long reported = static_cast<long>(SIGSTKSZ);
  size_t stack_size = (reported > 0) ? static_cast<size_t>(reported) : 0u;
  if (stack_size < kMinAltStackBytes)
    stack_size = kMinAltStackBytes;

  // Install time, not handler time, so stderr formatting is safe here — and a
  // thread that silently loses coverage is a crash class that quietly stops
  // being reported, which is exactly what #217 exists to end.
  const long tid = static_cast<long>(::syscall(SYS_gettid));

  t_alt_stack = new (std::nothrow) char[stack_size];
  if (t_alt_stack == nullptr)
  {
    ::fprintf(stderr,
              "[camp #217] could not allocate a %zu-byte alternate signal stack "
              "for thread %ld; a stack-overflow SIGSEGV on that thread will not "
              "be reported\n", stack_size, tid);
    return;
  }

  stack_t ss;
  ss.ss_sp = t_alt_stack;
  ss.ss_size = stack_size;
  ss.ss_flags = 0;
  if (::sigaltstack(&ss, nullptr) != 0)
  {
    ::fprintf(stderr,
              "[camp #217] sigaltstack() failed for thread %ld; a "
              "stack-overflow SIGSEGV on that thread will not be reported\n",
              tid);
    // Don't leave the pointer looking like a live alternate stack: a later retry
    // on this thread may succeed.
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
    // Release store, paired with the acquire load in open_crash_fd(): `volatile`
    // orders the compiler, not another CPU, and a handler running on a second
    // thread must never see this flag set over a half-copied path. Both installs
    // happen before CAMP starts a thread today; this file is careful about
    // exactly this class everywhere else.
    __atomic_store_n(&g_crash_path_valid, 1, __ATOMIC_RELEASE);
  }
}

void install_crash_handlers(int backtrace_fd)
{
  // Close a crash file a handler opened under an earlier install. Never close a
  // caller-supplied fd — we do not own it — which is what g_crash_fd_owned
  // distinguishes. Unreachable in main()'s sequence today, but the header
  // advertises this call as safe to repeat.
  if (g_crash_fd_owned && g_crash_fd >= 0)
    ::close(static_cast<int>(g_crash_fd));
  g_crash_fd_owned = 0;

  g_crash_fd = backtrace_fd;
  __atomic_store_n(&g_crash_path_valid, 0, __ATOMIC_RELEASE);
  __atomic_clear(&g_already_dumped, __ATOMIC_RELEASE);

  // Ignore SIGPIPE for the process. Without this, a write to a stderr pipe
  // whose reader has exited kills CAMP outright — changing the exit status the
  // supervisor sees from the real fault (-11) to 13, and truncating the dump
  // partway. CAMP writes nothing else to a pipe it needs SIGPIPE for.
  //
  // Scope, recorded so it is not rediscovered: this is process-wide, it survives
  // execve() into any child CAMP might one day spawn, and it overwrites whatever
  // disposition Qt Network or GDAL's curl established. Nothing under src/ spawns
  // a child today (no QProcess, popen or system) and neither library depends on
  // SIGPIPE killing the process, so it is safe as written.
  if (::signal(SIGPIPE, SIG_IGN) == SIG_ERR)
    ::fprintf(stderr,
              "[camp #217] could not ignore SIGPIPE; a crash dump written to a "
              "closed stderr pipe may kill CAMP mid-dump\n");

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
