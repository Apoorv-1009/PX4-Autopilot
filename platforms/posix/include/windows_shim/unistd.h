/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file unistd.h
 *
 * unistd.h extension shim for PX4 SITL on Windows.
 *
 * MinGW ships a minimal <unistd.h> but omits POSIX helpers that PX4 and
 * third-party code use directly (pipe, fsync, symlink, dprintf, sysconf,
 * process/session helpers, environment setters, hard links).
 * Forward to the real header via #include_next and layer the missing
 * pieces on top using Win32 CRT equivalents (_pipe, _commit, _write)
 * or Win32 APIs (CreateSymbolicLinkA, GetSystemInfo).
 */
#pragma once

#if defined(_MSC_VER) && !defined(__clang__)
#include <sys/types.h>
#else
#include_next <unistd.h>
#endif
#include <io.h>
#include <stdarg.h>
#include <stdio.h>
#include <windows.h>

/* POSIX standard fd numbers - MinGW defines them via io.h but belt-and-suspenders. */
#ifndef STDIN_FILENO
#define STDIN_FILENO  0
#endif
#ifndef STDOUT_FILENO
#define STDOUT_FILENO 1
#endif
#ifndef STDERR_FILENO
#define STDERR_FILENO 2
#endif
#ifndef F_OK
#define F_OK 0
#endif
#ifndef X_OK
#define X_OK 1
#endif
#ifndef W_OK
#define W_OK 2
#endif
#ifndef R_OK
#define R_OK 4
#endif

/* POSIX sysconf selectors that MinGW doesn't ship. Numerical values
 * don't need to match Linux - only our own sysconf() implementation
 * inspects them. */
#ifndef _SC_PAGESIZE
#define _SC_PAGESIZE          30
#endif
#ifndef _SC_PAGE_SIZE
#define _SC_PAGE_SIZE         _SC_PAGESIZE
#endif
#ifndef _SC_CLK_TCK
#define _SC_CLK_TCK           2
#endif
#ifndef _SC_NPROCESSORS_ONLN
#define _SC_NPROCESSORS_ONLN  84
#endif
#ifndef _SC_NPROCESSORS_CONF
#define _SC_NPROCESSORS_CONF  83
#endif
#ifndef _SC_OPEN_MAX
#define _SC_OPEN_MAX          4
#endif
#ifndef _SC_HOST_NAME_MAX
#define _SC_HOST_NAME_MAX     180
#endif
#ifndef _SC_LOGIN_NAME_MAX
#define _SC_LOGIN_NAME_MAX    71
#endif
#ifndef _SC_PHYS_PAGES
#define _SC_PHYS_PAGES        85
#endif
#ifndef _SC_AVPHYS_PAGES
#define _SC_AVPHYS_PAGES      86
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_MSC_VER) && !defined(__clang__)
/** @brief Sleep for at least @p usec microseconds using Windows Sleep(). */
static inline int usleep(useconds_t usec)
{
	Sleep((DWORD)((usec + 999U) / 1000U));
	return 0;
}

/** @brief Sleep for at least @p seconds seconds using Windows Sleep(). */
static inline unsigned int sleep(unsigned int seconds)
{
	Sleep(seconds * 1000U);
	return 0;
}
#endif

/* POSIX pipe(fd[2]) - default to 64 KiB buffer and binary mode. */
#ifndef _PX4_PIPE_SHIM_DEFINED
#define _PX4_PIPE_SHIM_DEFINED
/** @brief Create a binary CRT pipe with a 64 KiB buffer. */
static inline int pipe(int fds[2]) { return _pipe(fds, 65536, 0x8000 /* _O_BINARY */); }

/**
 * @brief pipe2() compatibility wrapper.
 *
 * Windows CRT pipes do not expose POSIX pipe2 flags; the shim accepts the
 * argument and creates the same binary pipe as pipe().
 */
static inline int pipe2(int fds[2], int flags)
{
	(void)flags;
	return pipe(fds);
}
#endif

/* POSIX fsync(fd) - forwards to _commit (flushes the CRT fd's buffers
 * down to the underlying HANDLE and flushes the HANDLE to disk). */
#ifndef _PX4_FSYNC_SHIM_DEFINED
#define _PX4_FSYNC_SHIM_DEFINED
static inline int fsync(int fd) { return _commit(fd); }
#endif

/* fdatasync: Windows has no separate metadata/data flush. Treat it as
 * equivalent to fsync (POSIX allows this; it's just stricter). */
#ifndef _PX4_FDATASYNC_SHIM_DEFINED
#define _PX4_FDATASYNC_SHIM_DEFINED
static inline int fdatasync(int fd) { return _commit(fd); }
#endif

/* POSIX sysconf - implemented in posix_shim.cpp so we can use
 * <windows.h>'s GetSystemInfo / GlobalMemoryStatusEx without forcing
 * every translation unit to drag in <windows.h>. */
#ifndef _PX4_SYSCONF_SHIM_DEFINED
#define _PX4_SYSCONF_SHIM_DEFINED
/**
 * @brief Query host limits for the POSIX selectors PX4 uses.
 *
 * Implemented in the Windows backend so page size, processor count, and memory
 * values come from GetSystemInfo/GlobalMemoryStatusEx.
 */
long sysconf(int name);
#endif

/* POSIX symlink(target, linkpath) - forwards to CreateSymbolicLinkA
 * (requires the SE_CREATE_SYMBOLIC_LINK_NAME privilege or Windows 10
 * developer-mode). Returns -1 with errno = EPERM if the user lacks
 * the privilege. */
#ifndef _PX4_SYMLINK_SHIM_DEFINED
#define _PX4_SYMLINK_SHIM_DEFINED
/**
 * @brief Create a filesystem symbolic link.
 *
 * @return 0 on success, -1 with errno set. EPERM indicates missing Windows
 *         symlink privilege or disabled developer mode.
 */
int symlink(const char *target, const char *linkpath);
#endif

/* POSIX readlink - Windows has no O(1) path-to-reparse-target readout;
 * fall through to DeviceIoControl on the reparse point. The base SITL
 * build doesn't follow symlinks, so the shim lives in posix_shim.cpp. */
#ifndef _PX4_READLINK_SHIM_DEFINED
#define _PX4_READLINK_SHIM_DEFINED
/**
 * @brief Read the target of a filesystem reparse-point symlink.
 *
 * @return Number of bytes copied into @p buf, or -1 with errno set.
 */
ssize_t readlink(const char *path, char *buf, size_t bufsiz);
#endif

/* POSIX truncate(path, length) - forwards to CreateFileA + SetEndOfFile
 * in posix_shim.cpp. ftruncate on a CRT fd uses _chsize_s. */
#ifndef _PX4_TRUNCATE_SHIM_DEFINED
#define _PX4_TRUNCATE_SHIM_DEFINED
/** @brief Truncate a file by path using CreateFileA/SetEndOfFile. */
int truncate(const char *path, off_t length);
#endif

/* POSIX link(oldpath, newpath) - maps onto CreateHardLinkA via
 * posix_shim.cpp. */
#ifndef _PX4_LINK_SHIM_DEFINED
#define _PX4_LINK_SHIM_DEFINED
/** @brief Create a hard link using CreateHardLinkA. */
int link(const char *existing_path, const char *new_path);
#endif

/* POSIX realpath fallback - CRT _fullpath returns the canonical path. */
#ifndef _PX4_REALPATH_SHIM_DEFINED
#define _PX4_REALPATH_SHIM_DEFINED
#include <stdlib.h>
/** @brief Resolve a path to an absolute CRT path using _fullpath(). */
static inline char *realpath(const char *path, char *resolved_path)
{
	return _fullpath(resolved_path, path, resolved_path ? 260 : 0);
}
#endif

/* POSIX dprintf / vdprintf - forward to _write on the CRT fd. */
#ifndef _PX4_DPRINTF_SHIM_DEFINED
#define _PX4_DPRINTF_SHIM_DEFINED
/** @brief Formatted write to a CRT file descriptor. */
static inline int vdprintf(int fd, const char *fmt, va_list ap)
{
	char buf[1024];
	int n = vsnprintf(buf, sizeof(buf), fmt, ap);
	if (n <= 0) { return n; }
	if ((size_t)n >= sizeof(buf)) { n = (int)sizeof(buf) - 1; }
	return (int)_write(fd, buf, (unsigned)n);
}

/** @brief Variadic formatted write to a CRT file descriptor. */
static inline int dprintf(int fd, const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	int n = vdprintf(fd, fmt, ap);
	va_end(ap);
	return n;
}
#endif

#ifndef TEMP_FAILURE_RETRY
#define TEMP_FAILURE_RETRY(expression) (expression)
#endif

/* POSIX getpagesize - Windows has GetSystemInfo.dwPageSize (typically
 * 4096 on x86/x64, 16384 on ARM64). */
#ifndef _PX4_GETPAGESIZE_SHIM_DEFINED
#define _PX4_GETPAGESIZE_SHIM_DEFINED
/** @brief Return the host memory page size in bytes. */
int getpagesize(void);
#endif

/* POSIX/BSD process and environment helpers that MinGW does not
 * declare. These are implemented in posix_shim.cpp with Windows-backed
 * compatibility semantics suitable for PX4 and embedded host apps. */
#ifndef _PX4_GETPPID_SHIM_DEFINED
#define _PX4_GETPPID_SHIM_DEFINED
/** @brief Return the parent process id when Windows can discover it. */
pid_t getppid(void);
#endif

#ifndef _PX4_SETSID_SHIM_DEFINED
#define _PX4_SETSID_SHIM_DEFINED
/** @brief Create a best-effort process session; returns the current pid. */
pid_t setsid(void);
#endif

#ifndef _PX4_GETSID_SHIM_DEFINED
#define _PX4_GETSID_SHIM_DEFINED
/** @brief Return the best-effort session id for @p pid. */
pid_t getsid(pid_t pid);
#endif

#ifndef _PX4_DAEMON_SHIM_DEFINED
#define _PX4_DAEMON_SHIM_DEFINED
/**
 * @brief daemon() compatibility shim.
 *
 * Windows cannot fork and detach in the POSIX sense; the implementation applies
 * the requested cwd/stdio behavior where possible and otherwise reports
 * success for callers that only need a background-compatible code path.
 */
int daemon(int nochdir, int noclose);
#endif

#ifndef _PX4_SETENV_SHIM_DEFINED
#define _PX4_SETENV_SHIM_DEFINED
/** @brief Set an environment variable using the Windows CRT environment. */
int setenv(const char *name, const char *value, int overwrite);
#endif

#ifndef _PX4_UNSETENV_SHIM_DEFINED
#define _PX4_UNSETENV_SHIM_DEFINED
/** @brief Remove an environment variable using the Windows CRT environment. */
int unsetenv(const char *name);
#endif

#ifdef __cplusplus
}
#endif
