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
 * @file time.h
 *
 * MinGW ships <time.h> with the Windows _s variants but not POSIX's
 * _r reentrant forms. The argument orders differ (_s takes buffer first,
 * _r takes the input first) so we can't just #define. Provide thin
 * inline wrappers.
 */
#pragma once

#if defined(_MSC_VER) && !defined(__clang__)
#if defined(__has_include)
#  if __has_include(<../ucrt/time.h>)
#    include <../ucrt/time.h>
#  else
#    include <time.h>
#  endif
#else
#include <time.h>
#endif
#include <sys/types.h>
typedef int clockid_t;
#ifndef CLOCK_REALTIME
#define CLOCK_REALTIME 0
#endif
#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 1
#endif
#ifndef CLOCK_BOOTTIME
#define CLOCK_BOOTTIME CLOCK_MONOTONIC
#endif
#ifndef CLOCK_REALTIME_COARSE
#define CLOCK_REALTIME_COARSE CLOCK_REALTIME
#endif
#ifndef CLOCK_MONOTONIC_COARSE
#define CLOCK_MONOTONIC_COARSE CLOCK_MONOTONIC
#endif
#ifndef _TIMEVAL_DEFINED
#define _TIMEVAL_DEFINED
struct timeval {
	long tv_sec;
	long tv_usec;
};
#endif
#else
#include_next <time.h>
#endif

#ifndef _PX4_TIME_R_SHIM_DEFINED
#define _PX4_TIME_R_SHIM_DEFINED

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_MSC_VER) && !defined(__clang__)
/** @brief POSIX clock_gettime() backed by Windows high-resolution timers. */
int clock_gettime(clockid_t clk_id, struct timespec *tp);

/** @brief POSIX clock_settime() compatibility entry point. */
int clock_settime(clockid_t clk_id, const struct timespec *tp);

/** @brief Fill a POSIX timeval with the current wall-clock time. */
int gettimeofday(struct timeval *tv, void *tz);

/** @brief Sleep for the requested interval, reporting no remaining time. */
int nanosleep(const struct timespec *req, struct timespec *rem);
#endif

/** @brief Thread-safe UTC conversion using the Windows gmtime_s() order. */
static inline struct tm *gmtime_r(const time_t *timep, struct tm *result)
{
	if (!timep || !result) { return NULL; }
	return (gmtime_s(result, timep) == 0) ? result : NULL;
}

/** @brief Thread-safe local-time conversion using the Windows localtime_s(). */
static inline struct tm *localtime_r(const time_t *timep, struct tm *result)
{
	if (!timep || !result) { return NULL; }
	return (localtime_s(result, timep) == 0) ? result : NULL;
}

/** @brief Thread-safe asctime() wrapper; @p buf must hold at least 26 bytes. */
static inline char *asctime_r(const struct tm *tm, char *buf)
{
	if (!tm || !buf) { return NULL; }
	/* POSIX requires a 26-byte buffer; match that to asctime_s. */
	return (asctime_s(buf, 26, tm) == 0) ? buf : NULL;
}

/** @brief Thread-safe ctime() wrapper; @p buf must hold at least 26 bytes. */
static inline char *ctime_r(const time_t *timep, char *buf)
{
	if (!timep || !buf) { return NULL; }
	return (ctime_s(buf, 26, timep) == 0) ? buf : NULL;
}

#ifndef timerisset
#define timerisset(tvp) ((tvp)->tv_sec || (tvp)->tv_usec)
#endif
#ifndef timerclear
#define timerclear(tvp) do { (tvp)->tv_sec = 0; (tvp)->tv_usec = 0; } while (0)
#endif
#ifndef timercmp
#define timercmp(a, b, CMP) (((a)->tv_sec == (b)->tv_sec) ? ((a)->tv_usec CMP (b)->tv_usec) : ((a)->tv_sec CMP (b)->tv_sec))
#endif
#ifndef timeradd
#define timeradd(a, b, result) \
	do { \
		(result)->tv_sec = (a)->tv_sec + (b)->tv_sec; \
		(result)->tv_usec = (a)->tv_usec + (b)->tv_usec; \
		if ((result)->tv_usec >= 1000000) { \
			++(result)->tv_sec; \
			(result)->tv_usec -= 1000000; \
		} \
	} while (0)
#endif
#ifndef timersub
#define timersub(a, b, result) \
	do { \
		(result)->tv_sec = (a)->tv_sec - (b)->tv_sec; \
		(result)->tv_usec = (a)->tv_usec - (b)->tv_usec; \
		if ((result)->tv_usec < 0) { \
			--(result)->tv_sec; \
			(result)->tv_usec += 1000000; \
		} \
	} while (0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* _PX4_TIME_R_SHIM_DEFINED */
