/*
 * Copyright 2026 OmniLink
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * UNIX_TIMESTAMP -- the moment this translation unit was compiled, as an
 * unsigned long count of seconds since 1970-01-01 00:00:00, computed entirely
 * by the preprocessor from the standard __DATE__ and __TIME__ macros.
 *
 * The whole expression is an integer constant expression, so it is usable in a
 * case label, an array bound, a static_assert or a constexpr initialiser --
 * nothing here reaches the runtime.
 *
 * Caveat inherited from the C standard, not from this implementation:
 * __DATE__ / __TIME__ carry the compiling machine's LOCAL wall clock. The value
 * is therefore a UTC-shaped integer built from local fields, and equals the true
 * UTC epoch only when the build machine runs on UTC. Treat it as a build
 * fingerprint, not as an authoritative instant.
 *
 * Provenance: this is an OmniLink clean-room implementation. It replaces an
 * earlier vendored header of the same name that was assembled from unlicensed
 * Stack Overflow answers (Stack Overflow contributions are CC BY-SA, whose
 * share-alike and attribution terms are incompatible with redistribution inside
 * this Apache-2.0 tree). The public macro surface below is deliberately
 * unchanged so that any out-of-tree caller keeps compiling; the derivation is
 * entirely new.
 */

#ifndef COMPILE_TIME_H_
#define COMPILE_TIME_H_

/* Public convenience constants, retained from the previous public surface. */
#define SEC_PER_MIN 60UL
#define SEC_PER_HOUR 3600UL
#define SEC_PER_DAY 86400UL
#define SEC_PER_YEAR (SEC_PER_DAY * 365UL)

/*
 * --- field extraction ------------------------------------------------------
 *
 * __TIME__ is always "hh:mm:ss".
 * __DATE__ is always "Mmm dd yyyy", where dd is SPACE-padded for days 1-9
 * ("Apr  1 2026"). OM_CT_DIGIT therefore maps any non-digit to zero, which is
 * exactly the right answer for that pad character.
 */
#define OM_CT_DIGIT(s, i) ((s)[i] >= '0' && (s)[i] <= '9' ? (unsigned long)((s)[i] - '0') : 0UL)
#define OM_CT_NUM2(s, i) (OM_CT_DIGIT(s, i) * 10UL + OM_CT_DIGIT(s, (i) + 1))
#define OM_CT_NUM4(s, i) (OM_CT_NUM2(s, i) * 100UL + OM_CT_NUM2(s, (i) + 2))

/*
 * Month name -> 1..12.
 *
 * The twelve three-letter English month abbreviations are separated by the
 * plain sum of their 2nd and 3rd characters -- all twelve sums are distinct, so
 * one addition and one comparison chain suffice and no string comparison is
 * needed:
 *
 *   Feb 199   Dec 200   Jan 207   Mar 211   Sep 213   Oct 215
 *   May 218   Aug 220   Jul 225   Apr 226   Jun 227   Nov 229
 */
#define OM_CT_MONTH_KEY(s) ((unsigned long)(s)[1] + (unsigned long)(s)[2])
#define OM_CT_MONTH(s)                                                                                                 \
  (OM_CT_MONTH_KEY(s) == 207UL ?                                                                                       \
     1UL :                                                                                                             \
     OM_CT_MONTH_KEY(s) == 199UL ?                                                                                     \
     2UL :                                                                                                             \
     OM_CT_MONTH_KEY(s) == 211UL ?                                                                                     \
     3UL :                                                                                                             \
     OM_CT_MONTH_KEY(s) == 226UL ?                                                                                     \
     4UL :                                                                                                             \
     OM_CT_MONTH_KEY(s) == 218UL ?                                                                                     \
     5UL :                                                                                                             \
     OM_CT_MONTH_KEY(s) == 227UL ?                                                                                     \
     6UL :                                                                                                             \
     OM_CT_MONTH_KEY(s) == 225UL ?                                                                                     \
     7UL :                                                                                                             \
     OM_CT_MONTH_KEY(s) == 220UL ?                                                                                     \
     8UL :                                                                                                             \
     OM_CT_MONTH_KEY(s) == 213UL ?                                                                                     \
     9UL :                                                                                                             \
     OM_CT_MONTH_KEY(s) == 215UL ? 10UL : OM_CT_MONTH_KEY(s) == 229UL ? 11UL : OM_CT_MONTH_KEY(s) == 200UL ? 12UL : 0UL)

/* Public field accessors, retained from the previous public surface. */
#define __TIME_SECONDS__ OM_CT_NUM2(__TIME__, 6)
#define __TIME_MINUTES__ OM_CT_NUM2(__TIME__, 3)
#define __TIME_HOURS__ OM_CT_NUM2(__TIME__, 0)
#define __TIME_DAYS__ OM_CT_NUM2(__DATE__, 4)
#define __TIME_MONTH__ OM_CT_MONTH(__DATE__)
#define __TIME_YEARS__ OM_CT_NUM4(__DATE__, 7)

/*
 * --- calendar --------------------------------------------------------------
 *
 * Days elapsed before 1 January of a given proleptic-Gregorian year, counted
 * from the epoch year: 365 per elapsed year, plus one for every leap year
 * strictly between 1969 and y. OM_CT_LEAPS_THROUGH(n) counts the leap years in
 * [1, n], so the difference of the two counts is the correction term.
 */
#define OM_CT_LEAPS_THROUGH(y) ((y) / 4UL - (y) / 100UL + (y) / 400UL)
#define OM_CT_DAYS_BEFORE_YEAR(y) \
  (365UL * ((y) - 1970UL) + OM_CT_LEAPS_THROUGH((y) - 1UL) - OM_CT_LEAPS_THROUGH(1969UL))

#define OM_CT_IS_LEAP(y) ((((y) % 4UL == 0UL) && ((y) % 100UL != 0UL)) || ((y) % 400UL == 0UL))

/* Cumulative days before month m in a common (non-leap) year. */
#define OM_CT_DAYS_BEFORE_MONTH(m)                                                                                     \
  ((m) == 1UL ?                                                                                                        \
     0UL :                                                                                                             \
     (m) == 2UL ?                                                                                                      \
     31UL :                                                                                                            \
     (m) == 3UL ?                                                                                                      \
     59UL :                                                                                                            \
     (m) == 4UL ?                                                                                                      \
     90UL :                                                                                                            \
     (m) == 5UL ?                                                                                                      \
     120UL :                                                                                                           \
     (m) == 6UL ?                                                                                                      \
     151UL :                                                                                                           \
     (m) == 7UL ?                                                                                                      \
     181UL :                                                                                                           \
     (m) == 8UL ? 212UL : (m) == 9UL ? 243UL : (m) == 10UL ? 273UL : (m) == 11UL ? 304UL : (m) == 12UL ? 334UL : 0UL)

/* Days elapsed since 1 January of year y -- zero on 1 January. */
#define OM_CT_YDAY(y, m, d) (OM_CT_DAYS_BEFORE_MONTH(m) + (((m) > 2UL && OM_CT_IS_LEAP(y)) ? 1UL : 0UL) + ((d) - 1UL))

/* Seconds since the epoch for a fully decomposed date/time. */
#define OM_CT_EPOCH(y, m, d, hh, mm, ss) \
  ((OM_CT_DAYS_BEFORE_YEAR(y) + OM_CT_YDAY(y, m, d)) * SEC_PER_DAY + (hh) * SEC_PER_HOUR + (mm) * SEC_PER_MIN + (ss))

/* The build timestamp. */
#define UNIX_TIMESTAMP \
  OM_CT_EPOCH(__TIME_YEARS__, __TIME_MONTH__, __TIME_DAYS__, __TIME_HOURS__, __TIME_MINUTES__, __TIME_SECONDS__)

#endif
