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

// Test-process-only CLOCK_REALTIME adjustment; never changes the OS clock.
#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdatomic.h>
#include <time.h>

static _Atomic long offset_ns = 0;

void set_realtime_offset(long value) {
  atomic_store(&offset_ns, value);
}

int clock_gettime(clockid_t id, struct timespec *ts) {
  int (*real_clock_gettime)(clockid_t, struct timespec *) = dlsym(RTLD_NEXT, "clock_gettime");
  int status = real_clock_gettime(id, ts);
  if (!status && id == CLOCK_REALTIME) {
    long ns = ts->tv_nsec + atomic_load(&offset_ns);
    while (ns < 0) {
      --ts->tv_sec;
      ns += 1000000000L;
    }
    while (ns >= 1000000000L) {
      ++ts->tv_sec;
      ns -= 1000000000L;
    }
    ts->tv_nsec = ns;
  }
  return status;
}
