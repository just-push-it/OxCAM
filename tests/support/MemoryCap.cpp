// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Caps the memory of a process that runs tests, so a runaway test fails on its own instead of
// taking the whole machine with it. Compiled into the test programs and the apps; it takes effect
// before main. The cap is OCAM_MEMORY_CAP_GB in the environment, or else
// OCAM_MEMORY_CAP_DEFAULT_GB from the build; 0 means no cap. The apps build with a default of 0,
// so only their tests are capped, never a user's session.

#include <cstdio>
#include <cstdlib>
#include <new>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#elif defined(__linux__)
#include <sys/resource.h>
#endif

#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
#define OCAM_MEMORY_CAP_SANITIZED 1
#elif defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer)
#define OCAM_MEMORY_CAP_SANITIZED 1
#endif
#endif

namespace {

double capGb = 0.0;

double requestedCapGb() {
#ifdef _WIN32
    char text[32];
    const DWORD length = GetEnvironmentVariableA("OCAM_MEMORY_CAP_GB", text, sizeof(text));
    const char* value = length > 0 && length < sizeof(text) ? text : nullptr;
#else
    const char* value = std::getenv("OCAM_MEMORY_CAP_GB");
#endif
    if (value) {
        char* end = nullptr;
        const double gb = std::strtod(value, &end);
        if (end != value && gb >= 0.0) {
            return gb;
        }
    }
    return OCAM_MEMORY_CAP_DEFAULT_GB;
}

/// The allocation failed: say why, rather than leave a test to fail in some unrelated way.
void capReached() {
    std::fprintf(stderr,
                 "Test stopped: it reached the memory cap of %g GB (OCAM_MEMORY_CAP_GB). This "
                 "usually means a runaway allocation.\n",
                 capGb);
    std::fflush(stderr);
    std::_Exit(99);
}

bool applyCap() {
    capGb = requestedCapGb();
    if (capGb <= 0.0) {
        return false;
    }
#if defined(OCAM_MEMORY_CAP_SANITIZED)
    return false; // sanitizers reserve far more address space than the program uses
#elif defined(_WIN32)
    const HANDLE job = CreateJobObjectW(nullptr, nullptr); // stays open: the job holds the limit
    if (!job) {
        return false;
    }
    JOBOBJECT_EXTENDED_LIMIT_INFORMATION limits{};
    limits.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_PROCESS_MEMORY;
    limits.ProcessMemoryLimit = static_cast<SIZE_T>(capGb * 1024.0 * 1024.0 * 1024.0);
    if (!SetInformationJobObject(job, JobObjectExtendedLimitInformation, &limits, sizeof(limits)) ||
        !AssignProcessToJobObject(job, GetCurrentProcess())) {
        return false;
    }
    std::set_new_handler(capReached);
    return true;
#elif defined(__linux__)
    const auto bytes = static_cast<rlim_t>(capGb * 1024.0 * 1024.0 * 1024.0);
    const rlimit limit{bytes, bytes};
    if (setrlimit(RLIMIT_AS, &limit) != 0) {
        return false;
    }
    std::set_new_handler(capReached);
    return true;
#else
    return false;
#endif
}

const bool capApplied = applyCap();

} // namespace
