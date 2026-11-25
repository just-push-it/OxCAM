// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <vector>

// Hidden: run by foundation.memory_cap with a 1 GB cap, which has to stop the process here.
TEST_CASE("the memory cap stops a runaway allocation", "[.][memorycap]") {
    constexpr std::size_t size = std::size_t{3} * 1024 * 1024 * 1024;
    const std::vector<char> runaway(size, 'x');
    // Clang deletes a new-expression whose bytes it can see are never read, including a single
    // volatile load of a fill byte. The asm makes the allocation observable.
#if defined(__GNUC__) || defined(__clang__)
    asm volatile("" : : "r"(runaway.data()), "r"(runaway.size()) : "memory");
#else
    const volatile char touched = runaway[size / 2];
    CHECK(touched == 'x');
#endif
    FAIL("the memory cap did not stop the allocation");
}
