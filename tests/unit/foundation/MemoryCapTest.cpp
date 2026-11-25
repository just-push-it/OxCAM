// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <vector>

// Hidden: run by foundation.memory_cap with a 1 GB cap, which has to stop the process here.
TEST_CASE("the memory cap stops a runaway allocation", "[.][memorycap]") {
    constexpr std::size_t size = std::size_t{3} * 1024 * 1024 * 1024;
    const std::vector<char> runaway(size, 'x');
    CHECK(runaway.size() == size);
    FAIL("the memory cap did not stop the allocation");
}
