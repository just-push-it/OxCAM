# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

ocam_add_test(sim SOURCES BackEdgeSimTest.cpp SimTest.cpp ProbeSimTest.cpp LatheTest.cpp LatheMachineTest.cpp MachineCheckTest.cpp TiltedTest.cpp
    SurfaceTest.cpp
    FixtureCheckTest.cpp DEPS ocam::sim ocam::strategies)
# The throughput test measures time: alone, not next to the parallel tests.
add_test(NAME sim.simulation_throughput COMMAND test_sim "simulation throughput")
set_tests_properties(sim.simulation_throughput PROPERTIES RUN_SERIAL TRUE LABELS "sim;perf")
