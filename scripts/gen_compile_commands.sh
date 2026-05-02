#!/bin/bash
# Generate compile_commands.json using Hedron's extractor.
#
# Because we have a default STM32 platform set in .bazelrc, we must:
# 1. Force the generator tool to run on the host using --platforms=
# 2. Pass the target platform to the analyzer using -- --platforms=//platforms:stm32g4

bazel run --platforms= //:refresh_compile_commands -- --platforms=//platforms:stm32g4
