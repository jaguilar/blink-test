load(":cc_toolchain_config.bzl", "cc_toolchain_config")
load("@rules_cc//cc:defs.bzl", "cc_toolchain")

package(default_visibility = ["//visibility:public"])

filegroup(name = "empty")

cc_toolchain_config(name = "arm_none_eabi_toolchain_config")

cc_toolchain(
    name = "arm_none_eabi_cc_toolchain",
    toolchain_identifier = "arm-none-eabi-toolchain",
    toolchain_config = ":arm_none_eabi_toolchain_config",
    all_files = ":empty",
    compiler_files = ":empty",
    dwp_files = ":empty",
    linker_files = ":empty",
    objcopy_files = ":empty",
    strip_files = ":empty",
    supports_param_files = 0,
)

toolchain(
    name = "arm_none_eabi_toolchain",
    exec_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
    target_compatible_with = [
        "@platforms//os:none",
        "@platforms//cpu:arm",
    ],
    toolchain = ":arm_none_eabi_cc_toolchain",
    toolchain_type = "@bazel_tools//tools/cpp:toolchain_type",
)
