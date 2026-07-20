load("@bazel_tools//tools/cpp:cc_toolchain_config_lib.bzl", "feature", "flag_group", "flag_set", "tool_path")
load("@bazel_tools//tools/build_defs/cc:action_names.bzl", "ACTION_NAMES")
load("@rules_cc//cc:defs.bzl", "CcToolchainConfigInfo", "cc_common")
load("@arm_none_eabi_config//:paths.bzl", "SYS_INCLUDE_DIRS", "TOOL_PATHS")

def _impl(ctx):
    tool_paths = [
        tool_path(name = "gcc", path = TOOL_PATHS["gcc"]),
        tool_path(name = "cpp", path = TOOL_PATHS["cpp"]),
        tool_path(name = "ar", path = TOOL_PATHS["ar"]),
        tool_path(name = "nm", path = TOOL_PATHS["nm"]),
        tool_path(name = "ld", path = TOOL_PATHS["ld"]),
        tool_path(name = "objcopy", path = TOOL_PATHS["objcopy"]),
        tool_path(name = "objdump", path = TOOL_PATHS["objdump"]),
        tool_path(name = "strip", path = TOOL_PATHS["strip"]),
        tool_path(name = "gcov", path = TOOL_PATHS["gcov"]),
    ]

    target_flags = [
        "-mcpu=cortex-m4",
        "-mfpu=fpv4-sp-d16",
        "-mfloat-abi=hard",
        "-Wdouble-promotion",
        "-Wfloat-conversion",
        "-Wall",
        "-fdata-sections",
        "-ffunction-sections",
        "-fstack-usage",
    ]

    c_flags = target_flags + ["-std=c11"]
    cxx_flags = target_flags + [
        "-std=c++20",
        "-fno-rtti",
        "-fno-exceptions",
        "-fno-threadsafe-statics",
    ]

    linker_flags = [
        "-mcpu=cortex-m4",
        "-mfpu=fpv4-sp-d16",
        "-mfloat-abi=hard",
        "--specs=nano.specs",
        "-u",
        "_printf_float",
        "-Wl,-Map=blink-test.map",
        "-Wl,--cref",
        "-Wl,--gc-sections",
        "-Wl,--print-memory-usage",
        "-flto",
        "-lm",
        "-lstdc++",
    ]

    if ctx.file.linker_script:
        linker_flags.extend(["-T", ctx.file.linker_script.path])

    features = [
        feature(
            name = "default_compiler_flags",
            enabled = True,
            flag_sets = [
                flag_set(
                    actions = [ACTION_NAMES.c_compile],
                    flag_groups = [flag_group(flags = c_flags)],
                ),
                flag_set(
                    actions = [ACTION_NAMES.cpp_compile],
                    flag_groups = [flag_group(flags = cxx_flags)],
                ),
                flag_set(
                    actions = [ACTION_NAMES.assemble, ACTION_NAMES.preprocess_assemble],
                    flag_groups = [flag_group(flags = target_flags + ["-x", "assembler-with-cpp"])],
                ),
            ],
        ),
        feature(
            name = "default_linker_flags",
            enabled = True,
            flag_sets = [
                flag_set(
                    actions = [ACTION_NAMES.cpp_link_executable],
                    flag_groups = [flag_group(flags = linker_flags)],
                ),
            ],
        ),
    ]

    return cc_common.create_cc_toolchain_config_info(
        ctx = ctx,
        toolchain_identifier = "arm-none-eabi-toolchain",
        host_system_name = "local",
        target_system_name = "arm-none-eabi",
        target_cpu = "arm",
        target_libc = "unknown",
        compiler = "gcc",
        abi_version = "unknown",
        abi_libc_version = "unknown",
        tool_paths = tool_paths,
        features = features,
        cxx_builtin_include_directories = SYS_INCLUDE_DIRS,
    )

cc_toolchain_config = rule(
    implementation = _impl,
    attrs = {
        "linker_script": attr.label(allow_single_file = True),
    },
    provides = [CcToolchainConfigInfo],
)
