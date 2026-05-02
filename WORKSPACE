workspace(name = "blink_test")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# rules_cc
http_archive(
    name = "rules_cc",
    sha256 = "203787c9f9c95fe0d30ca5cfc4d86c836a6a70ee90d1084fcaaccc286ccf5ca2",
    strip_prefix = "rules_cc-0.0.9",
    urls = ["https://github.com/bazelbuild/rules_cc/releases/download/0.0.9/rules_cc-0.0.9.tar.gz"],
)

# Eigen
http_archive(
    name = "eigen",
    build_file_content = """
cc_library(
    name = "eigen",
    hdrs = glob(["Eigen/**", "unsupported/Eigen/**"]),
    includes = ["."],
    visibility = ["//visibility:public"],
)
""",
    strip_prefix = "eigen-3.4.0",
    urls = ["https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz"],
)

# rules_foreign_cc for foreign repos
http_archive(
    name = "rules_foreign_cc",
    sha256 = "476303bd57a0ca239b0091ed6b027a6e695a762a5474463d276eb2e73d705302",
    strip_prefix = "rules_foreign_cc-0.10.1",
    url = "https://github.com/bazelbuild/rules_foreign_cc/archive/0.10.1.tar.gz",
)

load("@rules_foreign_cc//foreign_cc:repositories.bzl", "rules_foreign_cc_dependencies")
rules_foreign_cc_dependencies()

# Foreign repos in lib directory
# We'll use new_local_repository for these so we can use rules_foreign_cc or just wrap them.
# The user said "use the external tooling path", which I take to mean rules_foreign_cc.

new_local_repository(
    name = "arduino_foc",
    path = "lib/Arduino-FOC",
    build_file = "//third_party:arduino_foc.BUILD",
)

new_local_repository(
    name = "lwrb",
    path = "lib/lwrb",
    build_file = "//third_party:lwrb.BUILD",
)

new_local_repository(
    name = "stm32_arduino_shim",
    path = "lib/stm32_arduino_shim",
    build_file = "//third_party:stm32_arduino_shim.BUILD",
)

register_toolchains(
    "//toolchain:arm_none_eabi_toolchain",
)
