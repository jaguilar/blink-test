def _detect_impl(repository_ctx):
    # Search for arm-none-eabi-gcc on host PATH
    gcc_path = repository_ctx.which("arm-none-eabi-gcc")
    if not gcc_path:
        gcc_path = repository_ctx.which("arm-none-eabi-gcc.exe")

    if not gcc_path:
        fail("\n\nCould not find 'arm-none-eabi-gcc' in the system PATH.\n" +
             "Please make sure the Arm GNU Toolchain is installed and added to your system PATH.\n\n")

    gcc_dir = gcc_path.dirname
    extension = ".exe" if gcc_path.basename.lower().endswith(".exe") else ""

    # Map of tool names to their paths on the host
    tool_names = ["gcc", "cpp", "ar", "nm", "ld", "objcopy", "objdump", "strip", "gcov"]
    paths = {}
    for t in tool_names:
        exe_base = "arm-none-eabi-"
        if t == "gcc":
            exe_base += "gcc"
        elif t == "cpp":
            exe_base += "cpp"
        elif t == "ar":
            exe_base += "gcc-ar"
        elif t == "nm":
            exe_base += "gcc-nm"
        elif t == "ld":
            exe_base += "ld"
        elif t == "objcopy":
            exe_base += "objcopy"
        elif t == "objdump":
            exe_base += "objdump"
        elif t == "strip":
            exe_base += "strip"
        elif t == "gcov":
            exe_base += "gcov"

        exe_path = gcc_dir.get_child(exe_base + extension)
        paths[t] = str(exe_path).replace("\\", "/")

    # Create empty source file to run preprocessor
    repository_ctx.file("empty.cc", "")
    result = repository_ctx.execute([str(gcc_path), "-E", "-Wp,-v", "-xc++", "empty.cc"])

    # Parse system include directories from compiler's verbose preprocessor output
    lines = result.stderr.replace("\r\n", "\n").split("\n")
    include_dirs = []
    in_search_list = False
    for line in lines:
        line = line.strip()
        if line == "#include <...> search starts here:":
            in_search_list = True
            continue
        if line == "End of search list.":
            in_search_list = False
            continue
        if in_search_list:
            normalized = line.replace("\\", "/")
            if normalized.endswith(" (framework directory)"):
                normalized = normalized[:-len(" (framework directory)")]
            include_dirs.append(normalized)

    # Fallback to some standard directories if search list is empty
    if not include_dirs:
        # Fallback for Linux
        include_dirs = [
            "/usr/lib/gcc/arm-none-eabi/14.2.1/include",
            "/usr/lib/gcc/arm-none-eabi/14.2.1/include-fixed",
            "/usr/include/newlib",
            "/usr/arm-none-eabi/include",
        ]

    # Generate the paths.bzl containing compiler metadata
    bzl_content = "TOOL_PATHS = {\n"
    for k, v in paths.items():
        bzl_content += "    \"%s\": \"%s\",\n" % (k, v)
    bzl_content += "}\n\n"
    bzl_content += "SYS_INCLUDE_DIRS = [\n"
    for d in include_dirs:
        bzl_content += "    \"%s\",\n" % d
    bzl_content += "]\n"

    repository_ctx.file("paths.bzl", bzl_content)
    repository_ctx.file("BUILD", "exports_files(['paths.bzl'])")

detect_arm_none_eabi_toolchain = repository_rule(
    implementation = _detect_impl,
    local = True,
)

def _arm_toolchain_ext_impl(ctx):
    detect_arm_none_eabi_toolchain(name = "arm_none_eabi_config")

arm_toolchain_ext = module_extension(
    implementation = _arm_toolchain_ext_impl,
)
