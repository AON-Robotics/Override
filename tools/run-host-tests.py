"""Run the host suite with CXX, a PATH compiler, or an installed MSVC toolchain."""
import argparse
import os
import runpy
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def compiler():
    env = os.environ.copy()
    if env.get("CXX"):
        return [part.strip('"') for part in shlex.split(env["CXX"], posix=os.name != "nt")], env
    for name in ("clang++", "g++", "cl"):
        if shutil.which(name):
            return [shutil.which(name)], env
    if os.name == "nt":
        vswhere = shutil.which("vswhere")
        if not vswhere:
            installer = Path(env.get("ProgramFiles(x86)", "C:/Program Files (x86)")) / "Microsoft Visual Studio/Installer/vswhere.exe"
            if installer.is_file():
                vswhere = str(installer)
        if vswhere:
            install = subprocess.check_output(
                [vswhere, "-latest", "-products", "*",
                 "-requires", "Microsoft.VisualStudio.Component.VC.Tools.x86.x64",
                 "-property", "installationPath"],
                text=True).strip()
            if install:
                vcvars = Path(install) / "VC" / "Auxiliary" / "Build" / "vcvars64.bat"
                output = subprocess.check_output(
                    f'"{env.get("COMSPEC", "cmd.exe")}" /d /s /c ""{vcvars}" >nul && set"',
                    text=True)
                # Windows environment keys are case-insensitive.
                env = {k.upper(): v for k, v in env.items()}
                env.update((key.upper(), value) for line in output.splitlines()
                           if "=" in line and not line.startswith("=")
                           for key, value in [line.split("=", 1)])
                cl = shutil.which("cl", path=env.get("PATH"))
                if cl:
                    return [cl], env
    raise RuntimeError("No C++17 compiler found; install clang++, g++, or MSVC, or set CXX.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--python-only", action="store_true")
    args = parser.parse_args()
    for test in sorted((ROOT / "tests").glob("*-test.py")):
        subprocess.run([os.sys.executable, str(test)], cwd=ROOT, check=True)
    if args.python_only:
        return
    command, env = compiler()
    with tempfile.TemporaryDirectory(prefix="aon-host-") as folder:
        build = Path(folder)
        exports = build / "exports"
        exports.mkdir()
        for route in (ROOT / "static").glob("*.jerryio.txt"):
            shutil.copyfile(route, exports / route.name)
        fixture = runpy.run_path(str(ROOT / "tests/static-path-generator-test.py"))["corner_export"]
        (exports / "__host_corner.jerryio.txt").write_text(fixture(), encoding="utf-8")
        subprocess.run([os.sys.executable, str(ROOT / "tools/generate-static-path.py"),
                        str(exports), str(build / "aon/generated/static-path.hpp")], check=True)
        executable = build / ("host-tests.exe" if os.name == "nt" else "host-tests")
        source = str(ROOT / "tests/host-tests.cpp")
        if Path(command[0]).stem.lower() in ("cl", "clang-cl"):
            flags = ["/nologo", "/std:c++17", "/EHsc", "/W4", "/WX",
                     "/wd4100", "/wd4244", "/wd4458", "/wd4456", "/wd5038",
                     "/I" + str(build), "/I" + str(ROOT / "include"), source, "/Fe:" + str(executable)]
        else:
            flags = ["-std=c++17", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter", "-Wno-reorder",
                     "-I" + str(build), "-I" + str(ROOT / "include"), source, "-o", str(executable)]
        subprocess.run(command + flags, env=env, cwd=build, check=True)
        subprocess.run([str(executable)], cwd=build, check=True)
    print("Host suite passed")


if __name__ == "__main__":
    main()
