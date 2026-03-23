#!/usr/bin/env python3

"""
repack.py
------------------------------------------------------------------------------

    Simple JAR repacker
    
    Searches for all *.java files in current directory, then compiles them 
    with `javac -cp` referencing the specified BASE_JAR. 
    
    Artifacts are written to a temp dir.  
    The extracted class path is used to copy them back to DESTINATION_JAR.
    
    - If DESTINATION_JAR is a directory, the artifacts will be copied into it,
      then final JAR will be produced by zipping this directory and placing the
      resulting JAR in /output.
      
    - If DESTINATION_JAR is an existing JAR, the same thing will happen but via
      a temporary directory, with the result replacing this JAR.
"""

import sys
import re
import shutil
import subprocess
import tempfile
import zipfile
from dataclasses import dataclass
from pathlib import Path

# Edit these, obviously
BASE_JAR = "Digital_original.jar"
DESTINATION_JAR = "F:/digital/Digital.jar"

SHEBANG_RE = re.compile(r"^//#!\s*(.+?)\s*$")
PACKAGE_RE = re.compile(r"^\s*package\s+([a-zA-Z_][a-zA-Z0-9_\.]*)\s*;")
PUBLIC_CLASS_RE = re.compile(
    r"\bpublic\s+(?:class|interface|enum|record)\s+([A-Za-z_][A-Za-z0-9_]*)\b"
)
TOPLEVEL_CLASS_RE = re.compile(
    r"\b(?:class|interface|enum|record)\s+([A-Za-z_][A-Za-z0-9_]*)\b"
)


@dataclass
class JavaSourceInfo:
    source_path: Path
    target_path_in_jar: Path
    package_path: Path
    main_type_name: str


def read_source_info(java_file: Path) -> JavaSourceInfo:
    """
    Parse basic info (package, class, path override) from a Java source file.
    """
    package_name = None
    main_type_name = None
    shebang_path = None

    text = java_file.read_text(encoding="utf-8")
    lines = text.splitlines()

    # If we want to overwrite the class path, this supports a hacky notation where you
    # use a comment followed by a shebang followed by the desired destination path.
    # so like:  //#! overridden/class/path
    # if omitted, the path is determined by the 'package' path. 
    if lines:
        m = SHEBANG_RE.match(lines[0])
        if m:
            shebang_path = m.group(1).strip().strip("/")

    for line in lines:
        if package_name is None:
            m = PACKAGE_RE.match(line)
            if m:
                package_name = m.group(1)

        if main_type_name is None:
            m = PUBLIC_CLASS_RE.search(line)
            if m:
                main_type_name = m.group(1)

    if main_type_name is None:
        m = TOPLEVEL_CLASS_RE.search(text)
        if m:
            main_type_name = m.group(1)

    if main_type_name is None:
        main_type_name = java_file.stem

    package_path = Path(*(package_name.split("."))) if package_name else Path()

    if shebang_path is not None:
        target_path_in_jar = Path(shebang_path)
    else:
        target_path_in_jar = package_path

    return JavaSourceInfo(
        source_path=java_file,
        target_path_in_jar=target_path_in_jar,
        package_path=package_path,
        main_type_name=main_type_name,
    )


def compile_and_collect(java_infos: list[JavaSourceInfo], build_dir: Path):
    cmd = ["javac", "-cp", BASE_JAR, "-d", str(build_dir)]
    cmd.extend(str(info.source_path) for info in java_infos)

    print("Running:")
    print("  " + " ".join(cmd))

    try:
        result = subprocess.run(cmd, check=False)
    except FileNotFoundError:
        raise RuntimeError("javac not found in PATH.")

    if result.returncode != 0:
        raise RuntimeError(f"javac failed with exit code {result.returncode}.")

    artifact_map = {}
    for info in java_infos:
        artifacts = find_compiled_artifacts(info, build_dir)
        if not artifacts:
            raise RuntimeError(f"No artifacts for {info.source_path}")
        artifact_map[info.source_path] = artifacts

    return artifact_map


def find_compiled_artifacts(info: JavaSourceInfo, build_dir: Path) -> list[Path]:
    package_dir = build_dir / info.package_path
    if not package_dir.exists():
        return []

    pattern = f"{info.main_type_name}*.class"
    return sorted(
        p for p in package_dir.glob(pattern)
        if p.is_file() and (
            p.name == f"{info.main_type_name}.class"
            or p.name.startswith(f"{info.main_type_name}$")
        )
    )


def print_artifacts(java_infos, artifact_map, build_dir):
    print("\nCompiled artifacts:")
    for info in java_infos:
        print(f"\n  {info.source_path}:")
        for a in artifact_map[info.source_path]:
            print(f"    {a.relative_to(build_dir)}")


def copy_to_directory(java_infos, artifact_map):
    root = Path(DESTINATION_JAR)

    for info in java_infos:
        dest_dir = root / info.target_path_in_jar
        dest_dir.mkdir(parents=True, exist_ok=True)

        print(f"\nCopying to dir: {dest_dir}")
        for artifact in artifact_map[info.source_path]:
            dest = dest_dir / artifact.name
            shutil.copy2(artifact, dest)
            print(f"  {artifact.name}")


def copy_into_jar(java_infos, artifact_map):
    jar_path = Path(DESTINATION_JAR)

    print(f"\nRebuilding JAR: {jar_path}")

    replacements = {}

    for info in java_infos:
        for artifact in artifact_map[info.source_path]:
            arcname = (info.target_path_in_jar / artifact.name).as_posix()
            replacements[arcname] = artifact

    temp_jar = jar_path.with_suffix(".tmp")

    with zipfile.ZipFile(jar_path, "r") as src, \
         zipfile.ZipFile(temp_jar, "w", compression=zipfile.ZIP_DEFLATED) as dst:

        for item in src.infolist():
            if item.filename in replacements:
                continue
            data = src.read(item.filename)
            dst.writestr(item, data)

        print("\nUpdated entries:")
        for arcname, path in replacements.items():
            dst.write(path, arcname)
            print(f"  {arcname}")

    temp_jar.replace(jar_path)


def main() -> int:
    if not Path(BASE_JAR).exists():
        print(f"Missing {BASE_JAR}", file=sys.stderr)
        return 1

    java_files = sorted(Path(".").glob("*.java"))
    if not java_files:
        print("No .java files found", file=sys.stderr)
        return 1

    java_infos = [read_source_info(j) for j in java_files]

    print(f"Base JAR: {BASE_JAR}")
    print(f"Destination: {DESTINATION_JAR}")

    with tempfile.TemporaryDirectory() as tmp:
        build_dir = Path(tmp)        
        artifact_map = compile_and_collect(java_infos, build_dir)
        print_artifacts(java_infos, artifact_map, build_dir)

        dest = Path(DESTINATION_JAR)

        if dest.is_dir():
            copy_to_directory(java_infos, artifact_map)
        elif dest.is_file():
            copy_into_jar(java_infos, artifact_map)
        else:
            print(f"Destination '{dest}' does not exist", file=sys.stderr)
            return 1

    print("\nDone.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
