#!/usr/bin/env python3
# Copyright (c) 2026 Amal Dev Haridevan
# SPDX-License-Identifier: MIT

"""Generate and optionally build stage-aware C++/Python acados wrappers."""

import argparse
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sysconfig
from typing import Union

from jinja2 import Environment, FileSystemLoader, StrictUndefined
import pybind11


TEMPLATES = (
    "model_ocp.cc.j2",
    "model_ocp.hh.j2",
    "model_ocp_py.cc.j2",
    "model_sim.cc.j2",
    "model_sim.hh.j2",
    "model_sim_py.cc.j2",
)


def _validate_model_name(model_name: str) -> None:
    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", model_name):
        raise ValueError(
            "model_name must be a C identifier (letters, digits, and underscores)"
        )


def _copy_generated_code(source: Path, destination: Path) -> None:
    """Copy the complete export, including model-specific constraint directories."""
    if source.resolve() == destination.resolve():
        return
    destination.mkdir(parents=True, exist_ok=True)
    for item in source.iterdir():
        target = destination / item.name
        if item.is_dir():
            shutil.copytree(item, target, dirs_exist_ok=True)
        else:
            shutil.copy2(item, target)


def generate_code(
    model_name: str,
    c_generated_code_dir: str,
    output_dir: str,
    build_generated: bool = True,
) -> None:
    """Create wrappers around an existing acados code export.

    Set ``build_generated=False`` to render/copy only, which is useful for
    cross-compilation and build-system integration.
    """
    _validate_model_name(model_name)
    source = Path(c_generated_code_dir).expanduser().resolve()
    destination = Path(output_dir).expanduser().resolve()
    if not source.is_dir():
        raise ValueError(f"Generated code directory does not exist: {source}")
    if source != destination:
        common = Path(os.path.commonpath((source, destination)))
        if common == source or common == destination:
            raise ValueError(
                "Input and output directories must not contain one another"
            )

    required = (
        source / f"acados_solver_{model_name}.c",
        source / f"acados_solver_{model_name}.h",
        source / f"acados_sim_solver_{model_name}.c",
        source / f"acados_sim_solver_{model_name}.h",
        source / "Makefile",
    )
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise ValueError("Incomplete acados export; missing: " + ", ".join(missing))

    _copy_generated_code(source, destination)
    environment = Environment(
        loader=FileSystemLoader(Path(__file__).resolve().parent / "templates"),
        undefined=StrictUndefined,
        keep_trailing_newline=True,
    )
    for template_name in TEMPLATES:
        output_name = template_name.removesuffix(".j2")
        rendered = environment.get_template(template_name).render(model_name=model_name)
        (destination / output_name).write_text(rendered, encoding="utf-8")

    print(f"Generated wrappers for '{model_name}' in '{destination}'")
    if build_generated:
        build(model_name, destination)


def _run(command: list[str], cwd: Path) -> None:
    print("+", shlex.join(command))
    subprocess.run(command, cwd=cwd, check=True)


def build(model_name: str, destination: Union[str, Path]) -> None:
    """Build through the generated Makefile, then build the two C++ wrappers."""
    _validate_model_name(model_name)
    destination = Path(destination).resolve()
    if not destination.is_dir():
        raise ValueError(f"Destination does not exist: {destination}")

    acados_root_value = os.environ.get("ACADOS_ROOT")
    if not acados_root_value:
        raise EnvironmentError("ACADOS_ROOT environment variable is not set")
    acados_root = Path(acados_root_value).resolve()
    include_flags = [
        f"-I{destination}",
        f"-I{acados_root / 'include'}",
        f"-I{acados_root / 'include' / 'acados'}",
        f"-I{acados_root / 'include' / 'blasfeo' / 'include'}",
        f"-I{acados_root / 'include' / 'hpipm' / 'include'}",
    ]
    library_flags = [
        f"-L{destination}",
        f"-L{acados_root / 'lib'}",
        "-Wl,-rpath,$ORIGIN",
        f"-Wl,-rpath,{acados_root / 'lib'}",
    ]

    # The generated Makefile knows the exact dynamics, cost, and constraint
    # sources for ERK/IRK/DISCRETE and all supported cost formulations.
    _run(["make", "clean"], destination)
    _run(
        [
            "make",
            "bundled_shared_lib",
            f"INCLUDE_PATH={acados_root / 'include'}",
            f"LIB_PATH={acados_root / 'lib'}",
        ],
        destination,
    )

    generated_library = f"acados_solver_{model_name}"
    common = ["g++", "-std=c++17", "-O3", "-fPIC", "-shared"]
    _run(
        common
        + ["model_ocp.cc", f"-o", f"lib{model_name}_ocp.so"]
        + include_flags
        + library_flags
        + [f"-l{generated_library}", "-lacados", "-lhpipm", "-lblasfeo", "-lm"],
        destination,
    )
    _run(
        common
        + ["model_sim.cc", "-o", f"lib{model_name}_sim.so"]
        + include_flags
        + library_flags
        + [f"-l{generated_library}", "-lacados", "-lhpipm", "-lblasfeo", "-lm"],
        destination,
    )

    extension_suffix = sysconfig.get_config_var("EXT_SUFFIX")
    if not extension_suffix:
        raise RuntimeError("Python did not report an extension-module suffix")
    python_flags = shlex.split(
        subprocess.check_output(
            ["python3-config", "--cflags", "--ldflags"], text=True
        )
    )
    python_include = f"-I{pybind11.get_include()}"
    _run(
        common
        + ["model_ocp_py.cc", "-o", f"{model_name}_ocp_py{extension_suffix}"]
        + include_flags
        + [python_include]
        + python_flags
        + library_flags
        + [f"-l{model_name}_ocp", f"-l{generated_library}"],
        destination,
    )
    _run(
        common
        + ["model_sim_py.cc", "-o", f"{model_name}_sim_py{extension_suffix}"]
        + include_flags
        + [python_include]
        + python_flags
        + library_flags
        + [f"-l{model_name}_sim", f"-l{generated_library}"],
        destination,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate C++ and Python wrappers from acados-generated C code"
    )
    parser.add_argument("--model_name", required=True)
    parser.add_argument("--c_generated_code_dir", required=True)
    parser.add_argument("--output_dir", required=True)
    parser.add_argument(
        "--no-build", action="store_true", help="render wrappers without compiling"
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    generate_code(
        args.model_name,
        args.c_generated_code_dir,
        args.output_dir,
        build_generated=not args.no_build,
    )
