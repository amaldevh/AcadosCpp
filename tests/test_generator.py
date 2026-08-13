import tempfile
import unittest
from pathlib import Path

from generate_cpp_ocp import generate_code


class GeneratorTests(unittest.TestCase):
    def make_export(self, root: Path, model: str = "test_model") -> Path:
        export = root / "export"
        export.mkdir()
        for name in (
            f"acados_solver_{model}.c",
            f"acados_solver_{model}.h",
            f"acados_sim_solver_{model}.c",
            f"acados_sim_solver_{model}.h",
            "Makefile",
        ):
            (export / name).write_text(f"/* {name} */\n", encoding="utf-8")
        constraints = export / f"{model}_constraints"
        constraints.mkdir()
        (constraints / "custom_constraint.c").write_text("/* custom */\n")
        return export

    def test_render_only_copies_complete_export_and_substitutes_name(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = self.make_export(root)
            output = root / "output"

            generate_code("test_model", str(source), str(output), False)

            self.assertTrue(
                (output / "test_model_constraints" / "custom_constraint.c").is_file()
            )
            implementation = (output / "model_ocp.cc").read_text(encoding="utf-8")
            header = (output / "model_ocp.hh").read_text(encoding="utf-8")
            self.assertIn("test_model_acados_solve", implementation)
            self.assertNotIn("quadrotor_acados_solve", implementation)
            self.assertIn("TEST_MODEL_NY0", header)

    def test_rejects_unsafe_model_name(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = self.make_export(root)
            with self.assertRaises(ValueError):
                generate_code("bad-name", str(source), str(root / "output"), False)

    def test_rejects_nested_output(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = self.make_export(root)
            with self.assertRaises(ValueError):
                generate_code(
                    "test_model", str(source), str(source / "nested-output"), False
                )


if __name__ == "__main__":
    unittest.main()
