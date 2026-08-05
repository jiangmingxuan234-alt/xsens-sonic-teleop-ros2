import tempfile
import unittest
from pathlib import Path

import python_runtime


class ResolvePythonTest(unittest.TestCase):
    def test_preserves_virtual_environment_symlink(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            system_python = root / "system-python"
            system_python.write_bytes(b"#!/bin/sh\nexit 0\n")
            system_python.chmod(0o755)
            venv_python = root / "venv-python"
            venv_python.symlink_to(system_python)

            resolved = python_runtime._resolve_python(str(venv_python))

            self.assertEqual(resolved, venv_python)


if __name__ == "__main__":
    unittest.main()
