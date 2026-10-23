import importlib
import unittest


class TestHybridControllerPackage(unittest.TestCase):
    def test_controller_package_imports(self) -> None:
        controllers = importlib.import_module("hybrid_controller.controllers")
        self.assertIsNotNone(controllers)


if __name__ == "__main__":
    unittest.main()
