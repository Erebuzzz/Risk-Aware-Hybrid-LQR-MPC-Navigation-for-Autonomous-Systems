import importlib
import unittest


class TestHybridNavPackage(unittest.TestCase):
    def test_package_imports(self) -> None:
        package = importlib.import_module("hybrid_nav")
        self.assertEqual(package.__name__, "hybrid_nav")


if __name__ == "__main__":
    unittest.main()
