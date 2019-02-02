import unittest

from pybind_class import PybindClass
from pybind_writer import PybindWriter


class TestPybindClass(unittest.TestCase):

    def test_context_manager(self):
        writer = PybindWriter()
        with PybindClass(writer):
            writer.write_line('test')
        
        self.assertEqual(writer.text, 'test;\n\n')


if __name__ == '__main__':
    unittest.main()