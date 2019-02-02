import unittest

from pybind_block import PybindBlock
from pybind_writer import PybindWriter


class TestPybindBlock(unittest.TestCase):

    def test_context_manager(self):
        writer = PybindWriter()
        with PybindBlock(writer):
            writer.write_line('test')
        
        self.assertEqual(writer.text, '  test\n')

    def test_context_manager_with_braces(self):
        writer = PybindWriter()
        with PybindBlock(writer, braces=('{', '}')):
            writer.write_line('test')
        
        self.assertEqual(writer.text, '{\n  test\n}\n')


if __name__ == '__main__':
    unittest.main()