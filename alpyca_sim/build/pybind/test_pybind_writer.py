import unittest

from pybind_writer import PybindWriter


class TestPybindWriter(unittest.TestCase):

    def test_write(self):
        writer = PybindWriter()
        text = 'text'
        writer.write(text)
        self.assertEqual(writer.text, text)

    def test_remove_last_char(self):
        writer = PybindWriter()
        writer.write('text')
        writer.remove_last_char()
        self.assertEqual(writer.text, 'tex')

    def test_write_line(self):
        writer = PybindWriter()
        writer.write_line('text')
        self.assertEqual(writer.text, 'text\n')

    def test_include(self):
        writer = PybindWriter()
        writer.include('text')
        self.assertEqual(writer.text, '#include text\n')

    def test_new_line(self):
        writer = PybindWriter()
        writer.new_line()
        self.assertEqual(writer.text, '\n')
        
    def test_repeated(self):
        writer = PybindWriter()
        writer.add_repeated('text::123')
        self.assertEqual(writer.text, 'py::class_<Repeated<text::123>>(m, \"Repeated_text_123\")\n'\
                                      '.def(\"__getitem__\", &Repeated<text::123>::getitem);\n\n')

if __name__ == '__main__':
    unittest.main()