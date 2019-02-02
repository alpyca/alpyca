import unittest

from pybind_writer import PybindWriter


class TestPybindWriter(unittest.TestCase):

    def setUp(self):
        self.writer = PybindWriter()

    def test_write(self):
        text = 'text'
        self.writer.write(text)
        self.assertEqual(self.writer.text, text)

    def test_remove_last_char(self):
        self.writer.write('text')
        self.writer.remove_last_char()
        self.assertEqual(self.writer.text, 'tex')

    def test_write_line(self):
        self.writer.write_line('text')
        self.assertEqual(self.writer.text, 'text\n')

    def test_include(self):
        self.writer.include('text')
        self.assertEqual(self.writer.text, '#include text\n')

    def test_new_line(self):
        self.writer.new_line()
        self.assertEqual(self.writer.text, '\n')
        
    def test_repeated(self):
        self.writer.add_repeated('text::123')
        self.assertEqual(self.writer.text, 'py::class_<Repeated<text::123>>(m, \"Repeated_text_123\")\n'\
                                           '.def(\"__getitem__\", &Repeated<text::123>::getitem);\n\n')

if __name__ == '__main__':
    unittest.main()