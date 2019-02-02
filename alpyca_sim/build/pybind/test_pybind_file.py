import unittest
from mock import patch, mock_open

from pybind_file import PybindFile


class TestPybindFile(unittest.TestCase):

    def test_context_manager(self):
        with patch("__builtin__.open", mock_open()) as mock_file:
            path = 'text.txt'
            text = 'test'
            with PybindFile(path) as writer:
                writer.write(text)
        
            mock_file.assert_called_once_with(path, 'w')
            handle = mock_file()
            handle.write.assert_called_once_with(text)


if __name__ == '__main__':
    unittest.main()