from pybind_writer import PybindWriter


class PybindFile():
    def __init__(self, path):
        self.path = path

    def __enter__(self):
        self.writer = PybindWriter()
        return self.writer

    def __exit__(self, *args):
        pybind_file = open(self.path, 'w')
        pybind_file.write(self.writer.text)
        pybind_file.close()
