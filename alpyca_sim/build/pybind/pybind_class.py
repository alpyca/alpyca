
class PybindClass():
    def __init__(self, writer):
        self.writer = writer

    def __enter__(self):
        pass

    def __exit__(self, *args):
        self.writer.remove_last_char()
        self.writer.write(';')
        self.writer.new_line()
        self.writer.new_line()
