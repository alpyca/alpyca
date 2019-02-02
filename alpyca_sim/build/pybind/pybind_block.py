
class PybindBlock():
    def __init__(self, writer, braces=None):
        self.writer = writer
        self.braces = braces

    def new_write_line(self, text):
        self.org_write_line('  ' + text)
        
    def __enter__(self):
        if self.braces is not None:
            self.writer.write_line(self.braces[0])

        self.org_write_line = self.writer.write_line
        self.writer.write_line = self.new_write_line

    def __exit__(self, *args):
        self.writer.write_line = self.org_write_line

        if self.braces is not None:
            self.writer.write_line(self.braces[1])
