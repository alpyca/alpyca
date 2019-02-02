from google.protobuf.pyext._message import FieldDescriptor


class PybindWriter():
    def __init__(self):
        self.text = ''
        self.repeated_types = set()

    def add_repeated_types(self, attr_type):
        self.repeated_types.add(attr_type)

    def write(self, text):
        self.text += text

    def remove_last_char(self):
        self.text = self.text[:-1]

    def write_line(self, text):
        self.write(text)
        self.new_line()
    
    def include(self, text):
        self.write_line('#include ' + text)

    def new_line(self):
        self.write('\n')

    def includes(self):
        self.include('<string>')
        self.new_line()

        self.include('<pybind11/pybind11.h>')
        self.include('<pybind11/functional.h>')
        self.include('<pybind11/stl.h>')
        self.include('<gazebo/gazebo.hh>')
        self.new_line()

        self.include('\"alpyca/sim/repeated.h\"')
        self.new_line()

    def add_repeated(self, attr_type):
        normalized_attr_type = attr_type.replace('::', '_')
        self.write_line('py::class_<Repeated<{attr_type}>>(m, \"Repeated_{normalized_attr_type}\")'.format(attr_type=attr_type, normalized_attr_type=normalized_attr_type)) 
        self.write_line('.def(\"__getitem__\", &Repeated<{}>::getitem);'.format(attr_type))
        self.new_line()
