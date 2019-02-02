from pybind_class import PybindClass
from pybind_block import PybindBlock


class PybindWriter():
    def __init__(self):
        self.text = ''
        self.repeated_types = set()

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

    def add_class(self, clsname, clsmember):
        normalized_clsname = clsname.replace('::', '_')
        self.write_line('py::class_<{clsname}>(m, \"{normalized_clsname}\")'.format(clsname=clsname, normalized_clsname=normalized_clsname)) 

        with PybindClass(self):
            for field in clsmember.DESCRIPTOR.fields:
                if field.label == field.LABEL_REPEATED:
                    if field.message_type is not None:
                        attr_type = field.message_type.name
                    else:
                        dtype_mapper = {field.CPPTYPE_BOOL: 'bool',
                                        field.CPPTYPE_DOUBLE: 'double',
                                        field.CPPTYPE_FLOAT: 'float',
                                        field.CPPTYPE_INT32: 'int32_t',
                                        field.CPPTYPE_INT64: 'int64_t',
                                        field.CPPTYPE_STRING: 'std::string',
                                        field.CPPTYPE_UINT32: 'uint32_t',
                                        field.CPPTYPE_UINT64: 'int64_t'}
                        attr_type = dtype_mapper[field.cpp_type]
                    if attr_type in dir(clsmember):
                        attr_type = '::'.join([clsname, attr_type])
                    self.repeated_types.add(attr_type)
                    
                    self.write_line('.def_property_readonly(\"{attr}\", []({clsname} *obj) -> Repeated<{attr_type}>'.format(attr=field.name, clsname=clsname, attr_type=attr_type))
                    with PybindBlock(self, braces=('{', '})')):
                        self.write_line('std::function<{attr_type}(int)> func = [obj](int index) {{ return obj->{attr}(index); }};'.format(attr=field.name, attr_type=attr_type))
                        self.write_line('return Repeated<{attr_type}>(func, obj->{attr}_size());'.format(attr=field.name, attr_type=attr_type))
                else:
                    self.write_line('.def_property_readonly(\"{attr}\", &{clsname}::{attr})'.format(attr=field.name, clsname=clsname))

    def add_repeated(self, attr_type):
        normalized_attr_type = attr_type.replace('::', '_')
        self.write_line('py::class_<Repeated<{attr_type}>>(m, \"Repeated_{normalized_attr_type}\")'.format(attr_type=attr_type, normalized_attr_type=normalized_attr_type)) 
        self.write_line('.def(\"__getitem__\", &Repeated<{}>::getitem);'.format(attr_type))
        self.new_line()
