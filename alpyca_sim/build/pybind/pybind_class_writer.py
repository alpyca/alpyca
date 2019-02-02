from google.protobuf.pyext._message import FieldDescriptor

from pybind_class import PybindClass
from pybind_block import PybindBlock


class PybindClassWriter():
    def __init__(self, writer, clsname, clsmember):
        self.writer = writer
        self.clsname = clsname
        self.clsmember = clsmember

        self.dtype_mapper = {FieldDescriptor.CPPTYPE_BOOL: 'bool',
                             FieldDescriptor.CPPTYPE_DOUBLE: 'double',
                             FieldDescriptor.CPPTYPE_FLOAT: 'float',
                             FieldDescriptor.CPPTYPE_INT32: 'int32_t',
                             FieldDescriptor.CPPTYPE_INT64: 'int64_t',
                             FieldDescriptor.CPPTYPE_STRING: 'std::string',
                             FieldDescriptor.CPPTYPE_UINT32: 'uint32_t',
                             FieldDescriptor.CPPTYPE_UINT64: 'int64_t'}

    def extract_attr_type(self, field):
        if field.message_type is not None:
            attr_type = field.message_type.name
        else:
            attr_type = self.dtype_mapper[field.cpp_type]
        if attr_type in dir(self.clsmember):
                attr_type = '::'.join([self.clsname, attr_type])
        return attr_type

    def add_field(self, field):
        if field.label == FieldDescriptor.LABEL_REPEATED:
            attr_type = self.extract_attr_type(field)

            self.writer.add_repeated_types(attr_type)
                    
            self.writer.write_line('.def_property_readonly(\"{attr}\", []({clsname} *obj) -> Repeated<{attr_type}>'.format(attr=field.name, clsname=self.clsname, attr_type=attr_type))
            with PybindBlock(self.writer, braces=('{', '})')):
                self.writer.write_line('std::function<{attr_type}(int)> func = [obj](int index) {{ return obj->{attr}(index); }};'.format(attr=field.name, attr_type=attr_type))
                self.writer.write_line('return Repeated<{attr_type}>(func, obj->{attr}_size());'.format(attr=field.name, attr_type=attr_type))
        else:
            self.writer.write_line('.def_property_readonly(\"{attr}\", &{clsname}::{attr})'.format(attr=field.name, clsname=self.clsname))

    def write(self):
        normalized_clsname = self.clsname.replace('::', '_')
        self.writer.write_line('py::class_<{clsname}>(m, \"{normalized_clsname}\")'.format(clsname=self.clsname, normalized_clsname=normalized_clsname)) 

        with PybindClass(self.writer):
            for field in self.clsmember.DESCRIPTOR.fields:
                self.add_field(field)
