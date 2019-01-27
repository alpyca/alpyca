import sys
import os
import shutil
import argparse
import pkgutil
import inspect


class PybindWriter():
    def __init__(self):
        self.text = ''
        self.repeated_types = set()

    def write(self, text):
        self.text += text

    def get_text(self):
        return self.text

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
        writer.write_line('py::class_<{clsname}>(m, \"{normalized_clsname}\")'.format(clsname=clsname, normalized_clsname=normalized_clsname)) 
        print(clsname)

        with PybindClass(writer):
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
                    
                    writer.write_line('.def_property_readonly(\"{attr}\", []({clsname} *obj) -> Repeated<{attr_type}>'.format(attr=field.name, clsname=clsname, attr_type=attr_type))
                    with Block(writer, braces=('{', '})')):
                        writer.write_line('std::function<{attr_type}(int)> func = [obj](int index) {{ return obj->{attr}(index); }};'.format(attr=field.name, attr_type=attr_type))
                        writer.write_line('return Repeated<{attr_type}>(func, obj->{attr}_size());'.format(attr=field.name, attr_type=attr_type))
                else:
                    print(hasattr(getattr(clsmember(), field.name), 'DESCRIPTOR'))
                    print(field.name + ' ' + str(type(getattr(clsmember(), field.name))))
                    writer.write_line('.def_property_readonly(\"{attr}\", &{clsname}::{attr})'.format(attr=field.name, clsname=clsname))

    def add_repeated(self, attr_type):
        normalized_attr_type = attr_type.replace('::', '_')
        writer.write_line('py::class_<Repeated<{attr_type}>>(m, \"Repeated_{normalized_attr_type}\")'.format(attr_type=attr_type, normalized_attr_type=normalized_attr_type)) 
        writer.write_line(".def(\"__getitem__\", &Repeated<{}>::getitem);".format(attr_type))
        writer.new_line()


class PybindFile():
    def __init__(self, path):
        self.path = path

    def __enter__(self):
        self.writer = PybindWriter()
        return self.writer

    def __exit__(self, *args):
        pybind_file = open(self.path, 'w')
        pybind_file.write(self.writer.get_text())
        pybind_file.close()


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


class Block():
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


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('in_dir')
    parser.add_argument('msgs_path')
    args = parser.parse_args()
    return args

if __name__ == '__main__':
    args = get_args()
    
    out_dir = os.path.split(args.msgs_path)[0]
    if os.path.exists(out_dir):
        shutil.rmtree(out_dir)
    os.mkdir(out_dir)
    open(out_dir + '/__init__.py', 'w').close()

    cmd = 'protoc -I={in_dir} --python_out={out_dir} {in_dir}/*.proto'.format(in_dir=args.in_dir, out_dir=out_dir)
    print(cmd)
    os.system(cmd)
    sys.path.append(out_dir)

    package_name = os.path.split(out_dir)[1]

    with PybindFile(args.msgs_path) as writer:
        writer.includes()
        writer.write_line('namespace py = pybind11;')
        writer.new_line()

        writer.write_line('namespace gazebo')
        with Block(writer, braces=('{', '}')):

            writer.write_line('namespace msgs')
            with Block(writer, braces=('{', '}')):

                writer.write_line('PYBIND11_MODULE(msgs, m)')
                with Block(writer, braces=('{', '}')):

                    prefix = package_name + "."
                    for importer, modname, ispkg in pkgutil.iter_modules([out_dir], prefix):
                        print("Found submodule %s (is a package: %s)" % (modname, ispkg))
                        def create_module(modname):
                            import_name = modname.replace(prefix, '')
                            module = __import__(import_name, fromlist="dummy")
                            clsmembers = inspect.getmembers(module, inspect.isclass)
                            for (clsname, clsmember) in clsmembers:
                                for attr_name in dir(clsmember):
                                    attr = getattr(clsmember, attr_name)
                                    print(type(attr))
                                    print(type(attr) == type(clsmember))
                                    if type(attr) == type(clsmember):
                                        #Diagnostics.DiagTime
                                        concat_name = '::'.join([clsname, attr_name])
                                        writer.add_class(concat_name, attr)
                                writer.add_class(clsname, clsmember)
                                
                        create_module(modname)
        
                    for repeated_type in writer.repeated_types:
                        writer.add_repeated(repeated_type)       
