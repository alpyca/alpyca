import sys
import os
import shutil
import argparse
import pkgutil
import inspect

from pybind_file import PybindFile
from pybind_block import PybindBlock
from pybind_class_writer import PybindClassWriter


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('in_dir')
    parser.add_argument('msgs_path')
    args = parser.parse_args()
    return args


def proto_to_pybind11(args):
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
        with PybindBlock(writer, braces=('{', '}')):

            writer.write_line('namespace msgs')
            with PybindBlock(writer, braces=('{', '}')):

                writer.write_line('PYBIND11_MODULE(msgs, m)')
                with PybindBlock(writer, braces=('{', '}')):

                    prefix = package_name + "."
                    for _, modname, ispkg in pkgutil.iter_modules([out_dir], prefix):
                        print("Found submodule %s (is a package: %s)" % (modname, ispkg))
                        def create_module(modname):
                            import_name = modname.replace(prefix, '')
                            module = __import__(import_name, fromlist="dummy")
                            clsmembers = inspect.getmembers(module, inspect.isclass)
                            for (clsname, clsmember) in clsmembers:
                                for attr_name in dir(clsmember):
                                    attr = getattr(clsmember, attr_name)
                                    if type(attr) == type(clsmember):
                                        concat_name = '::'.join([clsname, attr_name])
                                        class_writer = PybindClassWriter(writer, concat_name, attr)
                                        class_writer.write()

                                class_writer = PybindClassWriter(writer, clsname, clsmember)
                                class_writer.write()
                                
                        create_module(modname)
        
                    for repeated_type in writer.repeated_types:
                        writer.add_repeated(repeated_type)       


if __name__ == '__main__':
    args = get_args()
    proto_to_pybind11(args)
    