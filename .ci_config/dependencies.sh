apt -qq install --no-install-recommends --allow-unauthenticated -y \
  build-essential python-catkin-tools python-pip protobuf-compiler
pip install protobuf

git clone https://github.com/pybind/pybind11.git
cd pybind11
cmake
make install
cd ..
