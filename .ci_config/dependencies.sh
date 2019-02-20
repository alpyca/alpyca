apt -qq install --no-install-recommends --allow-unauthenticated -y \
  build-essential python-catkin-tools python-pip protobuf-compiler
pip install protobuf

cur_dir=$PWD
cd ~/
git clone https://github.com/pybind/pybind11.git
cd pybind11
cmake
make install
cd $cur_dir
