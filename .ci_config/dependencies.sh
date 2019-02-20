apt -qq install --no-install-recommends --allow-unauthenticated -y \
  build-essential python-catkin-tools python-pip protobuf-compiler
  
pip install --upgrade --user travis pip setuptools wheel virtualenv

pip install --user protobuf
pip install --user pytest

cur_dir=$PWD
cd ~/
git clone https://github.com/pybind/pybind11.git
cd pybind11
cmake . -DPYTHON_EXECUTABLE=/usr/bin/python
make install
cd $cur_dir
