# Create Workspace
WORKSPACE=~/git

mkdir $WORKSPACE

# CLONE PROJECT
cd $WORKSPACE && git clone https://github.com/iamarkaj/Mapping.git

# INSTALL ORB_SLAM2
# Prerequisite
sudo apt-get install cmake build-essential libgtk2.0-de libjpeg-dev libtiff5-dev libjasper-dev libopenexr-dev libtbb-dev libglew-dev python-numpy python-matplotlib libboost-all-dev libflann1.8 libflann-dev libqglviewer-dev-qt4

source ~/.bashrc

# Eigen3
cd $WORKSPACE && git clone https://gitlab.com/libeigen/eigen.git
git checkout 3.2
cd eigen
mkdir build && cd build
cmake ..
sudo make install

source ~/.bashrc

# Pangolin
cd $WORKSPACE && git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
git checkout v0.6
cd Pangolin
mkdir build && cd build
cmake ..
cmake --build .

source ~/.bashrc

# g2o
cd $WORKSPACE/Mapping/ORB_SLAM2/Thirdparty/g2o
mkdir build && cd build
cmake ..
make

source ~/.bashrc

# DBoW2
cd $WORKSPACE/Mapping/ORB_SLAM2/Thirdparty/DBoW2
mkdir build && cd build
cmake ..
make

source ~/.bashrc

# VTK
cd $WORKSPACE && git clone https://github.com/Kitware/VTK.git
git checkout v8.1.1
cd VTK
mkdir build && cd build
cmake ..
make
sudo make install

source ~/.bashrc

# PCL
cd $WORKSPACE && git clone https://github.com/PointCloudLibrary/pcl.git
git checkout pcl-1.8.1
cd pcl
mkdir build && cd build
cmake ..
make
sudo make install

source ~/.bashrc

# OctoMap
cd $WORKSPACE && git clone https://github.com/OctoMap/octomap.git
cd octomap
mkdir build && cd build
cmake ..
make
sudo make install

source ~/.bashrc

# ORB_SLAM2
cd $WORKSPACE/Mapping/ORB_SLAM2
mkdir build && cd build
cmake ..
make

source ~/.bashrc

# Make virtual environment
cd $WORKSPACE/Mapping/DenseDepth
python3 -m venv venv
venv/bin/pip install --upgrade pip
venv/bin/pip3 install -r requirements.txt

cd $WORKSPACE/Mapping

echo "Done"

