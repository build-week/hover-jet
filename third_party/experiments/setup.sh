sudo apt-get install git -y
sudo apt-get install cmake -y

#
# Gtest
#
dpkg -l | grep libgtest-dev
if [ $? -eq 1 ]; then
    sudo apt-get install libgtest-dev -y
    cd /usr/src/gtest
    sudo cmake CMakeLists.txt
    sudo make
    sudo cp *.a /usr/lib
fi


#
# Libs
#

sudo apt-get install libopencv-dev python-opencv -y
sudo apt-get install libxmu-dev libxi-dev -y
sudo apt-get install libassimp-dev -y

#
# Tools
#

sudo apt-get install python-pip ipython -y
sudo pip install matplotlib -y
sudo pip install colorama -y
sudo pip install generate-cmake -y
sudo apt-get install python-tk -y

#
# Viewer
#

sudo apt-get install xorg-dev libglu1-mesa-dev -y
sudo apt-get install libglfw3 libglfw3-dev libglew-dev -y
sudo apt-get install freeglut3-dev -y
