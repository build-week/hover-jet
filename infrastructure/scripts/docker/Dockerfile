FROM ubuntu:bionic

#
# Install devtools
#

RUN apt update
RUN apt install software-properties-common -y
RUN add-apt-repository ppa:ubuntu-toolchain-r/test -y
RUN apt install build-essential -y
RUN apt update

RUN apt install git -y
RUN apt install cmake gcc-7 g++-7 -y

RUN apt install libgtest-dev -y
RUN cd /usr/src/gtest; cmake CMakeLists.txt; make; cp *.a /usr/lib
RUN apt install i2c-tools -y


#
# Libs
#

RUN apt install libxmu-dev libxi-dev -y
RUN apt install libassimp-dev -y
RUN apt install libyaml-cpp-dev -y
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata
RUN apt install libopencv-dev python-opencv -y
RUN apt install libssl-dev -y
RUN apt install libreadline6-dev -y
RUN apt install libserial-dev -y


#
# Python
#

RUN apt install python-pip ipython -y
RUN pip install --upgrade pip
RUN apt install libfreetype6-dev -y
RUN pip install matplotlib
RUN pip install colorama
RUN pip install generate-cmake 
RUN apt install python-tk -y


#
# Gl Viewer Support
#

RUN apt install xorg-dev libglu1-mesa-dev -y
RUN apt install libglfw3 libglfw3-dev libglew-dev -y
RUN apt install freeglut3-dev -y


#
# xvfb for x-fowarding while running headless (on a mac, server, or vehicle node)
#

RUN apt install -y xvfb
ADD support/xvfb.init /etc/init.d/xvfb
RUN chmod +x /etc/init.d/xvfb
RUN update-rc.d xvfb defaults
CMD (service xvfb start; export DISPLAY=:10;)


#
# Mosquitto
#
RUN apt install mosquitto mosquitto-clients -y


#
# paho.mqtt.cpp
#

RUN apt install doxygen graphviz -y
RUN git clone https://github.com/eclipse/paho.mqtt.c.git; cd paho.mqtt.c; git checkout v1.2.1; cmake -Bbuild -H. -DPAHO_WITH_SSL=ON; cmake --build build/ --target install; ldconfig
RUN rm -r paho.mqtt.c
RUN git clone https://github.com/eclipse/paho.mqtt.cpp; cd paho.mqtt.cpp; cmake -Bbuild -H. -DPAHO_BUILD_DOCUMENTATION=TRUE -DPAHO_BUILD_SAMPLES=TRUE; cmake --build build/ --target install
RUN rm -r paho.mqtt.cpp


#
# GUID library
#

RUN apt install uuid-dev -y
RUN git clone https://github.com/graeme-hill/crossguid.git; cd crossguid; mkdir build; cd build; cmake ..; make install
RUN rm -r crossguid


#
# X tests (xeyes, glxgears, etc.)
#

RUN apt install mesa-utils x11-apps -y


#
# Webcam tests (ex. fswebcam --save test.jpeg; eog test.jpeg)
#

RUN apt install fswebcam eog -y
