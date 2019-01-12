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
# Process management
#
RUN apt install supervisor -y
RUN mkdir -p /var/log/supervisor


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
# Serial library
#

RUN apt install catkin -y; git clone https://github.com/wjwwood/serial.git; cd serial; make; make install;
RUN cp -R /tmp/usr/ .
RUN rm -r serial
RUN cd /usr/local/lib/; mv libserial.so libjet_serial.so


#
# Realsense support
#

RUN apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
RUN add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
RUN apt update
RUN apt install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg -y


#
# X tests (xeyes, glxgears, etc.)
#

RUN apt install mesa-utils x11-apps -y


#
# Webcam tests (ex. fswebcam --save test.jpeg; eog test.jpeg)
#

RUN apt install fswebcam eog -y
