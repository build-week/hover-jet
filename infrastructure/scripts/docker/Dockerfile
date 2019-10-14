FROM ubuntu:bionic

#
# Install devtools
#
RUN apt update && \
    apt install software-properties-common -y && \
    add-apt-repository ppa:ubuntu-toolchain-r/test -y && \
    apt install build-essential -y && \
    apt update && \
    apt install git -y && \
    apt install cmake gcc-7 g++-7 -y && \
    apt install libgtest-dev -y && \
    cd /usr/src/gtest; cmake CMakeLists.txt; make; cp *.a /usr/lib && \
    apt install i2c-tools -y && \
    apt install vim nano -y && \
    apt install gdb -y

#
# Libs
#
RUN apt install libxmu-dev libxi-dev -y && \
    apt install libassimp-dev -y && \
    apt install libyaml-cpp-dev -y && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata && \
    apt install libssl-dev -y && \
    apt install libreadline6-dev -y && \
    apt install libserial-dev -y

#
# Process management
#
RUN apt install supervisor -y && \
    mkdir -p /var/log/supervisor && \
    mkdir /var/log/supervisord/

#
# Python
#
RUN apt install pkg-config python-pip ipython -y && \
    apt install libfreetype6-dev -y && \
    pip install matplotlib && \
    pip install colorama && \
    pip install generate-cmake==1.6 && \
    apt install python-tk -y

#
# Gl Viewer Support
#
RUN apt update && \
    apt install xorg-dev libglu1-mesa-dev -y && \
    apt install libglfw3 libglfw3-dev libglew-dev -y && \
    apt install freeglut3-dev -y

#
# xvfb for x-fowarding while running headless (on a mac, server, or vehicle node)
#
ADD support/xvfb.init /etc/init.d/xvfb
RUN apt install -y xvfb && \
    chmod +x /etc/init.d/xvfb && \
    update-rc.d xvfb defaults
CMD (service xvfb start; export DISPLAY=:10;)

#
# Mosquitto
#
RUN apt install mosquitto mosquitto-clients -y

#
# paho.mqtt.cpp
#
RUN apt install doxygen graphviz -y && \
    git clone https://github.com/eclipse/paho.mqtt.c.git && \
    cd paho.mqtt.c && \
    git checkout v1.2.1 && \
    cmake -Bbuild -H. -DPAHO_WITH_SSL=ON && \
    cmake --build build/ --target install && \
    ldconfig && \
    cd .. && \
    rm -r paho.mqtt.c && \
    git clone https://github.com/eclipse/paho.mqtt.cpp && \
    cd paho.mqtt.cpp && \
    git checkout f737952c94dd3db62b7ddc75e7d4f8c3ac5cb44e && \
    cmake -Bbuild -H. -DPAHO_BUILD_DOCUMENTATION=TRUE -DPAHO_BUILD_SAMPLES=TRUE && \
    cmake --build build/ --target install && \
    cd .. && \
    rm -r paho.mqtt.cpp

#
# GUID library
#
RUN apt install uuid-dev -y && \
    git clone https://github.com/graeme-hill/crossguid.git && \
    cd crossguid && \
    git reset --hard 0f2753174c914bc561504b14c963d179283fe829 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make install && \
    cd ../.. && \
    rm -r crossguid

#
# X tests (xeyes, glxgears, etc.)
#
RUN apt install mesa-utils x11-apps -y

#
# Webcam tests (ex. fswebcam --save test.jpeg; eog test.jpeg)
#
RUN apt install fswebcam eog -y

#
# Adding serial library
#
RUN apt install -y autoconf libtool && \
    git clone https://github.com/martinling/libserialport.git && \
    cd libserialport && \
    ./autogen.sh && \
    ./configure && \
    make && \
    make install && \
    cd .. && \
    ldconfig && \
    rm -r libserialport

#
# Opencv
#
RUN cd / && \
    apt install libgtk2.0-dev libavcodec-dev libavformat-dev libswscale-dev -y && \
    git clone https://github.com/opencv/opencv_contrib.git && \
    cd opencv_contrib && \
    CONTRIB_PATH=$PWD && \
    git reset --hard a17185c6dc7aa554591ad1be38923232472f8001 && \
    cd .. && \
    git clone https://github.com/opencv/opencv.git && \
    cd opencv && \
    git reset --hard e2dbf054ac5c54bd328f648d9d6146c09dfd5484 && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DOPENCV_EXTRA_MODULES_PATH=$CONTRIB_PATH/modules .. && \
    make -j3 && \
    make install && \
    cd ../.. && \
    ln -s /usr/local/include/opencv4/opencv2/ /usr/local/include/ && \
    rm -rf opencv && \
    rm -rf opencv_contrib && \
    ldconfig -p
