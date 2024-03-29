FROM ubuntu:jammy

#
# Install devtools
#

RUN apt update
RUN apt install software-properties-common -y
RUN add-apt-repository ppa:ubuntu-toolchain-r/test -y
RUN apt install build-essential -y
RUN apt update

RUN apt install git -y
RUN apt install cmake gcc-11 g++-11 -y

RUN apt install libgtest-dev -y
RUN cd /usr/src/gtest; cmake CMakeLists.txt; make; cp lib/*.a /usr/lib
RUN apt install i2c-tools -y
RUN apt install vim nano -y
RUN apt install gdb -y

#
# Libs
#

RUN apt install libxmu-dev libxi-dev -y
RUN apt install libassimp-dev -y
RUN apt install libyaml-cpp-dev -y
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata
RUN apt install libssl-dev -y
RUN apt install libreadline6-dev -y
RUN apt install libserial-dev -y

#
# Opencv
#
RUN apt update --fix-missing
RUN cd /; apt install libgtk2.0-dev libavcodec-dev libavformat-dev libswscale-dev -y
RUN git clone https://github.com/opencv/opencv_contrib.git --depth=1
RUN cd opencv_contrib; CONTRIB_PATH=$PWD; git reset --hard a17185c6dc7aa554591ad1be38923232472f8001; cd ..; git clone https://github.com/opencv/opencv.git --depth=1; cd opencv; git reset --hard e2dbf054ac5c54bd328f648d9d6146c09dfd5484; mkdir build; cd build; cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DOPENCV_EXTRA_MODULES_PATH=$CONTRIB_PATH/modules ..; make -j7; make install
RUN ln -s /usr/local/include/opencv4/opencv2/ /usr/local/include/
RUN rm -rf opencv
RUN rm -rf opencv_contrib
RUN ldconfig -p

#
# Process management
#
RUN apt install supervisor -y
RUN mkdir -p /var/log/supervisor
RUN mkdir /var/log/supervisord/


#
# Python
#

RUN apt install pkg-config python3-pip -y
RUN pip install --upgrade pip
RUN apt install libfreetype6-dev -y
RUN pip install matplotlib
RUN pip install colorama
RUN pip install 2to3
# Convert pymake to Python3 before installing. Long term solution will be to update the third party package
RUN git clone https://github.com/jpanikulam/experiments.git && \
    cd experiments/pymake/generate_cmake && \
    2to3 -w log.py graph.py parse.py file_system.py cmake.py bazel.py && \
    cd .. && \
    pip install .
RUN apt install python-tk -y

#
# Gl Viewer Support
#
RUN apt update
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
RUN git clone https://github.com/eclipse/paho.mqtt.c.git; cd paho.mqtt.c; git checkout v1.3.8; cmake -Bbuild -H. -DPAHO_ENABLE_TESTING=OFF -DPAHO_BUILD_STATIC=ON -DPAHO_WITH_SSL=ON -DPAHO_HIGH_PERFORMANCE=ON; cmake --build build/ --target install; ldconfig
RUN rm -r paho.mqtt.c
RUN git clone https://github.com/eclipse/paho.mqtt.cpp; cd paho.mqtt.cpp; cmake -Bbuild -H. -DPAHO_BUILD_STATIC=ON -DPAHO_BUILD_DOCUMENTATION=TRUE -DPAHO_BUILD_SAMPLES=TRUE; cmake --build build/ --target install; ldconfig
RUN rm -r paho.mqtt.cpp


#
# GUID library
#

RUN apt install uuid-dev -y
RUN git clone https://github.com/graeme-hill/crossguid.git; cd crossguid; git reset --hard 0f2753174c914bc561504b14c963d179283fe829; mkdir build; cd build; cmake ..; make install
RUN rm -r crossguid


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
RUN apt install -y autoconf libtool
RUN git clone https://github.com/martinling/libserialport.git; cd libserialport; ./autogen.sh; ./configure; make; make install;
RUN rm -r libserialport
RUN ldconfig

