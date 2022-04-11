# Based on image with PCL 
FROM dlopezmadrid/pcl-docker:latest

USER root

# Basic utilities
RUN touch /etc/apt/apt.conf.d/99verify-peer.conf \
    && echo >>/etc/apt/apt.conf.d/99verify-peer.conf "Acquire { https::Verify-Peer false }"
RUN apt update && apt install -y \
    clang-8 ninja-build libtiff-dev libjpeg-dev
RUN ln -s /usr/bin/clang++-8 /usr/bin/clang++

# Build CARLA client library
RUN git clone https://github.com/carla-simulator/carla
RUN cd carla/Examples/CppClient && make run

# Insert project files
COPY /include ./include
COPY /src ./src
COPY /data ./data
COPY CMakeLists.txt config.cfg ./

# Build project
WORKDIR /home/pcl
RUN mkdir build && cd build && cmake .. && make

USER pcl
CMD ["./build/cloud_loc"]
