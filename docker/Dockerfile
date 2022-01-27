FROM dlopezmadrid/pcl-docker:latest

RUN git clone https://github.com/carla-simulator/carla

USER root
RUN apt-get install -y clang-8
RUN ln -s /usr/bin/clang++-8 /usr/bin/clang++
RUN touch /etc/apt/apt.conf.d/99verify-peer.conf \
    && echo >>/etc/apt/apt.conf.d/99verify-peer.conf "Acquire { https::Verify-Peer false }"
RUN apt update && apt install -y \
    ninja-build libtiff-dev libjpeg-dev

RUN cd carla/Examples/CppClient && make run

# Copy project files
COPY /src ./src
COPY CMakeLists.txt map.pcd ./

# Compile project
WORKDIR /home/pcl/build
RUN cmake .. && make

USER pcl
CMD ["./cloud_loc"]
