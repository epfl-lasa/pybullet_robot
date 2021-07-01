FROM osrf/ros:noetic-desktop AS project-sources

ENV DEBIAN_FRONTEND=noninteractive
ENV QT_X11_NO_MITSHM 1

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

RUN apt-get update && apt-get install -y \
    cmake \
    libgl1-mesa-glx \
    sudo git nano \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# install pybullet
RUN git clone https://github.com/bulletphysics/bullet3
RUN cd bullet3 && ./build_cmake_pybullet_double.sh
ENV PYTHONPATH="/bullet3/build_cmake/examples/pybullet":"${PYTHONPATH}"

RUN apt-get update && apt-get install -y \
    python3-pip \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /project_sources
COPY ./config/requirements.txt ./requirements.txt
RUN pip3 install -r requirements.txt
COPY ./src/simulation ./simulation
COPY ./src/robots ./robots
RUN pip3 install ./simulation ./robots

FROM project-sources AS zmq-user

COPY ./src/interfaces ./interfaces
RUN pip3 install ./interfaces

ARG UID=1000
ARG GID=1000
RUN addgroup --gid ${GID} zmq
RUN adduser --gecos "ZMQ User" --disabled-password --uid ${UID} --gid ${GID} zmq

USER zmq
RUN echo "alias python='python3'" >> /home/zmq/.bashrc

WORKDIR /home/zmq
COPY ./models ./models
COPY ./tests ./tests

ENTRYPOINT ["bash"]
