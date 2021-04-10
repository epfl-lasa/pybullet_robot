FROM python:3.8.9-buster

ENV DEBIAN_FRONTEND=noninteractive
ENV QT_X11_NO_MITSHM 1

RUN apt-get update && apt-get install -y \
    cmake \
    libgl1-mesa-glx \
    sudo \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# install pybullet
RUN git clone https://github.com/bulletphysics/bullet3
RUN cd bullet3 && ./build_cmake_pybullet_double.sh
ENV PYTHONPATH="/bullet3/build_cmake/examples/pybullet":"${PYTHONPATH}"

WORKDIR /pybullet

COPY ./src ./src
COPY ./demos ./demos
COPY ./models ./models
COPY ./tests ./tests
COPY ./install.sh ./install.sh
RUN chmod +x ./install.sh && ./install.sh

ENTRYPOINT ["bash"]
