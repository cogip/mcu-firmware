FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    doxygen \
    g++ \
    g++-multilib \
    gcc-arm-none-eabi \
    gcc-multilib \
    gdb-multiarch \
    git \
    libstdc++-arm-none-eabi-newlib \
    make \
    openocd \
    protobuf-compiler \
    python-is-python3 \
    python3-pip \
    python3-serial \
    quilt \
    unzip \
    wget

ADD requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt
