FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    clang-format \
    cppcheck \
    doxygen \
    g++ \
    g++-multilib \
    gcc-arm-none-eabi \
    gcc-multilib \
    gdb-multiarch \
    git \
    libc6-dev \
    libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib \
    make \
    openocd \
    protobuf-compiler \
    python-is-python3 \
    python3-pip \
    python3-serial \
    quilt \
    uncrustify \
    unzip \
    wget

RUN apt-get install -y \
    python3-venv
