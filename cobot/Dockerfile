FROM mcr.microsoft.com/devcontainers/base:ubuntu-22.04

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    PYTHONPATH=/app/src

WORKDIR /app

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3 \
    python3-dev \
    python3-pip \
    python3-venv \
    python3-setuptools \
    python3-wheel \
    python3-numpy \
    python3-pybind11 \
    pkg-config \
    libeigen3-dev \
    libpoco-dev \
    liburdfdom-dev \
    liburdfdom-headers-dev \
    libboost-all-dev \
    libboost-filesystem-dev \
    libboost-serialization-dev \
    libboost-system-dev \
    libboost-program-options-dev \
    libtinyxml2-dev \
    libassimp-dev \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN git config --global http.sslVerify false

RUN python3 -m pip install --upgrade pip setuptools wheel numpy pybind11

# Build pinocchio from source for pylibfranka.
RUN git clone https://github.com/stack-of-tasks/pinocchio.git /tmp/pinocchio \
    && cmake -S /tmp/pinocchio -B /tmp/pinocchio/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_BENCHMARK=OFF \
        -DBUILD_PYTHON_INTERFACE=OFF \
        -DBUILD_WITH_COLLISION_SUPPORT=OFF \
    && cmake --build /tmp/pinocchio/build -j"$(nproc)" \
    && cmake --install /tmp/pinocchio/build \
    && rm -rf /tmp/pinocchio

ARG PYLIBFRANKA_VERSION=0.21.1
RUN git clone --recursive https://github.com/frankarobotics/libfranka.git /tmp/libfranka \
    && cd /tmp/libfranka \
    && git checkout ${PYLIBFRANKA_VERSION} \
    && git submodule update --init --recursive \
    && cmake -S /tmp/libfranka -B /tmp/libfranka/build_cpp \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_DOCUMENTATION=OFF \
        -DGENERATE_PYLIBFRANKA=OFF \
    && cmake --build /tmp/libfranka/build_cpp -j"$(nproc)" \
    && cmake --install /tmp/libfranka/build_cpp \
    && ldconfig \
    && python3 -m pip install --no-build-isolation /tmp/libfranka \
    && rm -rf /tmp/libfranka

COPY requirements.txt .
RUN python3 -m pip install -r requirements.txt

COPY src/ ./src/
COPY app/ ./app/

EXPOSE 5000

CMD ["python3", "-m", "app.server"]
