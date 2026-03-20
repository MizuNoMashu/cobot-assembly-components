FROM mcr.microsoft.com/devcontainers/base:ubuntu-22.04

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    PYTHONPATH=/app/src

WORKDIR /app

# System dependencies required to build libfranka/pylibfranka and run the project.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3 \
    python3-dev \
    python3-pip \
    python3-venv \
    python3-numpy \
    python3-pybind11 \
    python3-pytest \
    pkg-config \
    libeigen3-dev \
    libpoco-dev \
    libboost-all-dev \
    liburdfdom-dev \
    libassimp-dev \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Network in this environment uses custom TLS inspection, so git SSL verification is disabled in-container.
RUN git config --global http.sslVerify false

# Install pinocchio from source as fallback when distro packages are unavailable.
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

# Build and install libfranka + pylibfranka from source (official stable tag).
RUN git clone --recursive https://github.com/frankarobotics/libfranka.git /tmp/libfranka \
    && cd /tmp/libfranka \
    && git checkout 0.16.0 \
    && git submodule update --init --recursive \
    && python3 -m pip install --no-build-isolation /tmp/libfranka \
    && ldconfig \
    && rm -rf /tmp/libfranka

COPY . /app

# Install project package using already available system Python dependencies.
RUN python3 -m pip install -e . --no-deps --no-build-isolation

COPY docker/entrypoint.sh /usr/local/bin/robot-entrypoint
RUN chmod +x /usr/local/bin/robot-entrypoint

ENTRYPOINT ["/usr/local/bin/robot-entrypoint"]
CMD ["help"]
