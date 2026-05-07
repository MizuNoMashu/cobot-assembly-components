FROM mcr.microsoft.com/devcontainers/base:ubuntu-22.04

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    PYTHONPATH=/app/src

WORKDIR /app

# Dipendenze di sistema per buildare libfranka/pylibfranka
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
    pkg-config \
    libeigen3-dev \
    libpoco-dev \
    libboost-all-dev \
    liburdfdom-dev \
    libassimp-dev \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Ambiente di laboratorio con TLS inspection: disabilitiamo SSL verification per git
RUN git config --global http.sslVerify false

# pinocchio è richiesto da pylibfranka 0.16.0 come dipendenza CMake.
# Non è disponibile come pacchetto distro su arm64/22.04, quindi si builda da sorgente.
RUN git clone https://github.com/stack-of-tasks/pinocchio.git /tmp/pinocchio \
    && cmake -S /tmp/pinocchio -B /tmp/pinocchio/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_BENCHMARK=OFF \
        -DBUILD_PYTHON_INTERFACE=OFF \
        -DBUILD_WITH_COLLISION_SUPPORT=OFF \
    && cmake --build /tmp/pinocchio/build -j2 \
    && cmake --install /tmp/pinocchio/build \
    && rm -rf /tmp/pinocchio

# Build libfranka C++ + pylibfranka Python binding.
# Due step separati: prima si installa la shared library C++ nel sistema,
# poi si builda il Python binding che la linka dinamicamente.
RUN git clone --recursive https://github.com/frankarobotics/libfranka.git /tmp/libfranka \
    && cd /tmp/libfranka \
    && git checkout 0.16.0 \
    && git submodule update --init --recursive \
    # Step 1: build e installa libfranka.so nel sistema (/usr/local/lib)
    && cmake -S /tmp/libfranka -B /tmp/libfranka/build_cpp \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_DOCUMENTATION=OFF \
        -DGENERATE_PYLIBFRANKA=OFF \
    && cmake --build /tmp/libfranka/build_cpp -j2 \
    && cmake --install /tmp/libfranka/build_cpp \
    # Step 2: aggiorna il linker cache così libfranka.so è trovabile
    && ldconfig \
    # Step 3: builda il Python binding (ora libfranka.so è disponibile)
    && python3 -m pip install --no-build-isolation /tmp/libfranka \
    && rm -rf /tmp/libfranka

# Installa dipendenze Python (Flask, SocketIO, eventlet)
COPY requirements.txt .
RUN python3 -m pip install -r requirements.txt

# Copia sorgenti applicazione
COPY src/ ./src/
COPY app/ ./app/

EXPOSE 5000

CMD ["python3", "-m", "app.server"]
