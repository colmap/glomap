FROM colmap/colmap:latest AS builder

# Build GLOMAP on top of the COLMAP image.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        git \
        cmake \
        ninja-build \
        build-essential \
        libboost-program-options-dev \
        libboost-graph-dev \
        libboost-system-dev \
        libeigen3-dev \
        libfreeimage-dev \
        libmetis-dev \
        libgoogle-glog-dev \
        libgtest-dev \
        libgmock-dev \
        libsqlite3-dev \
        libglew-dev \
        qt6-base-dev \
        libqt6opengl6-dev \
        libqt6openglwidgets6 \
        libcgal-dev \
        libceres-dev \
        libcurl4-openssl-dev \
        libssl-dev \
        libmkl-full-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/glomap
COPY . /opt/glomap

RUN mkdir -p build /opt/glomap/install /opt/glomap/runtime-libs \
    && cd build \
    && cmake .. -GNinja -DCMAKE_INSTALL_PREFIX=/opt/glomap/install \
    && ninja \
    && ninja install \
    && ldd /opt/glomap/install/bin/glomap \
        | awk '{if ($3 ~ /^\\//) print $3; else if ($1 ~ /^\\//) print $1}' \
        | xargs -r -I{} cp --parents {} /opt/glomap/runtime-libs \
    && cd /opt/glomap \
    && rm -rf build

FROM colmap/colmap:latest

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        libmkl-sequential \
        libmkl-core \
        libmkl-def \
        libmkl-locale \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /opt/glomap/install/ /usr/local/
COPY --from=builder /opt/glomap/runtime-libs/ /

RUN ldconfig

CMD ["glomap", "-h"]
