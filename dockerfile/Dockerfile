FROM debian:11 AS builder
LABEL authors="grilo"

RUN dpkg --add-architecture armhf \
    && apt-get -y update \
    && apt-get -y install \
      build-essential \
      pkg-config \
      cmake \
      autoconf \
      automake \
      libtool \
      gcc-arm-linux-gnueabihf \
      g++-arm-linux-gnueabihf \
      gdb \
      libopenblas-dev:armhf \
      libopenblas-dev:armhf \
      liblapack-dev:armhf \
      libarpack2-dev:armhf \
      libsuperlu-dev:armhf \
      libarmadillo-dev:armhf

FROM builder AS builder-lely-core
RUN mkdir /lely-core && touch /lely-core/xablau
COPY lely-core/ /lely-core
RUN mkdir /lely-core/build-arm-docker-debian-11
RUN mkdir /lely-core/install-arm-docker-debian-11
WORKDIR /lely-core/build-arm-docker-debian-11
RUN ../configure --host=arm-linux-gnueabihf --disable-python --disable-tests --prefix=/lely-core/install-arm-docker-debian-11 \
    && make -j20 \
    && make install

FROM builder AS builder-manopla
RUN mkdir -p /lely-core/install-arm-docker-debian-11
ENV PKG_CONFIG_PATH=/lely-core/install-arm-docker-debian-11/lib/pkgconfig
COPY --from=builder-lely-core /lely-core/install-arm-docker-debian-11 /lely-core/install-arm-docker-debian-11
ARG UID=1000
RUN useradd -m -u ${UID} -s /bin/bash builder
USER builder
