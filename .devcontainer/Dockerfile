FROM mcr.microsoft.com/devcontainers/cpp:1-debian-11

RUN apt-get update \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get install -y --no-install-recommends \
        clang-format-13 \
        vim

RUN update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-13 100