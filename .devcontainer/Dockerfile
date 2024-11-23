FROM mcr.microsoft.com/devcontainers/cpp:1-debian-12

RUN apt-get update \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get install -y --no-install-recommends \
        clang-format \
        vim