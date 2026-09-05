#!/bin/bash
# Cross-compile the RP2040 firmware inside the ROS image (or any Ubuntu 22.04
# container). Used when the host cannot install gcc-arm-embedded (the Homebrew
# cask needs sudo). Writes firmware/build/tactile_umi_fw.uf2 on the bind mount.
set -eo pipefail
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
# libnewlib provides nosys.specs / crt0; gcc-arm-none-eabi alone is not enough
# to link Pico SDK boot stage 2.
apt-get install -y --no-install-recommends \
    gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib \
    git python3 >/dev/null
cd /ws/firmware
cmake -B build
cmake --build build -j"$(nproc)"
test -f build/tactile_umi_fw.uf2
ls -la build/tactile_umi_fw.uf2
