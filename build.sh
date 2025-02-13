#!/bin/bash -e

### 编译内核 设置环境变量
source /opt/st/myd-yf13x-emmc/4.0.4-snapshot/environment-setup-cortexa7t2hf-neon-vfpv4-ostl-linux-gnueabi

mkdir -p ../build

### source "drivers/input/touchscreen/gt9xx/Kconfig"

make ARCH=arm O="$PWD/../build" myir_stm32mp135x_defconfig

make ARCH=arm uImage vmlinux dtbs LOADADDR=0xC2000040 O="$PWD/../build"

make ARCH=arm modules O="$PWD/../build"

### 生成输出文件

make ARCH=arm INSTALL_MOD_PATH="$PWD/../build/install_artifact" modules_install O="$PWD/../build"

mkdir -p $PWD/../build/install_artifact/boot/

cp $PWD/../build/arch/arm/boot/uImage $PWD/../build/install_artifact/boot/

cp $PWD/../build/arch/arm/boot/dts/my*.dtb $PWD/../build/install_artifact/boot/

exit 0