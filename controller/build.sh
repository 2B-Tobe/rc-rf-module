#!/bin/sh
# Command-line build of the RF Module Controller for the STM32F103C8 (arm-none-eabi-gcc, newlib-nano).
# The STM32CubeIDE project (.project / .cproject, see docs/CUBEIDE.md) is the normal way; this script builds
# the same sources, the Arm CMSIS and ST device files in third_party/ and the linker script in the root.
#   sh build.sh            -> build/module_controller.hex / .bin
#   LINK_RX_PIN=10 sh build.sh   (reply pin: 8 = PA8 software receiver (default), 10 = PA10 shared with DEC, 3 = PA3)
set -e
cd "$(dirname "$0")"
OUT=build
mkdir -p $OUT

MCU="-mcpu=cortex-m3 -mthumb"
DEFS="-DSTM32F103xB"
[ -n "$LINK_RX_PIN" ] && DEFS="$DEFS -DLINK_RX_PIN=$LINK_RX_PIN"
INCS="-ICore/Inc -Ithird_party/cmsis_core -Ithird_party/cmsis_device_f1"
CFLAGS="$MCU $DEFS $INCS -Os -g -ffunction-sections -fdata-sections -Wall --specs=nano.specs"

OBJS=""
for f in Core/Src/*.c Core/Src/u8glib/*.c third_party/cmsis_device_f1/system_stm32f1xx.c; do
  o=$OUT/$(basename "$f" .c).o
  arm-none-eabi-gcc $CFLAGS -c "$f" -o "$o"
  OBJS="$OBJS $o"
done
arm-none-eabi-gcc $MCU -c third_party/cmsis_device_f1/startup_stm32f103xb.s -o $OUT/startup.o
OBJS="$OBJS $OUT/startup.o"

arm-none-eabi-gcc $MCU -T STM32F103C8TX_FLASH.ld --specs=nosys.specs --specs=nano.specs \
  -Wl,--gc-sections -Wl,-Map=$OUT/module_controller.map -o $OUT/module_controller.elf $OBJS
arm-none-eabi-objcopy -O ihex   $OUT/module_controller.elf $OUT/module_controller.hex
arm-none-eabi-objcopy -O binary $OUT/module_controller.elf $OUT/module_controller.bin
arm-none-eabi-size $OUT/module_controller.elf
