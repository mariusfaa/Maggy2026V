ASFLAGS_ADDITIONAL = -ffunction-sections -fdata-sections -nostdlib -MMD -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -D__IMXRT1062__ -DTEENSYDUINO=158 -DARDUINO=10819 -DARDUINO_TEENSY41 -DF_CPU=$(TEENSY_CPU_FREQ) -DUSB_SERIAL -DLAYOUT_US_ENGLISH
TARGET_LOAD_CMD = 
TARGET_LOAD_CMD_ARGS = 
TARGET_PKG_INSTALLDIR = /home/miku/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinotarget/registry/..
STACK_SIZE = 64
