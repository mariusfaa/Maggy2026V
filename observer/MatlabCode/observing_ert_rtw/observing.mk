###########################################################################
## Makefile generated for component 'observing'. 
## 
## Makefile     : observing.mk
## Generated on : Fri Apr 24 13:39:24 2026
## Final product: $(RELATIVE_PATH_TO_ANCHOR)/observing.elf
## Product type : executable
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile

PRODUCT_NAME              = observing
MAKEFILE                  = observing.mk
MATLAB_ROOT               = /usr/local/MATLAB/R2025b
MATLAB_BIN                = /usr/local/MATLAB/R2025b/bin
MATLAB_ARCH_BIN           = $(MATLAB_BIN)/glnxa64
START_DIR                 = /home/miku/MSc/Maggy2026V/observer/MatlabCode
SOLVER                    = 
SOLVER_OBJ                = 
CLASSIC_INTERFACE         = 0
TGT_FCN_LIB               = ISO_C
MODEL_HAS_DYNAMICALLY_LOADED_SFCNS = 0
RELATIVE_PATH_TO_ANCHOR   = ..
SLIB_PATH                 = /home/miku/Documents/MATLAB/R2025b/ArduinoStaticLibrary/Teensy4.1ArduinoCompatible/FasterRuns
SSLIB_PATH                = /home/miku/Documents/MATLAB/R2025b/ArduinoStaticLibrary/Teensy4.1ArduinoCompatible/FasterRuns
C_STANDARD_OPTS           = 
CPP_STANDARD_OPTS         = 

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          Arduino ARM M7
# Supported Version(s):    
# ToolchainInfo Version:   2025b
# Specification Revision:  1.0
# 
#-------------------------------------------
# Macros assumed to be defined elsewhere
#-------------------------------------------

# ARDUINO_ROOT
# ARDUINO_PACKAGES_TOOLS_ROOT
# ARDUINO_PORT
# ARDUINO_MCU
# ARDUINO_BAUD
# ARDUINO_PROTOCOL
# ARDUINO_F_CPU

#-----------
# MACROS
#-----------

PRODUCT_HEX                 = $(RELATIVE_PATH_TO_ANCHOR)/$(PRODUCT_NAME).hex
PRODUCT_BIN                 = $(RELATIVE_PATH_TO_ANCHOR)/$(PRODUCT_NAME).bin
ARDUINO_M7_TOOLS            = $(ARDUINO_TEENSY_ROOT)/tools/teensy-compile/$(TEENSY_GCC_VERSION)/arm/bin
ARDUINO_TEENSY_TOOLS        = $(ARDUINO_TEENSY_ROOT)/tools/teensy-tools/$(TEENSY_TOOLS_VERSION)
ELF2EEP_OPTIONS             = -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0

TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = -larm_cortexM7lfsp_math -lm -lstdc++ -lcore

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# Assembler: Arduino ARM M7 Assembler
AS_PATH = $(ARDUINO_M7_TOOLS)
AS = "$(AS_PATH)/arm-none-eabi-gcc"

# C Compiler: Arduino ARM M7 C Compiler
CC_PATH = $(ARDUINO_M7_TOOLS)
CC = "$(CC_PATH)/arm-none-eabi-gcc"

# Linker: Arduino ARM M7 Linker
LD_PATH = $(ARDUINO_M7_TOOLS)
LD = "$(LD_PATH)/arm-none-eabi-gcc"

# C++ Compiler: Arduino ARM M7 C++ Compiler
CPP_PATH = $(ARDUINO_M7_TOOLS)
CPP = "$(CPP_PATH)/arm-none-eabi-g++"

# C++ Linker: Arduino ARM M7 C++ Linker
CPP_LD_PATH = $(ARDUINO_M7_TOOLS)
CPP_LD = "$(CPP_LD_PATH)/arm-none-eabi-gcc"

# Archiver: Arduino ARM M7 Archiver
AR_PATH = $(ARDUINO_M7_TOOLS)
AR = "$(AR_PATH)/arm-none-eabi-ar"

# MEX Tool: MEX Tool
MEX_PATH = $(MATLAB_ARCH_BIN)
MEX = "$(MEX_PATH)/mex"

# Binary Converter: Binary Converter
OBJCOPY_PATH = $(ARDUINO_TEENSY_ROOT)/tools/teensy-compile/$(TEENSY_GCC_VERSION)/arm/bin
OBJCOPY = "$(OBJCOPY_PATH)/arm-none-eabi-objcopy"

# Hex Converter: Hex Converter
OBJCOPY_PATH = $(ARDUINO_TEENSY_ROOT)/tools/teensy-compile/$(TEENSY_GCC_VERSION)/arm/bin
OBJCOPY = "$(OBJCOPY_PATH)/arm-none-eabi-objcopy"

# Download: Download
DOWNLOAD =

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder: Make Tool
MAKE_PATH = %MATLAB%/bin/glnxa64
MAKE = "$(MAKE_PATH)/gmake"


#-------------------------
# Directives/Utilities
#-------------------------

ASDEBUG             = -g
AS_OUTPUT_FLAG      = -o
CDEBUG              = -g
C_OUTPUT_FLAG       = -o
LDDEBUG             = -g
OUTPUT_FLAG         = -o
CPPDEBUG            = -g
CPP_OUTPUT_FLAG     = -o
CPPLDDEBUG          = -g
OUTPUT_FLAG         = -o
ARDEBUG             =
STATICLIB_OUTPUT_FLAG =
MEX_DEBUG           = -g
RM                  =
ECHO                = echo
MV                  =
RUN                 =

#----------------------------------------
# "Faster Builds" Build Configuration
#----------------------------------------

MEX_CPPFLAGS         =
MEX_CPPLDFLAGS       =
MEX_CFLAGS           =
MEX_LDFLAGS          =



#---------------------------
# Model-Specific Options
#---------------------------

ASFLAGS = -MMD -MP -MF"$(@:%.o=%.dep)" -MT"$@"  -Wall -x assembler-with-cpp $(ASFLAGS_ADDITIONAL) $(DEFINES) $(INCLUDES) -c

CFLAGS = -O2 -c -w -ffunction-sections -fdata-sections -nostdlib -MMD -MP -MF"$(@:%.o=%.dep)" -MT"$@" -g

LDFLAGS = -O2 -Wl,--gc-sections,--relax -g

SHAREDLIB_LDFLAGS = -g

CPPFLAGS = -std=gnu++14 -fno-exceptions -fpermissive -fno-rtti -fno-threadsafe-statics -felide-constructors -Wno-error=narrowing -O2 -c -w -ffunction-sections -fdata-sections -nostdlib -MMD -MP -MF"$(@:%.o=%.dep)" -MT"$@" -g

CPP_LDFLAGS = -O2 -Wl,--gc-sections,--relax -g

CPP_SHAREDLIB_LDFLAGS = -g

ARFLAGS = rcs

OBJCOPYFLAGS_BIN = $(ELF2EEP_OPTIONS) $(PRODUCT) $(PRODUCT_BIN)

OBJCOPYFLAGS_HEX = -O ihex -R .eeprom $(PRODUCT) $(PRODUCT_HEX)

DOWNLOAD_FLAGS = 

EXECUTE_FLAGS = 

MAKE_FLAGS = -f $(MAKEFILE)

###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = $(RELATIVE_PATH_TO_ANCHOR)/observing.elf
PRODUCT_TYPE = "executable"
BUILD_TYPE = "Top-Level Standalone Executable"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = -I$(START_DIR) -I$(START_DIR)/observing_ert_rtw -I$(MATLAB_ROOT)/extern/include -I$(MATLAB_ROOT)/simulink/include -I$(MATLAB_ROOT)/rtw/c/src -I$(MATLAB_ROOT)/rtw/c/src/ext_mode/common -I$(MATLAB_ROOT)/rtw/c/ert -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/include -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/common -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/include -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/transport/include -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/transport/src -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/platform/include -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/platform/default -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/include -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src -I$(MATLAB_ROOT)/toolbox/coder/rtiostream/src -I/home/miku/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/teensy/hardware/avr/1.58.2/cores/teensy4 -I/home/miku/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/teensy/hardware/avr/1.58.2/libraries/Wire -I/home/miku/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/teensy/hardware/avr/1.58.2/libraries/SPI -I/home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D -I$(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4 -I$(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4/avr -I$(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4/debug -I$(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4/util -I/home/miku/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinotarget/include -I/home/miku/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinotarget/scheduler/include -I/home/miku/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinobase/include -I$(MATLAB_ROOT)/toolbox/target/shared/armcortexmbase/scheduler/include

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_ = -DXCP_ADDRESS_GRANULARITY=XCP_ADDRESS_GRANULARITY_BYTE -DCODERTARGET_XCP_DAQ_PACKED_MODE -DCODERTARGET_XCP_MAX_CONTIGUOUS_SAMPLES=2 -D__MW_TARGET_USE_HARDWARE_RESOURCES_H__ -DMW_TIMERID=60 -DMW_TIMERCOUNT=240000 -DARDUINO_NUM_SERIAL_PORTS=9 -DARDUINO_SERIAL_RECEIVE_BUFFER_SIZE=64 -D_RTT_BAUDRATE_SERIAL0_=9600 -D_RTT_BAUDRATE_SERIAL1_=9600 -D_RTT_BAUDRATE_SERIAL2_=9600 -D_RTT_BAUDRATE_SERIAL3_=9600 -D_RTT_BAUDRATE_SERIAL4_=9600 -D_RTT_BAUDRATE_SERIAL5_=9600 -D_RTT_BAUDRATE_SERIAL6_=9600 -D_RTT_BAUDRATE_SERIAL7_=9600 -D_RTT_BAUDRATE_SERIAL8_=9600 -D_RTT_CONFIG_SERIAL0_=SERIAL_8N1 -D_RTT_CONFIG_SERIAL1_=SERIAL_8N1 -D_RTT_CONFIG_SERIAL2_=SERIAL_8N1 -D_RTT_CONFIG_SERIAL3_=SERIAL_8N1 -D_RTT_CONFIG_SERIAL4_=SERIAL_8N1 -D_RTT_CONFIG_SERIAL5_=SERIAL_8N1 -D_RTT_CONFIG_SERIAL6_=SERIAL_8N1 -D_RTT_CONFIG_SERIAL7_=SERIAL_8N1 -D_RTT_CONFIG_SERIAL8_=SERIAL_8N1 -D_RTT_ANALOG_REF_=0 -DMW_RTIO_SERIAL0 -DMW_NUM_PINS=55 -D_ONBOARD_EEPROM_SIZE_=4284
DEFINES_BUILD_ARGS = -DCLASSIC_INTERFACE=0 -DALLOCATIONFCN=0 -DTERMFCN=1 -DONESTEPFCN=1 -DMAT_FILE=0 -DMULTI_INSTANCE_CODE=0 -DEXT_MODE=1 -DINTEGER_CODE=0 -DMT=0
DEFINES_CUSTOM = 
DEFINES_OPTS = -DXCP_DAQ_SUPPORT -DXCP_CALIBRATION_SUPPORT -DXCP_TIMESTAMP_SUPPORT -DXCP_TIMESTAMP_BASED_ON_SIMULATION_TIME -DXCP_SET_MTA_SUPPORT -DEXTMODE_XCP_TRIGGER_SUPPORT -DXCP_MEM_BLOCK_1_SIZE=32 -DXCP_MEM_BLOCK_1_NUMBER=1 -DXCP_MEM_BLOCK_2_SIZE=56 -DXCP_MEM_BLOCK_2_NUMBER=1 -DXCP_MEM_BLOCK_3_SIZE=40 -DXCP_MEM_BLOCK_3_NUMBER=1 -DXCP_MEM_RESERVED_POOLS_TOTAL_SIZE=1107 -DXCP_MEM_RESERVED_POOLS_NUMBER=2 -DXCP_MEM_DAQ_RESERVED_POOL_BLOCKS_NUMBER=3 -DXCP_MEM_DAQ_RESERVED_POOLS_NUMBER=1 -DXCP_MIN_EVENT_NO_RESERVED_POOL=1 -DXCP_MAX_CTO_SIZE=255 -DXCP_MAX_DTO_SIZE=65532 -DXCP_MAX_ODT_ENTRY_SIZE=255 -DEXTMODE_STATIC -DEXTMODE_STATIC_SIZE=65536 -DON_TARGET_WAIT_FOR_START=1 -DTID01EQ=0
DEFINES_SKIPFORSIL = -DXCP_CUSTOM_PLATFORM -D__NVIC_PRIO_BITS=3 -D__CORTEX_M=7U -D__FPU_USED=1U -D__FPU_PRESENT=1U -DEXIT_FAILURE=1 -DEXTMODE_DISABLEPRINTF -DEXTMODE_DISABLETESTING -DEXTMODE_DISABLE_ARGS_PROCESSING=1 -DSTACK_SIZE=64 -DRT
DEFINES_STANDARD = -DMODEL=observing -DNUMST=1 -DNCSTATES=0 -DHAVESTDIO -DMODEL_HAS_DYNAMICALLY_LOADED_SFCNS=0

DEFINES = $(DEFINES_) $(DEFINES_BUILD_ARGS) $(DEFINES_CUSTOM) $(DEFINES_OPTS) $(DEFINES_SKIPFORSIL) $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/xcp_ext_mode.c $(START_DIR)/observing_ert_rtw/observing.c $(START_DIR)/observing_ert_rtw/observing_data.c $(START_DIR)/observing_ert_rtw/rtGetInf.c $(START_DIR)/observing_ert_rtw/rtGetNaN.c $(START_DIR)/observing_ert_rtw/rt_nonfinite.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/xcp_ext_common.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/xcp_ext_classic_trigger.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src/xcp.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src/xcp_standard.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src/xcp_daq.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src/xcp_calibration.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/transport/src/xcp_fifo.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/transport/src/xcp_transport.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/platform/default/xcp_mem_default.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/platform/default/xcp_drv_rtiostream.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/common/xcp_utils.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/transport/src/xcp_frame_serial.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/xcp_ext_param_default_serial.c /home/miku/MSc/MagLevTbx/Blocks/MagSens/v4.3/sfun_MagLevTbx_MagSens_v43_WrappedFcns.cpp /home/miku/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/teensy/hardware/avr/1.58.2/libraries/Wire/Wire.cpp /home/miku/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/teensy/hardware/avr/1.58.2/libraries/Wire/WireIMXRT.cpp /home/miku/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/teensy/hardware/avr/1.58.2/libraries/SPI/SPI.cpp /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/BoardSupportUsingKit2Go.cpp /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/CommunicationInterface.cpp /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/IICUsingTwoWire.cpp /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/Kit2GoBoardSupport.cpp /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/Logger.cpp /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/TLx493D_A1B6.c /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/TLx493D_A2B6.c /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/TLx493D_A2BW.c /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/TLx493D_P2B6.c /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/TLx493D_P3B6.c /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/TLx493D_P3I8.c /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/TLx493D_W2B6.c /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/TLx493D_W2BW.c /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/tlx493d_common.c /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/tlx493d_gen_2_common.c /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/tlx493d_gen_3_common.c /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/tlx493d.c /home/miku/MSc/MagLevTbx/Blocks/CurrSens/sfun_MagLevTbx_CurrSens_WrappedFcns.cpp /home/miku/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinotarget/src/MW_ArduinoHWInit.cpp /home/miku/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinobase/src/io_wrappers.cpp "$(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4/memcpy-armv7m.S" "$(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4/memset.S" "$(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4/startup.c" "$(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4/interrupt.c" /home/miku/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinotarget/scheduler/src/arduinoARMm7Scheduler.cpp /home/miku/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinotarget/scheduler/src/arm_cortex_m_multitasking.c /home/miku/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinotarget/src/platform_timer.cpp /home/miku/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinotarget/src/rtiostream_serial_daemon.cpp

MAIN_SRC = $(START_DIR)/observing_ert_rtw/ert_main.c

ALL_SRCS = $(SRCS) $(MAIN_SRC)

###########################################################################
## OBJECTS
###########################################################################

OBJS = xcp_ext_mode.o observing.o observing_data.o rtGetInf.o rtGetNaN.o rt_nonfinite.o xcp_ext_common.o xcp_ext_classic_trigger.o xcp.o xcp_standard.o xcp_daq.o xcp_calibration.o xcp_fifo.o xcp_transport.o xcp_mem_default.o xcp_drv_rtiostream.o xcp_utils.o xcp_frame_serial.o xcp_ext_param_default_serial.o sfun_MagLevTbx_MagSens_v43_WrappedFcns.o Wire.o WireIMXRT.o SPI.o BoardSupportUsingKit2Go.o CommunicationInterface.o IICUsingTwoWire.o Kit2GoBoardSupport.o Logger.o TLx493D_A1B6.o TLx493D_A2B6.o TLx493D_A2BW.o TLx493D_P2B6.o TLx493D_P3B6.o TLx493D_P3I8.o TLx493D_W2B6.o TLx493D_W2BW.o tlx493d_common.o tlx493d_gen_2_common.o tlx493d_gen_3_common.o tlx493d.o sfun_MagLevTbx_CurrSens_WrappedFcns.o MW_ArduinoHWInit.o io_wrappers.o memcpy-armv7m.S.o memset.S.o startup.o interrupt.o arduinoARMm7Scheduler.o arm_cortex_m_multitasking.o platform_timer.o rtiostream_serial_daemon.o

MAIN_OBJ = ert_main.o

ALL_OBJS = $(OBJS) $(MAIN_OBJ)

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = $(SSLIB_PATH)/MW_RebuildSrc_Core.o $(SSLIB_PATH)/libcore.a

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS = 

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_SKIPFORSIL = -Wall -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -MMD -D__IMXRT1062__ -DTEENSYDUINO=158 -DARDUINO=10819 -DARDUINO_TEENSY41 -DF_CPU=$(TEENSY_CPU_FREQ) -DUSB_SERIAL -DLAYOUT_US_ENGLISH
CFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CFLAGS += $(CFLAGS_SKIPFORSIL) $(CFLAGS_BASIC)

#-----------
# Linker
#-----------

LDFLAGS_ = -L"$(SLIB_PATH)"
LDFLAGS_SKIPFORSIL = -T$(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4/imxrt1062_t41.ld -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -larm_cortexM7lfsp_math -lm -lstdc++

LDFLAGS += $(LDFLAGS_) $(LDFLAGS_SKIPFORSIL)

#--------------------------
# Shared Library Linker
#--------------------------

SHAREDLIB_LDFLAGS_ = -L"$(SLIB_PATH)"
SHAREDLIB_LDFLAGS_SKIPFORSIL = -T$(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4/imxrt1062_t41.ld -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -larm_cortexM7lfsp_math -lm -lstdc++

SHAREDLIB_LDFLAGS += $(SHAREDLIB_LDFLAGS_) $(SHAREDLIB_LDFLAGS_SKIPFORSIL)

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_SKIPFORSIL = -Wall -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -MMD -D__IMXRT1062__ -DTEENSYDUINO=158 -DARDUINO=10819 -DARDUINO_TEENSY41 -DF_CPU=$(TEENSY_CPU_FREQ) -DUSB_SERIAL -DLAYOUT_US_ENGLISH
CPPFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CPPFLAGS += $(CPPFLAGS_SKIPFORSIL) $(CPPFLAGS_BASIC)

#---------------
# C++ Linker
#---------------

CPP_LDFLAGS_ = -L"$(SLIB_PATH)"
CPP_LDFLAGS_SKIPFORSIL = -T$(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4/imxrt1062_t41.ld -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -larm_cortexM7lfsp_math -lm -lstdc++

CPP_LDFLAGS += $(CPP_LDFLAGS_) $(CPP_LDFLAGS_SKIPFORSIL)

#------------------------------
# C++ Shared Library Linker
#------------------------------

CPP_SHAREDLIB_LDFLAGS_ = -L"$(SLIB_PATH)"
CPP_SHAREDLIB_LDFLAGS_SKIPFORSIL = -T$(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4/imxrt1062_t41.ld -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -larm_cortexM7lfsp_math -lm -lstdc++

CPP_SHAREDLIB_LDFLAGS += $(CPP_SHAREDLIB_LDFLAGS_) $(CPP_SHAREDLIB_LDFLAGS_SKIPFORSIL)

###########################################################################
## INLINED COMMANDS
###########################################################################


DERIVED_SRCS = $(subst .o,.dep,$(OBJS))

build:

%.dep:



-include arduino_macros.mk
-include codertarget_assembly_flags.mk
-include *.dep


###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build buildobj clean info prebuild postbuild download execute


all : build postbuild
	echo "### Successfully generated all binary outputs."


build : prebuild $(PRODUCT)


buildobj : prebuild $(OBJS) $(PREBUILT_OBJS) $(LIBS)
	echo "### Successfully generated all binary outputs."


prebuild : 


postbuild : $(PRODUCT)
	echo "### Invoking postbuild tool "Binary Converter" ..."
	$(OBJCOPY) $(OBJCOPYFLAGS_BIN)
	echo "### Done invoking postbuild tool."
	echo "### Invoking postbuild tool "Hex Converter" ..."
	$(OBJCOPY) $(OBJCOPYFLAGS_HEX)
	echo "### Done invoking postbuild tool."


download : postbuild


execute : download
	echo "### Invoking postbuild tool "Execute" ..."
	$(EXECUTE) $(EXECUTE_FLAGS)
	echo "### Done invoking postbuild tool."


###########################################################################
## FINAL TARGET
###########################################################################

#-------------------------------------------
# Create a standalone executable            
#-------------------------------------------

$(PRODUCT) : $(OBJS) $(PREBUILT_OBJS) $(LIBS) $(MAIN_OBJ)
	echo "### Creating standalone executable "$(PRODUCT)" ..."
	$(CPP_LD) $(CPP_LDFLAGS) -o $(PRODUCT) $(OBJS) $(MAIN_OBJ) $(LIBS) $(SYSTEM_LIBS) $(TOOLCHAIN_LIBS)
	echo "### Created: $(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

%.o : %.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : %.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : %.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : %.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(RELATIVE_PATH_TO_ANCHOR)/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(RELATIVE_PATH_TO_ANCHOR)/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : /home/miku/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/teensy/hardware/avr/1.58.2/cores/teensy4/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : /home/miku/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/teensy/hardware/avr/1.58.2/cores/teensy4/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : /home/miku/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/teensy/hardware/avr/1.58.2/cores/teensy4/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : /home/miku/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/teensy/hardware/avr/1.58.2/cores/teensy4/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : /home/miku/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/teensy/hardware/avr/1.58.2/libraries/Wire/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : /home/miku/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/teensy/hardware/avr/1.58.2/libraries/Wire/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : /home/miku/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/teensy/hardware/avr/1.58.2/libraries/Wire/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : /home/miku/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/teensy/hardware/avr/1.58.2/libraries/Wire/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : /home/miku/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/teensy/hardware/avr/1.58.2/libraries/SPI/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : /home/miku/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/teensy/hardware/avr/1.58.2/libraries/SPI/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : /home/miku/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/teensy/hardware/avr/1.58.2/libraries/SPI/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : /home/miku/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/teensy/hardware/avr/1.58.2/libraries/SPI/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : /home/miku/MSc/MagLevTbx/Blocks/MagSens/v4.3/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : /home/miku/MSc/MagLevTbx/Blocks/MagSens/v4.3/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : /home/miku/MSc/MagLevTbx/Blocks/MagSens/v4.3/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : /home/miku/MSc/MagLevTbx/Blocks/MagSens/v4.3/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : /home/miku/MSc/MagLevTbx/Blocks/CurrSens/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : /home/miku/MSc/MagLevTbx/Blocks/CurrSens/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : /home/miku/MSc/MagLevTbx/Blocks/CurrSens/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : /home/miku/MSc/MagLevTbx/Blocks/CurrSens/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(START_DIR)/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(START_DIR)/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/observing_ert_rtw/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(START_DIR)/observing_ert_rtw/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(START_DIR)/observing_ert_rtw/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/observing_ert_rtw/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/rtw/c/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/rtw/c/src/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/rtw/c/src/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/rtw/c/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/simulink/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/simulink/src/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/simulink/src/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/simulink/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/simulink/blocks/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/toolbox/simulink/blocks/src/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/toolbox/simulink/blocks/src/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/simulink/blocks/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/transport/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/transport/src/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/transport/src/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/transport/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/common/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/common/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/common/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/common/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/platform/default/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/platform/default/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/platform/default/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/platform/default/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xcp_ext_mode.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/xcp_ext_mode.c
	$(CC) $(CFLAGS) -o "$@" "$<"


ert_main.o : $(START_DIR)/observing_ert_rtw/ert_main.c
	$(CC) $(CFLAGS) -o "$@" "$<"


observing.o : $(START_DIR)/observing_ert_rtw/observing.c
	$(CC) $(CFLAGS) -o "$@" "$<"


observing_data.o : $(START_DIR)/observing_ert_rtw/observing_data.c
	$(CC) $(CFLAGS) -o "$@" "$<"


rtGetInf.o : $(START_DIR)/observing_ert_rtw/rtGetInf.c
	$(CC) $(CFLAGS) -o "$@" "$<"


rtGetNaN.o : $(START_DIR)/observing_ert_rtw/rtGetNaN.c
	$(CC) $(CFLAGS) -o "$@" "$<"


rt_nonfinite.o : $(START_DIR)/observing_ert_rtw/rt_nonfinite.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xcp_ext_common.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/xcp_ext_common.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xcp_ext_classic_trigger.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/xcp_ext_classic_trigger.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xcp.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src/xcp.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xcp_standard.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src/xcp_standard.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xcp_daq.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src/xcp_daq.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xcp_calibration.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src/xcp_calibration.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xcp_fifo.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/transport/src/xcp_fifo.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xcp_transport.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/transport/src/xcp_transport.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xcp_mem_default.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/platform/default/xcp_mem_default.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xcp_drv_rtiostream.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/platform/default/xcp_drv_rtiostream.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xcp_utils.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/common/xcp_utils.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xcp_frame_serial.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/transport/src/xcp_frame_serial.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xcp_ext_param_default_serial.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/xcp_ext_param_default_serial.c
	$(CC) $(CFLAGS) -o "$@" "$<"


sfun_MagLevTbx_MagSens_v43_WrappedFcns.o : /home/miku/MSc/MagLevTbx/Blocks/MagSens/v4.3/sfun_MagLevTbx_MagSens_v43_WrappedFcns.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


Wire.o : /home/miku/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/teensy/hardware/avr/1.58.2/libraries/Wire/Wire.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


WireIMXRT.o : /home/miku/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/teensy/hardware/avr/1.58.2/libraries/Wire/WireIMXRT.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


SPI.o : /home/miku/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/teensy/hardware/avr/1.58.2/libraries/SPI/SPI.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


BoardSupportUsingKit2Go.o : /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/BoardSupportUsingKit2Go.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


CommunicationInterface.o : /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/CommunicationInterface.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


IICUsingTwoWire.o : /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/IICUsingTwoWire.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


Kit2GoBoardSupport.o : /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/Kit2GoBoardSupport.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


Logger.o : /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/Logger.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


TLx493D_A1B6.o : /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/TLx493D_A1B6.c
	$(CC) $(CFLAGS) -o "$@" "$<"


TLx493D_A2B6.o : /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/TLx493D_A2B6.c
	$(CC) $(CFLAGS) -o "$@" "$<"


TLx493D_A2BW.o : /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/TLx493D_A2BW.c
	$(CC) $(CFLAGS) -o "$@" "$<"


TLx493D_P2B6.o : /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/TLx493D_P2B6.c
	$(CC) $(CFLAGS) -o "$@" "$<"


TLx493D_P3B6.o : /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/TLx493D_P3B6.c
	$(CC) $(CFLAGS) -o "$@" "$<"


TLx493D_P3I8.o : /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/TLx493D_P3I8.c
	$(CC) $(CFLAGS) -o "$@" "$<"


TLx493D_W2B6.o : /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/TLx493D_W2B6.c
	$(CC) $(CFLAGS) -o "$@" "$<"


TLx493D_W2BW.o : /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/TLx493D_W2BW.c
	$(CC) $(CFLAGS) -o "$@" "$<"


tlx493d_common.o : /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/tlx493d_common.c
	$(CC) $(CFLAGS) -o "$@" "$<"


tlx493d_gen_2_common.o : /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/tlx493d_gen_2_common.c
	$(CC) $(CFLAGS) -o "$@" "$<"


tlx493d_gen_3_common.o : /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/tlx493d_gen_3_common.c
	$(CC) $(CFLAGS) -o "$@" "$<"


tlx493d.o : /home/miku/MSc/MagLevTbx/Blocks/MagSens/Lib/XENSIV_TLx493D/tlx493d.c
	$(CC) $(CFLAGS) -o "$@" "$<"


sfun_MagLevTbx_CurrSens_WrappedFcns.o : /home/miku/MSc/MagLevTbx/Blocks/CurrSens/sfun_MagLevTbx_CurrSens_WrappedFcns.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


MW_ArduinoHWInit.o : /home/miku/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinotarget/src/MW_ArduinoHWInit.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


io_wrappers.o : /home/miku/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinobase/src/io_wrappers.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


memcpy-armv7m.S.o : $(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4/memcpy-armv7m.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


memset.S.o : $(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4/memset.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


startup.o : $(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4/startup.c
	$(CC) $(CFLAGS) -o "$@" "$<"


interrupt.o : $(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4/interrupt.c
	$(CC) $(CFLAGS) -o "$@" "$<"


arduinoARMm7Scheduler.o : /home/miku/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinotarget/scheduler/src/arduinoARMm7Scheduler.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


arm_cortex_m_multitasking.o : /home/miku/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinotarget/scheduler/src/arm_cortex_m_multitasking.c
	$(CC) $(CFLAGS) -o "$@" "$<"


platform_timer.o : /home/miku/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinotarget/src/platform_timer.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rtiostream_serial_daemon.o : /home/miku/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinotarget/src/rtiostream_serial_daemon.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


###########################################################################
## DEPENDENCIES
###########################################################################

$(ALL_OBJS) : rtw_proj.tmw $(MAKEFILE)


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	echo "### PRODUCT = $(PRODUCT)"
	echo "### PRODUCT_TYPE = $(PRODUCT_TYPE)"
	echo "### BUILD_TYPE = $(BUILD_TYPE)"
	echo "### INCLUDES = $(INCLUDES)"
	echo "### DEFINES = $(DEFINES)"
	echo "### ALL_SRCS = $(ALL_SRCS)"
	echo "### ALL_OBJS = $(ALL_OBJS)"
	echo "### LIBS = $(LIBS)"
	echo "### MODELREF_LIBS = $(MODELREF_LIBS)"
	echo "### SYSTEM_LIBS = $(SYSTEM_LIBS)"
	echo "### TOOLCHAIN_LIBS = $(TOOLCHAIN_LIBS)"
	echo "### ASFLAGS = $(ASFLAGS)"
	echo "### CFLAGS = $(CFLAGS)"
	echo "### LDFLAGS = $(LDFLAGS)"
	echo "### SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS)"
	echo "### CPPFLAGS = $(CPPFLAGS)"
	echo "### CPP_LDFLAGS = $(CPP_LDFLAGS)"
	echo "### CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS)"
	echo "### ARFLAGS = $(ARFLAGS)"
	echo "### MEX_CFLAGS = $(MEX_CFLAGS)"
	echo "### MEX_CPPFLAGS = $(MEX_CPPFLAGS)"
	echo "### MEX_LDFLAGS = $(MEX_LDFLAGS)"
	echo "### MEX_CPPLDFLAGS = $(MEX_CPPLDFLAGS)"
	echo "### OBJCOPYFLAGS_BIN = $(OBJCOPYFLAGS_BIN)"
	echo "### OBJCOPYFLAGS_HEX = $(OBJCOPYFLAGS_HEX)"
	echo "### DOWNLOAD_FLAGS = $(DOWNLOAD_FLAGS)"
	echo "### EXECUTE_FLAGS = $(EXECUTE_FLAGS)"
	echo "### MAKE_FLAGS = $(MAKE_FLAGS)"


clean : 
	$(ECHO) "### Deleting all derived files ..."
	$(RM) $(PRODUCT)
	$(RM) $(ALL_OBJS)
	$(RM) *.dep
	$(ECHO) "### Deleted all derived files."


