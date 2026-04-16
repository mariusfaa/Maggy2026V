###########################################################################
## Makefile generated for component 'CurrDrvSensCombo_Test'. 
## 
## Makefile     : CurrDrvSensCombo_Test.mk
## Generated on : Mon Apr 06 10:37:41 2026
## Final product: $(RELATIVE_PATH_TO_ANCHOR)/CurrDrvSensCombo_Test.elf
## Product type : executable
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile

PRODUCT_NAME              = CurrDrvSensCombo_Test
MAKEFILE                  = CurrDrvSensCombo_Test.mk
MATLAB_ROOT               = C:/PROGRA~1/MATLAB/R2025b
MATLAB_BIN                = C:/PROGRA~1/MATLAB/R2025b/bin
MATLAB_ARCH_BIN           = $(MATLAB_BIN)/win64
START_DIR                 = C:/Users/mariujf/MagLevTbx-main/Controller/hardware
SOLVER                    = 
SOLVER_OBJ                = 
CLASSIC_INTERFACE         = 0
TGT_FCN_LIB               = ISO_C
MODEL_HAS_DYNAMICALLY_LOADED_SFCNS = 0
RELATIVE_PATH_TO_ANCHOR   = ..
SLIB_PATH                 = C:/Users/mariujf/DOCUME~1/MATLAB/R2025b/ARDUIN~1/TEENSY~1.1AR/FASTER~1
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

SHELL                       = %SystemRoot%/system32/cmd.exe
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
MAKE_PATH = %MATLAB%\bin\win64
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

#--------------------------------------
# "Faster Runs" Build Configuration
#--------------------------------------

ARFLAGS              = rcs
ASFLAGS              = -MMD -MP -MF"$(@:%.o=%.dep)" -MT"$@"  \
                       -Wall \
                       -x assembler-with-cpp \
                       $(ASFLAGS_ADDITIONAL) \
                       $(DEFINES) \
                       $(INCLUDES) \
                       -c
OBJCOPYFLAGS_BIN     = $(ELF2EEP_OPTIONS) $(PRODUCT) $(PRODUCT_BIN)
CFLAGS               = -O2 \
                       -c \
                       -w \
                       -ffunction-sections \
                       -fdata-sections \
                       -nostdlib \
                       -MMD -MP -MF"$(@:%.o=%.dep)" -MT"$@" 
CPPFLAGS             = -std=gnu++14 -fno-exceptions -fpermissive -fno-rtti -fno-threadsafe-statics -felide-constructors -Wno-error=narrowing \
                       -O2 \
                       -c \
                       -w \
                       -ffunction-sections \
                       -fdata-sections \
                       -nostdlib \
                       -MMD -MP -MF"$(@:%.o=%.dep)" -MT"$@" 
CPP_LDFLAGS          =  -O2 -Wl,--gc-sections,--relax
CPP_SHAREDLIB_LDFLAGS  =
DOWNLOAD_FLAGS       =
EXECUTE_FLAGS        =
OBJCOPYFLAGS_HEX     = -O ihex -R .eeprom $(PRODUCT) $(PRODUCT_HEX)
LDFLAGS              =  -O2 -Wl,--gc-sections,--relax
MEX_CPPFLAGS         =
MEX_CPPLDFLAGS       =
MEX_CFLAGS           =
MEX_LDFLAGS          =
MAKE_FLAGS           = -f $(MAKEFILE)
SHAREDLIB_LDFLAGS    =



###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = $(RELATIVE_PATH_TO_ANCHOR)/CurrDrvSensCombo_Test.elf
PRODUCT_TYPE = "executable"
BUILD_TYPE = "Top-Level Standalone Executable"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = -I$(START_DIR) -I$(START_DIR)/CurrDrvSensCombo_Test_ert_rtw -I$(MATLAB_ROOT)/extern/include -I$(MATLAB_ROOT)/simulink/include -I$(MATLAB_ROOT)/rtw/c/src -I$(MATLAB_ROOT)/rtw/c/src/ext_mode/common -I$(MATLAB_ROOT)/rtw/c/ert -IC:/ProgramData/MATLAB/SupportPackages/R2025b/aCLI/data/packages/teensy/hardware/avr/1.58.2/cores/teensy4 -IC:/Users/mariujf/MagLevTbx-main/Lib/COBS -I$(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4 -I$(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4/avr -I$(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4/debug -I$(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4/util -IC:/ProgramData/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinotarget/include -IC:/ProgramData/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinotarget/scheduler/include -IC:/ProgramData/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinobase/include -I$(MATLAB_ROOT)/toolbox/target/shared/armcortexmbase/scheduler/include

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_ = -D__MW_TARGET_USE_HARDWARE_RESOURCES_H__ -DMW_TIMERID=60 -DMW_TIMERCOUNT=240000 -DARDUINO_NUM_SERIAL_PORTS=9 -DARDUINO_SERIAL_RECEIVE_BUFFER_SIZE=64 -D_RTT_BAUDRATE_SERIAL0_=9600 -D_RTT_BAUDRATE_SERIAL1_=9600 -D_RTT_BAUDRATE_SERIAL2_=9600 -D_RTT_BAUDRATE_SERIAL3_=9600 -D_RTT_BAUDRATE_SERIAL4_=9600 -D_RTT_BAUDRATE_SERIAL5_=9600 -D_RTT_BAUDRATE_SERIAL6_=9600 -D_RTT_BAUDRATE_SERIAL7_=9600 -D_RTT_BAUDRATE_SERIAL8_=9600 -D_RTT_CONFIG_SERIAL0_=SERIAL_8N1 -D_RTT_CONFIG_SERIAL1_=SERIAL_8N1 -D_RTT_CONFIG_SERIAL2_=SERIAL_8N1 -D_RTT_CONFIG_SERIAL3_=SERIAL_8N1 -D_RTT_CONFIG_SERIAL4_=SERIAL_8N1 -D_RTT_CONFIG_SERIAL5_=SERIAL_8N1 -D_RTT_CONFIG_SERIAL6_=SERIAL_8N1 -D_RTT_CONFIG_SERIAL7_=SERIAL_8N1 -D_RTT_CONFIG_SERIAL8_=SERIAL_8N1 -D_RTT_ANALOG_REF_=0 -DMW_NUM_PINS=55 -D_ONBOARD_EEPROM_SIZE_=4284
DEFINES_BUILD_ARGS = -DCLASSIC_INTERFACE=0 -DALLOCATIONFCN=0 -DTERMFCN=1 -DONESTEPFCN=1 -DMAT_FILE=0 -DMULTI_INSTANCE_CODE=0 -DINTEGER_CODE=0 -DMT=1
DEFINES_CUSTOM = 
DEFINES_OPTS = -DTID01EQ=0
DEFINES_SKIPFORSIL = -DXCP_CUSTOM_PLATFORM -D__NVIC_PRIO_BITS=3 -D__CORTEX_M=7U -D__FPU_USED=1U -D__FPU_PRESENT=1U -DEXIT_FAILURE=1 -DEXTMODE_DISABLEPRINTF -DEXTMODE_DISABLETESTING -DEXTMODE_DISABLE_ARGS_PROCESSING=1 -DSTACK_SIZE=64 -DRT
DEFINES_STANDARD = -DMODEL=CurrDrvSensCombo_Test -DNUMST=2 -DNCSTATES=0 -DHAVESTDIO -DMODEL_HAS_DYNAMICALLY_LOADED_SFCNS=0

DEFINES = $(DEFINES_) $(DEFINES_BUILD_ARGS) $(DEFINES_CUSTOM) $(DEFINES_OPTS) $(DEFINES_SKIPFORSIL) $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = $(START_DIR)/CurrDrvSensCombo_Test_ert_rtw/CurrDrvSensCombo_Test.c $(START_DIR)/CurrDrvSensCombo_Test_ert_rtw/CurrDrvSensCombo_Test_data.c $(START_DIR)/CurrDrvSensCombo_Test_ert_rtw/rt_nonfinite.c sfun_MagLevTbx_CurrDrv_WrappedFcns.cpp sfun_MagLevTbx_CurrSens_WrappedFcns.cpp sfun_MagLevTbx_UsbSerialPacketSend_WrappedFcns.cpp cobs.c C:/ProgramData/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinotarget/src/MW_ArduinoHWInit.cpp C:/ProgramData/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinobase/src/io_wrappers.cpp "$(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4/memcpy-armv7m.S" "$(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4/memset.S" "$(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4/startup.c" "$(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4/interrupt.c" C:/ProgramData/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinotarget/scheduler/src/arduinoARMm7Scheduler.cpp C:/ProgramData/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinotarget/scheduler/src/arm_cortex_m_multitasking.c

MAIN_SRC = $(START_DIR)/CurrDrvSensCombo_Test_ert_rtw/ert_main.c

ALL_SRCS = $(SRCS) $(MAIN_SRC)

###########################################################################
## OBJECTS
###########################################################################

OBJS = CurrDrvSensCombo_Test.o CurrDrvSensCombo_Test_data.o rt_nonfinite.o sfun_MagLevTbx_CurrDrv_WrappedFcns.o sfun_MagLevTbx_CurrSens_WrappedFcns.o sfun_MagLevTbx_UsbSerialPacketSend_WrappedFcns.o cobs.o MW_ArduinoHWInit.o io_wrappers.o memcpy-armv7m.S.o memset.S.o startup.o interrupt.o arduinoARMm7Scheduler.o arm_cortex_m_multitasking.o

MAIN_OBJ = ert_main.o

ALL_OBJS = $(OBJS) $(MAIN_OBJ)

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = $(SLIB_PATH)/MW_RebuildSrc_Core.o $(SLIB_PATH)/libcore.a

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
	$(CPP_LD) $(CPP_LDFLAGS) -o $(PRODUCT) $(subst /,\,$(OBJS)) $(subst /,\,$(MAIN_OBJ)) $(subst /,\,$(LIBS)) $(subst /,\,$(SYSTEM_LIBS)) $(subst /,\,$(TOOLCHAIN_LIBS))
	echo "### Created: $(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

%.o : %.c
	$(CC) $(CFLAGS) -o "$@" $(subst /,\,"$<")


%.S.o : %.s
	$(AS) $(ASFLAGS) -o "$@" $(subst /,\,"$<")


%.S.o : %.S
	$(AS) $(ASFLAGS) -o "$@" $(subst /,\,"$<")


%.o : %.cpp
	$(CPP) $(CPPFLAGS) -o "$@" $(subst /,\,"$<")


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.c
	$(CC) $(CFLAGS) -o "$@" $(subst /,\,"$<")


%.S.o : $(RELATIVE_PATH_TO_ANCHOR)/%.s
	$(AS) $(ASFLAGS) -o "$@" $(subst /,\,"$<")


%.S.o : $(RELATIVE_PATH_TO_ANCHOR)/%.S
	$(AS) $(ASFLAGS) -o "$@" $(subst /,\,"$<")


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" $(subst /,\,"$<")


%.o : C:/ProgramData/MATLAB/SupportPackages/R2025b/aCLI/data/packages/teensy/hardware/avr/1.58.2/cores/teensy4/%.c
	$(CC) $(CFLAGS) -o "$@" $(subst /,\,"$<")


%.S.o : C:/ProgramData/MATLAB/SupportPackages/R2025b/aCLI/data/packages/teensy/hardware/avr/1.58.2/cores/teensy4/%.s
	$(AS) $(ASFLAGS) -o "$@" $(subst /,\,"$<")


%.S.o : C:/ProgramData/MATLAB/SupportPackages/R2025b/aCLI/data/packages/teensy/hardware/avr/1.58.2/cores/teensy4/%.S
	$(AS) $(ASFLAGS) -o "$@" $(subst /,\,"$<")


%.o : C:/ProgramData/MATLAB/SupportPackages/R2025b/aCLI/data/packages/teensy/hardware/avr/1.58.2/cores/teensy4/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" $(subst /,\,"$<")


%.o : C:/Users/mariujf/MagLevTbx-main/Lib/COBS/%.c
	$(CC) $(CFLAGS) -o "$@" $(subst /,\,"$<")


%.S.o : C:/Users/mariujf/MagLevTbx-main/Lib/COBS/%.s
	$(AS) $(ASFLAGS) -o "$@" $(subst /,\,"$<")


%.S.o : C:/Users/mariujf/MagLevTbx-main/Lib/COBS/%.S
	$(AS) $(ASFLAGS) -o "$@" $(subst /,\,"$<")


%.o : C:/Users/mariujf/MagLevTbx-main/Lib/COBS/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" $(subst /,\,"$<")


%.o : C:/Users/mariujf/MagLevTbx-main/Blocks/CurrDrv/%.c
	$(CC) $(CFLAGS) -o "$@" $(subst /,\,"$<")


%.S.o : C:/Users/mariujf/MagLevTbx-main/Blocks/CurrDrv/%.s
	$(AS) $(ASFLAGS) -o "$@" $(subst /,\,"$<")


%.S.o : C:/Users/mariujf/MagLevTbx-main/Blocks/CurrDrv/%.S
	$(AS) $(ASFLAGS) -o "$@" $(subst /,\,"$<")


%.o : C:/Users/mariujf/MagLevTbx-main/Blocks/CurrDrv/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" $(subst /,\,"$<")


%.o : C:/Users/mariujf/MagLevTbx-main/Blocks/CurrSens/%.c
	$(CC) $(CFLAGS) -o "$@" $(subst /,\,"$<")


%.S.o : C:/Users/mariujf/MagLevTbx-main/Blocks/CurrSens/%.s
	$(AS) $(ASFLAGS) -o "$@" $(subst /,\,"$<")


%.S.o : C:/Users/mariujf/MagLevTbx-main/Blocks/CurrSens/%.S
	$(AS) $(ASFLAGS) -o "$@" $(subst /,\,"$<")


%.o : C:/Users/mariujf/MagLevTbx-main/Blocks/CurrSens/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" $(subst /,\,"$<")


%.o : C:/Users/mariujf/MagLevTbx-main/Blocks/UsbSerialPacketSend/%.c
	$(CC) $(CFLAGS) -o "$@" $(subst /,\,"$<")


%.S.o : C:/Users/mariujf/MagLevTbx-main/Blocks/UsbSerialPacketSend/%.s
	$(AS) $(ASFLAGS) -o "$@" $(subst /,\,"$<")


%.S.o : C:/Users/mariujf/MagLevTbx-main/Blocks/UsbSerialPacketSend/%.S
	$(AS) $(ASFLAGS) -o "$@" $(subst /,\,"$<")


%.o : C:/Users/mariujf/MagLevTbx-main/Blocks/UsbSerialPacketSend/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" $(subst /,\,"$<")


%.o : $(START_DIR)/%.c
	$(CC) $(CFLAGS) -o "$@" $(subst /,\,"$<")


%.S.o : $(START_DIR)/%.s
	$(AS) $(ASFLAGS) -o "$@" $(subst /,\,"$<")


%.S.o : $(START_DIR)/%.S
	$(AS) $(ASFLAGS) -o "$@" $(subst /,\,"$<")


%.o : $(START_DIR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" $(subst /,\,"$<")


%.o : $(START_DIR)/CurrDrvSensCombo_Test_ert_rtw/%.c
	$(CC) $(CFLAGS) -o "$@" $(subst /,\,"$<")


%.S.o : $(START_DIR)/CurrDrvSensCombo_Test_ert_rtw/%.s
	$(AS) $(ASFLAGS) -o "$@" $(subst /,\,"$<")


%.S.o : $(START_DIR)/CurrDrvSensCombo_Test_ert_rtw/%.S
	$(AS) $(ASFLAGS) -o "$@" $(subst /,\,"$<")


%.o : $(START_DIR)/CurrDrvSensCombo_Test_ert_rtw/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" $(subst /,\,"$<")


%.o : $(MATLAB_ROOT)/rtw/c/src/%.c
	$(CC) $(CFLAGS) -o "$@" $(subst /,\,"$<")


%.S.o : $(MATLAB_ROOT)/rtw/c/src/%.s
	$(AS) $(ASFLAGS) -o "$@" $(subst /,\,"$<")


%.S.o : $(MATLAB_ROOT)/rtw/c/src/%.S
	$(AS) $(ASFLAGS) -o "$@" $(subst /,\,"$<")


%.o : $(MATLAB_ROOT)/rtw/c/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" $(subst /,\,"$<")


%.o : $(MATLAB_ROOT)/simulink/src/%.c
	$(CC) $(CFLAGS) -o "$@" $(subst /,\,"$<")


%.S.o : $(MATLAB_ROOT)/simulink/src/%.s
	$(AS) $(ASFLAGS) -o "$@" $(subst /,\,"$<")


%.S.o : $(MATLAB_ROOT)/simulink/src/%.S
	$(AS) $(ASFLAGS) -o "$@" $(subst /,\,"$<")


%.o : $(MATLAB_ROOT)/simulink/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" $(subst /,\,"$<")


%.o : $(MATLAB_ROOT)/toolbox/simulink/blocks/src/%.c
	$(CC) $(CFLAGS) -o "$@" $(subst /,\,"$<")


%.S.o : $(MATLAB_ROOT)/toolbox/simulink/blocks/src/%.s
	$(AS) $(ASFLAGS) -o "$@" $(subst /,\,"$<")


%.S.o : $(MATLAB_ROOT)/toolbox/simulink/blocks/src/%.S
	$(AS) $(ASFLAGS) -o "$@" $(subst /,\,"$<")


%.o : $(MATLAB_ROOT)/toolbox/simulink/blocks/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" $(subst /,\,"$<")


CurrDrvSensCombo_Test.o : $(START_DIR)/CurrDrvSensCombo_Test_ert_rtw/CurrDrvSensCombo_Test.c
	$(CC) $(CFLAGS) -o "$@" $(subst /,\,"$<")


CurrDrvSensCombo_Test_data.o : $(START_DIR)/CurrDrvSensCombo_Test_ert_rtw/CurrDrvSensCombo_Test_data.c
	$(CC) $(CFLAGS) -o "$@" $(subst /,\,"$<")


ert_main.o : $(START_DIR)/CurrDrvSensCombo_Test_ert_rtw/ert_main.c
	$(CC) $(CFLAGS) -o "$@" $(subst /,\,"$<")


rt_nonfinite.o : $(START_DIR)/CurrDrvSensCombo_Test_ert_rtw/rt_nonfinite.c
	$(CC) $(CFLAGS) -o "$@" $(subst /,\,"$<")


MW_ArduinoHWInit.o : C:/ProgramData/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinotarget/src/MW_ArduinoHWInit.cpp
	$(CPP) $(CPPFLAGS) -o "$@" $(subst /,\,"$<")


io_wrappers.o : C:/ProgramData/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinobase/src/io_wrappers.cpp
	$(CPP) $(CPPFLAGS) -o "$@" $(subst /,\,"$<")


memcpy-armv7m.S.o : $(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4/memcpy-armv7m.S
	$(AS) $(ASFLAGS) -o "$@" $(subst /,\,"$<")


memset.S.o : $(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4/memset.S
	$(AS) $(ASFLAGS) -o "$@" $(subst /,\,"$<")


startup.o : $(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4/startup.c
	$(CC) $(CFLAGS) -o "$@" $(subst /,\,"$<")


interrupt.o : $(ARDUINO_TEENSY_ROOT)/hardware/avr/$(TEENSY_LIB_VERSION)/cores/teensy4/interrupt.c
	$(CC) $(CFLAGS) -o "$@" $(subst /,\,"$<")


arduinoARMm7Scheduler.o : C:/ProgramData/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinotarget/scheduler/src/arduinoARMm7Scheduler.cpp
	$(CPP) $(CPPFLAGS) -o "$@" $(subst /,\,"$<")


arm_cortex_m_multitasking.o : C:/ProgramData/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinotarget/scheduler/src/arm_cortex_m_multitasking.c
	$(CC) $(CFLAGS) -o "$@" $(subst /,\,"$<")


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


