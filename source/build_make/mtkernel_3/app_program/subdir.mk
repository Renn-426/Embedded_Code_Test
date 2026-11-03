################################################################################
# micro T-Kernel 3.0 BSP  makefile
#
# AAI03 autonomous robot car application.
#
# The application is split into two compilation classes because the kernel
# headers and the Pico SDK hardware headers define the same peripheral base
# address macros and cannot share a translation unit:
#
#   *_hw.c, *_net.c   Hardware / network driver modules.  Include only Pico
#                     SDK (or lwIP / cyw43) headers, never <tk/tkernel.h>.
#                     Built as gnu11 because the SDK uses C11 static_assert.
#   everything else   Kernel-side modules.  Include <tk/tkernel.h> and the
#                     plain-C driver headers, never an SDK header.  Built as
#                     strict ISO C99 (Barr-C rule 1.1.a).
#
# Both classes are built with -Werror so a warning cannot reach the robot.
################################################################################

APP_ROOT     := ../app_program
APP_SUBDIRS  := platform buddy1_comms buddy2_motion buddy3_line \
                buddy4_imu buddy5_obstacle integration

# demo_tasks.c is the kernel demo left over from the port: not part of the robot.
APP_SRCS     := $(filter-out $(APP_ROOT)/demo_tasks.c,$(wildcard $(APP_ROOT)/*.c)) \
                $(foreach dir,$(APP_SUBDIRS),$(wildcard $(APP_ROOT)/$(dir)/*.c))
APP_HW_SRCS  := $(filter %_hw.c %_net.c,$(APP_SRCS))
APP_KNL_SRCS := $(filter-out $(APP_HW_SRCS),$(APP_SRCS))

APP_OBJS     := $(subst ../,./mtkernel_3/,$(APP_SRCS:.c=.o))
APP_HW_OBJS  := $(subst ../,./mtkernel_3/,$(APP_HW_SRCS:.c=.o))
APP_KNL_OBJS := $(subst ../,./mtkernel_3/,$(APP_KNL_SRCS:.c=.o))

OBJS   += $(APP_OBJS)
C_DEPS += $(APP_OBJS:.o=.d)

APP_INCLUDES := -I"$(APP_ROOT)" \
                $(foreach dir,$(APP_SUBDIRS),-I"$(APP_ROOT)/$(dir)")

# Kernel headers are third-party code: include them as system headers so
# their own style (for example the non-prototype FP typedef) does not trip
# the application's -Werror.
APP_KERNEL_INCLUDES := $(subst -I,-isystem ,$(INCPATH))

# -Wsign-conversion enforces Barr-C rule 5.3.c (no implicit mixing of
# signed and unsigned values) at compile time.
APP_WARNINGS := -Wall -Wextra -Werror -Wshadow -Wsign-compare -Wsign-conversion \
                -Wstrict-prototypes -Wmissing-prototypes

# Bench tests (BUDDY_TEST=3 Buddy 3 alone, 23 Buddy 2 + 3; 0 is the mission),
# the USB console monitor for the mission (DEBUG_MON=1), and BENCH_NO_IMU=1,
# which lets the mission start with a dead IMU (bench use only).
# The stamp file name carries the values, and the recipe deletes the other
# stamps, so the files that read them are rebuilt whenever a value changes.
BUDDY_TEST   ?= 0
DEBUG_MON    ?= 0
BENCH_NO_IMU ?= 0
APP_TEST_STAMP := ./mtkernel_3/app_program/.buddy_test_$(BUDDY_TEST)_$(DEBUG_MON)_$(BENCH_NO_IMU)

$(APP_TEST_STAMP):
	@mkdir -p "$(@D)"
	@rm -f ./mtkernel_3/app_program/.buddy_test_*
	@touch "$@"

./mtkernel_3/app_program/app_main.o: $(APP_TEST_STAMP)
./mtkernel_3/app_program/integration/vehicle.o: $(APP_TEST_STAMP)

################################################################################
# Pico SDK subset used by the hardware modules
################################################################################

PICO_SDK_PATH ?= ../../sdk/pico-sdk

ifneq ($(MAKECMDGOALS),clean)

# GP0/GP1 carry the IMU's I2C bus on this robot, so the UART console pins
# are not routed (BOARD_UART0_PINS = 0) and the console must be USB-CDC.
ifneq ($(CONSOLE),usb_cdc)
$(error The AAI03 robot uses GP0/GP1 for I2C: build with CONSOLE=usb_cdc)
endif

# The application's shared-data rules (single aligned stores, interrupt
# masking with DI/EI) are proven for one core only.
ifneq ($(SMP),0)
$(error The AAI03 robot application is single-core: build with SMP=0)
endif

ifeq ($(wildcard $(PICO_SDK_PATH)/src/rp2_common/hardware_gpio/gpio.c),)
$(error The robot application needs the Pico SDK.  Set PICO_SDK_PATH)
endif

endif

APP_SDK_MODULES := common/pico_base_headers common/hardware_claim \
                   common/pico_time common/pico_util common/pico_sync \
                   rp2040/pico_platform rp2040/hardware_regs \
                   rp2040/hardware_structs \
                   rp2_common/pico_platform_common \
                   rp2_common/pico_platform_compiler \
                   rp2_common/pico_platform_sections \
                   rp2_common/pico_platform_panic \
                   rp2_common/hardware_base rp2_common/hardware_gpio \
                   rp2_common/hardware_pwm rp2_common/hardware_adc \
                   rp2_common/hardware_i2c rp2_common/hardware_timer \
                   rp2_common/hardware_resets rp2_common/hardware_sync \
                   rp2_common/hardware_sync_spin_lock \
                   rp2_common/hardware_irq rp2_common/hardware_clocks

# The SDK is included with -isystem so its own header warnings do not trip
# -Werror; the generated pico/version.h stubs live in the USB console tree.
APP_SDK_INCLUDES := -isystem "../lib/libtm/sysdepend/pico_rp2040/usb" \
                    -isystem "$(PICO_SDK_PATH)/src/boards/include" \
                    $(foreach mod,$(APP_SDK_MODULES), \
                        -isystem "$(PICO_SDK_PATH)/src/$(mod)/include")

APP_SDK_DEFINES := -DPICO_RP2040=1 -DPICO_32BIT=1 -DPICO_ON_DEVICE=1 \
                   -DPICO_BUILD=1 -DPICO_NO_HARDWARE=0 \
                   -DRASPBERRYPI_PICO_W=1 -DNDEBUG

# SDK driver sources the robot links.  gpio.c is already part of the radio
# profile, so it is added here only for the radio-less bench build.
APP_SDK_SRCS := rp2_common/hardware_adc/adc.c rp2_common/hardware_i2c/i2c.c \
                common/pico_time/timeout_helper.c
ifneq ($(WIFI),cyw43)
APP_SDK_SRCS += rp2_common/hardware_gpio/gpio.c
endif
APP_SDK_OBJS := $(addprefix ./mtkernel_3/app_sdk/,$(APP_SDK_SRCS:.c=.o))

OBJS   += $(APP_SDK_OBJS)
C_DEPS += $(APP_SDK_OBJS:.o=.d)

################################################################################
# Barr-C automated scan (rules 1.2, 1.3, 1.7, 3.5, 3.6, 5.2 and more).
# Runs once per build before any application object is compiled.  Set
# BARR_CHECK=0 to skip it on a machine without Python 3.
################################################################################

BARR_CHECK ?= 1
APP_LINT_STAMP := ./mtkernel_3/app_program/.barr_check_ok
APP_HEADERS := $(wildcard $(APP_ROOT)/*.h) \
               $(foreach dir,$(APP_SUBDIRS),$(wildcard $(APP_ROOT)/$(dir)/*.h))

ifeq ($(BARR_CHECK),1)
$(APP_LINT_STAMP): $(APP_SRCS) $(APP_HEADERS) ../tools/barr_check.py
	@echo 'Barr-C scan: $(APP_ROOT)'
	@mkdir -p "$(@D)"
	python3 ../tools/barr_check.py $(APP_ROOT)
	@touch "$@"
else
$(APP_LINT_STAMP):
	@mkdir -p "$(@D)"
	@touch "$@"
endif

################################################################################
# Rules
################################################################################

# The scan stamp is order-only: it must exist before any object is built,
# but refreshing it (which any source edit does) must not rebuild them all.
$(APP_KNL_OBJS): ./mtkernel_3/app_program/%.o: ../app_program/%.c | $(APP_LINT_STAMP)
	@echo 'Building kernel-side module: $<'
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -std=c99 -pedantic $(APP_WARNINGS) \
	    -D$(TARGET) -DBUDDY_TEST=$(BUDDY_TEST) -DDEBUG_MON=$(DEBUG_MON) \
	    -DBENCH_NO_IMU=$(BENCH_NO_IMU) \
	    $(APP_KERNEL_INCLUDES) $(APP_INCLUDES) \
	    -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

$(APP_HW_OBJS): ./mtkernel_3/app_program/%.o: ../app_program/%.c | $(APP_LINT_STAMP)
	@echo 'Building hardware module: $<'
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -std=gnu11 $(APP_WARNINGS) $(APP_SDK_DEFINES) \
	    $(APP_SDK_INCLUDES) $(APP_INCLUDES) -I"../config" \
	    -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

./mtkernel_3/app_sdk/%.o: $(PICO_SDK_PATH)/src/%.c
	@echo 'Building Pico SDK file: $<'
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -std=gnu11 $(APP_SDK_DEFINES) $(APP_SDK_INCLUDES) \
	    -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
