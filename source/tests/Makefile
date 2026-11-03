# Host unit tests for the hardware-independent robot modules.
#   make -C tests        build and run
APP     := ../app_program
CC      ?= gcc
CFLAGS  := -std=c99 -Wall -Wextra -Werror -Wshadow -Wsign-compare -O1 -g \
           -I$(APP)/platform -I$(APP)/buddy1_comms -I$(APP)/buddy2_motion \
           -I$(APP)/buddy3_line -I$(APP)/buddy4_imu -I$(APP)/buddy5_obstacle \
           -I$(APP)/integration
SRCS    := test_main.c \
           $(APP)/platform/bitpack.c $(APP)/platform/fix_math.c \
           $(APP)/platform/sys_status.c $(APP)/platform/tune.c \
           $(APP)/buddy1_comms/tlm_codec.c \
           $(APP)/buddy2_motion/pid_ctrl.c \
           $(APP)/buddy3_line/code39.c \
           $(APP)/buddy4_imu/hump.c \
           $(APP)/buddy5_obstacle/obs_profile.c \
           $(APP)/integration/nav_map.c

run: robot_tests codec_vectors
	./robot_tests
	python3 test_decode.py

robot_tests: $(SRCS) $(wildcard $(APP)/*/*.h)
	$(CC) $(CFLAGS) -o $@ $(SRCS)

codec_vectors: codec_vectors.c $(APP)/platform/bitpack.c \
               $(APP)/platform/sys_status.c $(APP)/buddy1_comms/tlm_codec.c
	$(CC) $(CFLAGS) -o $@ $^

clean:
	rm -f robot_tests codec_vectors

.PHONY: run clean
