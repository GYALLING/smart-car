################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/balance.c \
../code/constrain.c \
../code/get_angle.c \
../code/image.c \
../code/motor.c \
../code/small_driver_uart_control.c 

COMPILED_SRCS += \
code/balance.src \
code/constrain.src \
code/get_angle.src \
code/image.src \
code/motor.src \
code/small_driver_uart_control.src 

C_DEPS += \
code/balance.d \
code/constrain.d \
code/get_angle.d \
code/image.d \
code/motor.d \
code/small_driver_uart_control.d 

OBJS += \
code/balance.o \
code/constrain.o \
code/get_angle.o \
code/image.o \
code/motor.o \
code/small_driver_uart_control.o 


# Each subdirectory must supply rules for building sources it contributes
code/balance.src: ../code/balance.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/33278/AURIX-v1.10.2-workspace/Seekfree_TC377_Opensource_Library£¨11£©/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/balance.o: code/balance.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/constrain.src: ../code/constrain.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/33278/AURIX-v1.10.2-workspace/Seekfree_TC377_Opensource_Library£¨11£©/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/constrain.o: code/constrain.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/get_angle.src: ../code/get_angle.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/33278/AURIX-v1.10.2-workspace/Seekfree_TC377_Opensource_Library£¨11£©/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/get_angle.o: code/get_angle.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/image.src: ../code/image.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/33278/AURIX-v1.10.2-workspace/Seekfree_TC377_Opensource_Library£¨11£©/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/image.o: code/image.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/motor.src: ../code/motor.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/33278/AURIX-v1.10.2-workspace/Seekfree_TC377_Opensource_Library£¨11£©/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/motor.o: code/motor.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/small_driver_uart_control.src: ../code/small_driver_uart_control.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/33278/AURIX-v1.10.2-workspace/Seekfree_TC377_Opensource_Library£¨11£©/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/small_driver_uart_control.o: code/small_driver_uart_control.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code

clean-code:
	-$(RM) code/balance.d code/balance.o code/balance.src code/constrain.d code/constrain.o code/constrain.src code/get_angle.d code/get_angle.o code/get_angle.src code/image.d code/image.o code/image.src code/motor.d code/motor.o code/motor.src code/small_driver_uart_control.d code/small_driver_uart_control.o code/small_driver_uart_control.src

.PHONY: clean-code

