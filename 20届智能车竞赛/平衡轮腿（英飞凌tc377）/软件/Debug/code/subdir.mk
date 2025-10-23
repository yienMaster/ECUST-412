################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/FiveBarLinkageData\ .c \
../code/adc.c \
../code/control.c \
../code/engine.c \
../code/image.c \
../code/jump_control.c \
../code/kalman.c \
../code/kalman_rm.c \
../code/leg_adaptive.c \
../code/pid.c \
../code/small_driver_uart_control.c \
../code/tft.c \
../code/wifi.c \
../code/zf_device_lora3a22.c 

COMPILED_SRCS += \
./code/FiveBarLinkageData\ .src \
./code/adc.src \
./code/control.src \
./code/engine.src \
./code/image.src \
./code/jump_control.src \
./code/kalman.src \
./code/kalman_rm.src \
./code/leg_adaptive.src \
./code/pid.src \
./code/small_driver_uart_control.src \
./code/tft.src \
./code/wifi.src \
./code/zf_device_lora3a22.src 

C_DEPS += \
./code/FiveBarLinkageData\ .d \
./code/adc.d \
./code/control.d \
./code/engine.d \
./code/image.d \
./code/jump_control.d \
./code/kalman.d \
./code/kalman_rm.d \
./code/leg_adaptive.d \
./code/pid.d \
./code/small_driver_uart_control.d \
./code/tft.d \
./code/wifi.d \
./code/zf_device_lora3a22.d 

OBJS += \
./code/FiveBarLinkageData\ .o \
./code/adc.o \
./code/control.o \
./code/engine.o \
./code/image.o \
./code/jump_control.o \
./code/kalman.o \
./code/kalman_rm.o \
./code/leg_adaptive.o \
./code/pid.o \
./code/small_driver_uart_control.o \
./code/tft.o \
./code/wifi.o \
./code/zf_device_lora3a22.o 


# Each subdirectory must supply rules for building sources it contributes
code/FiveBarLinkageData\ .src: ../code/FiveBarLinkageData\ .c code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -cs --dep-file="$(basename $@).d" --misrac-version=2004 -D__CPU__=tc37x --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<" && \
	if [ -f "$(basename $@).d" ]; then sed.exe -r  -e 's/\b(.+\.o)\b/code\/\1/g' -e 's/\\/\//g' -e 's/\/\//\//g' -e 's/"//g' -e 's/([a-zA-Z]:\/)/\L\1/g' -e 's/\d32:/@TARGET_DELIMITER@/g; s/\\\d32/@ESCAPED_SPACE@/g; s/\d32/\\\d32/g; s/@ESCAPED_SPACE@/\\\d32/g; s/@TARGET_DELIMITER@/\d32:/g' "$(basename $@).d" > "$(basename $@).d_sed" && cp "$(basename $@).d_sed" "$(basename $@).d" && rm -f "$(basename $@).d_sed" 2>/dev/null; else echo 'No dependency file to process';fi
	@echo 'Finished building: $<'
	@echo ' '

code/FiveBarLinkageData\ .o: ./code/FiveBarLinkageData\ .src code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

code/%.src: ../code/%.c code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -cs --dep-file="$(basename $@).d" --misrac-version=2004 -D__CPU__=tc37x "-fE:/yingfenlin/project/20th_bluecar/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<" && \
	if [ -f "$(basename $@).d" ]; then sed.exe -r  -e 's/\b(.+\.o)\b/code\/\1/g' -e 's/\\/\//g' -e 's/\/\//\//g' -e 's/"//g' -e 's/([a-zA-Z]:\/)/\L\1/g' -e 's/\d32:/@TARGET_DELIMITER@/g; s/\\\d32/@ESCAPED_SPACE@/g; s/\d32/\\\d32/g; s/@ESCAPED_SPACE@/\\\d32/g; s/@TARGET_DELIMITER@/\d32:/g' "$(basename $@).d" > "$(basename $@).d_sed" && cp "$(basename $@).d_sed" "$(basename $@).d" && rm -f "$(basename $@).d_sed" 2>/dev/null; else echo 'No dependency file to process';fi
	@echo 'Finished building: $<'
	@echo ' '

code/%.o: ./code/%.src code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-code

clean-code:
	-$(RM) ./code/FiveBarLinkageData\ .d ./code/FiveBarLinkageData\ .o ./code/FiveBarLinkageData\ .src ./code/adc.d ./code/adc.o ./code/adc.src ./code/control.d ./code/control.o ./code/control.src ./code/engine.d ./code/engine.o ./code/engine.src ./code/image.d ./code/image.o ./code/image.src ./code/jump_control.d ./code/jump_control.o ./code/jump_control.src ./code/kalman.d ./code/kalman.o ./code/kalman.src ./code/kalman_rm.d ./code/kalman_rm.o ./code/kalman_rm.src ./code/leg_adaptive.d ./code/leg_adaptive.o ./code/leg_adaptive.src ./code/pid.d ./code/pid.o ./code/pid.src ./code/small_driver_uart_control.d ./code/small_driver_uart_control.o ./code/small_driver_uart_control.src ./code/tft.d ./code/tft.o ./code/tft.src ./code/wifi.d ./code/wifi.o ./code/wifi.src ./code/zf_device_lora3a22.d ./code/zf_device_lora3a22.o ./code/zf_device_lora3a22.src

.PHONY: clean-code

