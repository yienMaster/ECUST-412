################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libraries/zf_device/zf_device_camera.c \
../libraries/zf_device/zf_device_imu660ra.c \
../libraries/zf_device/zf_device_ips114.c \
../libraries/zf_device/zf_device_key.c \
../libraries/zf_device/zf_device_mt9v03x.c \
../libraries/zf_device/zf_device_tft180.c \
../libraries/zf_device/zf_device_type.c \
../libraries/zf_device/zf_device_uart_receiver.c \
../libraries/zf_device/zf_device_virtual_oscilloscope.c \
../libraries/zf_device/zf_device_wifi_spi.c 

COMPILED_SRCS += \
./libraries/zf_device/zf_device_camera.src \
./libraries/zf_device/zf_device_imu660ra.src \
./libraries/zf_device/zf_device_ips114.src \
./libraries/zf_device/zf_device_key.src \
./libraries/zf_device/zf_device_mt9v03x.src \
./libraries/zf_device/zf_device_tft180.src \
./libraries/zf_device/zf_device_type.src \
./libraries/zf_device/zf_device_uart_receiver.src \
./libraries/zf_device/zf_device_virtual_oscilloscope.src \
./libraries/zf_device/zf_device_wifi_spi.src 

C_DEPS += \
./libraries/zf_device/zf_device_camera.d \
./libraries/zf_device/zf_device_imu660ra.d \
./libraries/zf_device/zf_device_ips114.d \
./libraries/zf_device/zf_device_key.d \
./libraries/zf_device/zf_device_mt9v03x.d \
./libraries/zf_device/zf_device_tft180.d \
./libraries/zf_device/zf_device_type.d \
./libraries/zf_device/zf_device_uart_receiver.d \
./libraries/zf_device/zf_device_virtual_oscilloscope.d \
./libraries/zf_device/zf_device_wifi_spi.d 

OBJS += \
./libraries/zf_device/zf_device_camera.o \
./libraries/zf_device/zf_device_imu660ra.o \
./libraries/zf_device/zf_device_ips114.o \
./libraries/zf_device/zf_device_key.o \
./libraries/zf_device/zf_device_mt9v03x.o \
./libraries/zf_device/zf_device_tft180.o \
./libraries/zf_device/zf_device_type.o \
./libraries/zf_device/zf_device_uart_receiver.o \
./libraries/zf_device/zf_device_virtual_oscilloscope.o \
./libraries/zf_device/zf_device_wifi_spi.o 


# Each subdirectory must supply rules for building sources it contributes
libraries/zf_device/%.src: ../libraries/zf_device/%.c libraries/zf_device/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -cs --dep-file="$(basename $@).d" --misrac-version=2004 -D__CPU__=tc36x "-fE:/ADSWorkspace/Car_rear/Car_Rear/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc36x -Y0 -N0 -Z0 -o "$@" "$<" && \
	if [ -f "$(basename $@).d" ]; then sed.exe -r  -e 's/\b(.+\.o)\b/libraries\/zf_device\/\1/g' -e 's/\\/\//g' -e 's/\/\//\//g' -e 's/"//g' -e 's/([a-zA-Z]:\/)/\L\1/g' -e 's/\d32:/@TARGET_DELIMITER@/g; s/\\\d32/@ESCAPED_SPACE@/g; s/\d32/\\\d32/g; s/@ESCAPED_SPACE@/\\\d32/g; s/@TARGET_DELIMITER@/\d32:/g' "$(basename $@).d" > "$(basename $@).d_sed" && cp "$(basename $@).d_sed" "$(basename $@).d" && rm -f "$(basename $@).d_sed" 2>/dev/null; else echo 'No dependency file to process';fi
	@echo 'Finished building: $<'
	@echo ' '

libraries/zf_device/%.o: ./libraries/zf_device/%.src libraries/zf_device/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-libraries-2f-zf_device

clean-libraries-2f-zf_device:
	-$(RM) ./libraries/zf_device/zf_device_camera.d ./libraries/zf_device/zf_device_camera.o ./libraries/zf_device/zf_device_camera.src ./libraries/zf_device/zf_device_imu660ra.d ./libraries/zf_device/zf_device_imu660ra.o ./libraries/zf_device/zf_device_imu660ra.src ./libraries/zf_device/zf_device_ips114.d ./libraries/zf_device/zf_device_ips114.o ./libraries/zf_device/zf_device_ips114.src ./libraries/zf_device/zf_device_key.d ./libraries/zf_device/zf_device_key.o ./libraries/zf_device/zf_device_key.src ./libraries/zf_device/zf_device_mt9v03x.d ./libraries/zf_device/zf_device_mt9v03x.o ./libraries/zf_device/zf_device_mt9v03x.src ./libraries/zf_device/zf_device_tft180.d ./libraries/zf_device/zf_device_tft180.o ./libraries/zf_device/zf_device_tft180.src ./libraries/zf_device/zf_device_type.d ./libraries/zf_device/zf_device_type.o ./libraries/zf_device/zf_device_type.src ./libraries/zf_device/zf_device_uart_receiver.d ./libraries/zf_device/zf_device_uart_receiver.o ./libraries/zf_device/zf_device_uart_receiver.src ./libraries/zf_device/zf_device_virtual_oscilloscope.d ./libraries/zf_device/zf_device_virtual_oscilloscope.o ./libraries/zf_device/zf_device_virtual_oscilloscope.src ./libraries/zf_device/zf_device_wifi_spi.d ./libraries/zf_device/zf_device_wifi_spi.o ./libraries/zf_device/zf_device_wifi_spi.src

.PHONY: clean-libraries-2f-zf_device

