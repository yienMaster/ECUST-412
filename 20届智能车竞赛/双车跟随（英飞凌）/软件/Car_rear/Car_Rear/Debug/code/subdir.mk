################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/control.c \
../code/device.c \
../code/eeprom.c \
../code/image.c \
../code/init.c \
../code/mathfunc.c \
../code/pid.c \
../code/upper_computer.c 

COMPILED_SRCS += \
./code/control.src \
./code/device.src \
./code/eeprom.src \
./code/image.src \
./code/init.src \
./code/mathfunc.src \
./code/pid.src \
./code/upper_computer.src 

C_DEPS += \
./code/control.d \
./code/device.d \
./code/eeprom.d \
./code/image.d \
./code/init.d \
./code/mathfunc.d \
./code/pid.d \
./code/upper_computer.d 

OBJS += \
./code/control.o \
./code/device.o \
./code/eeprom.o \
./code/image.o \
./code/init.o \
./code/mathfunc.o \
./code/pid.o \
./code/upper_computer.o 


# Each subdirectory must supply rules for building sources it contributes
code/%.src: ../code/%.c code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -cs --dep-file="$(basename $@).d" --misrac-version=2004 -D__CPU__=tc36x "-fE:/ADSWorkspace/Car_rear/Car_Rear/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc36x -Y0 -N0 -Z0 -o "$@" "$<" && \
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
	-$(RM) ./code/control.d ./code/control.o ./code/control.src ./code/device.d ./code/device.o ./code/device.src ./code/eeprom.d ./code/eeprom.o ./code/eeprom.src ./code/image.d ./code/image.o ./code/image.src ./code/init.d ./code/init.o ./code/init.src ./code/mathfunc.d ./code/mathfunc.o ./code/mathfunc.src ./code/pid.d ./code/pid.o ./code/pid.src ./code/upper_computer.d ./code/upper_computer.o ./code/upper_computer.src

.PHONY: clean-code

