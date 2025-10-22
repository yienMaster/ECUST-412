################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libraries/infineon_libraries/Infra/Ssw/TC36A/Tricore/Ifx_Ssw_Infra.c \
../libraries/infineon_libraries/Infra/Ssw/TC36A/Tricore/Ifx_Ssw_Tc0.c \
../libraries/infineon_libraries/Infra/Ssw/TC36A/Tricore/Ifx_Ssw_Tc1.c 

COMPILED_SRCS += \
./libraries/infineon_libraries/Infra/Ssw/TC36A/Tricore/Ifx_Ssw_Infra.src \
./libraries/infineon_libraries/Infra/Ssw/TC36A/Tricore/Ifx_Ssw_Tc0.src \
./libraries/infineon_libraries/Infra/Ssw/TC36A/Tricore/Ifx_Ssw_Tc1.src 

C_DEPS += \
./libraries/infineon_libraries/Infra/Ssw/TC36A/Tricore/Ifx_Ssw_Infra.d \
./libraries/infineon_libraries/Infra/Ssw/TC36A/Tricore/Ifx_Ssw_Tc0.d \
./libraries/infineon_libraries/Infra/Ssw/TC36A/Tricore/Ifx_Ssw_Tc1.d 

OBJS += \
./libraries/infineon_libraries/Infra/Ssw/TC36A/Tricore/Ifx_Ssw_Infra.o \
./libraries/infineon_libraries/Infra/Ssw/TC36A/Tricore/Ifx_Ssw_Tc0.o \
./libraries/infineon_libraries/Infra/Ssw/TC36A/Tricore/Ifx_Ssw_Tc1.o 


# Each subdirectory must supply rules for building sources it contributes
libraries/infineon_libraries/Infra/Ssw/TC36A/Tricore/%.src: ../libraries/infineon_libraries/Infra/Ssw/TC36A/Tricore/%.c libraries/infineon_libraries/Infra/Ssw/TC36A/Tricore/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -cs --dep-file="$(basename $@).d" --misrac-version=2004 -D__CPU__=tc36x "-fE:/ADSWorkspace/Car_rear/Car_Rear/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc36x -Y0 -N0 -Z0 -o "$@" "$<" && \
	if [ -f "$(basename $@).d" ]; then sed.exe -r  -e 's/\b(.+\.o)\b/libraries\/infineon_libraries\/Infra\/Ssw\/TC36A\/Tricore\/\1/g' -e 's/\\/\//g' -e 's/\/\//\//g' -e 's/"//g' -e 's/([a-zA-Z]:\/)/\L\1/g' -e 's/\d32:/@TARGET_DELIMITER@/g; s/\\\d32/@ESCAPED_SPACE@/g; s/\d32/\\\d32/g; s/@ESCAPED_SPACE@/\\\d32/g; s/@TARGET_DELIMITER@/\d32:/g' "$(basename $@).d" > "$(basename $@).d_sed" && cp "$(basename $@).d_sed" "$(basename $@).d" && rm -f "$(basename $@).d_sed" 2>/dev/null; else echo 'No dependency file to process';fi
	@echo 'Finished building: $<'
	@echo ' '

libraries/infineon_libraries/Infra/Ssw/TC36A/Tricore/%.o: ./libraries/infineon_libraries/Infra/Ssw/TC36A/Tricore/%.src libraries/infineon_libraries/Infra/Ssw/TC36A/Tricore/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-libraries-2f-infineon_libraries-2f-Infra-2f-Ssw-2f-TC36A-2f-Tricore

clean-libraries-2f-infineon_libraries-2f-Infra-2f-Ssw-2f-TC36A-2f-Tricore:
	-$(RM) ./libraries/infineon_libraries/Infra/Ssw/TC36A/Tricore/Ifx_Ssw_Infra.d ./libraries/infineon_libraries/Infra/Ssw/TC36A/Tricore/Ifx_Ssw_Infra.o ./libraries/infineon_libraries/Infra/Ssw/TC36A/Tricore/Ifx_Ssw_Infra.src ./libraries/infineon_libraries/Infra/Ssw/TC36A/Tricore/Ifx_Ssw_Tc0.d ./libraries/infineon_libraries/Infra/Ssw/TC36A/Tricore/Ifx_Ssw_Tc0.o ./libraries/infineon_libraries/Infra/Ssw/TC36A/Tricore/Ifx_Ssw_Tc0.src ./libraries/infineon_libraries/Infra/Ssw/TC36A/Tricore/Ifx_Ssw_Tc1.d ./libraries/infineon_libraries/Infra/Ssw/TC36A/Tricore/Ifx_Ssw_Tc1.o ./libraries/infineon_libraries/Infra/Ssw/TC36A/Tricore/Ifx_Ssw_Tc1.src

.PHONY: clean-libraries-2f-infineon_libraries-2f-Infra-2f-Ssw-2f-TC36A-2f-Tricore

