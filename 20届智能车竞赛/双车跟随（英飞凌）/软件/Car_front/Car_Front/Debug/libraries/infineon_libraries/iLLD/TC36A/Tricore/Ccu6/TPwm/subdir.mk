################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libraries/infineon_libraries/iLLD/TC36A/Tricore/Ccu6/TPwm/IfxCcu6_TPwm.c 

COMPILED_SRCS += \
./libraries/infineon_libraries/iLLD/TC36A/Tricore/Ccu6/TPwm/IfxCcu6_TPwm.src 

C_DEPS += \
./libraries/infineon_libraries/iLLD/TC36A/Tricore/Ccu6/TPwm/IfxCcu6_TPwm.d 

OBJS += \
./libraries/infineon_libraries/iLLD/TC36A/Tricore/Ccu6/TPwm/IfxCcu6_TPwm.o 


# Each subdirectory must supply rules for building sources it contributes
libraries/infineon_libraries/iLLD/TC36A/Tricore/Ccu6/TPwm/%.src: ../libraries/infineon_libraries/iLLD/TC36A/Tricore/Ccu6/TPwm/%.c libraries/infineon_libraries/iLLD/TC36A/Tricore/Ccu6/TPwm/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -cs --dep-file="$(basename $@).d" --misrac-version=2004 -D__CPU__=tc36x "-fE:/ADSWorkspace/Car_front/Car_Front/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc36x -Y0 -N0 -Z0 -o "$@" "$<" && \
	if [ -f "$(basename $@).d" ]; then sed.exe -r  -e 's/\b(.+\.o)\b/libraries\/infineon_libraries\/iLLD\/TC36A\/Tricore\/Ccu6\/TPwm\/\1/g' -e 's/\\/\//g' -e 's/\/\//\//g' -e 's/"//g' -e 's/([a-zA-Z]:\/)/\L\1/g' -e 's/\d32:/@TARGET_DELIMITER@/g; s/\\\d32/@ESCAPED_SPACE@/g; s/\d32/\\\d32/g; s/@ESCAPED_SPACE@/\\\d32/g; s/@TARGET_DELIMITER@/\d32:/g' "$(basename $@).d" > "$(basename $@).d_sed" && cp "$(basename $@).d_sed" "$(basename $@).d" && rm -f "$(basename $@).d_sed" 2>/dev/null; else echo 'No dependency file to process';fi
	@echo 'Finished building: $<'
	@echo ' '

libraries/infineon_libraries/iLLD/TC36A/Tricore/Ccu6/TPwm/%.o: ./libraries/infineon_libraries/iLLD/TC36A/Tricore/Ccu6/TPwm/%.src libraries/infineon_libraries/iLLD/TC36A/Tricore/Ccu6/TPwm/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-libraries-2f-infineon_libraries-2f-iLLD-2f-TC36A-2f-Tricore-2f-Ccu6-2f-TPwm

clean-libraries-2f-infineon_libraries-2f-iLLD-2f-TC36A-2f-Tricore-2f-Ccu6-2f-TPwm:
	-$(RM) ./libraries/infineon_libraries/iLLD/TC36A/Tricore/Ccu6/TPwm/IfxCcu6_TPwm.d ./libraries/infineon_libraries/iLLD/TC36A/Tricore/Ccu6/TPwm/IfxCcu6_TPwm.o ./libraries/infineon_libraries/iLLD/TC36A/Tricore/Ccu6/TPwm/IfxCcu6_TPwm.src

.PHONY: clean-libraries-2f-infineon_libraries-2f-iLLD-2f-TC36A-2f-Tricore-2f-Ccu6-2f-TPwm

