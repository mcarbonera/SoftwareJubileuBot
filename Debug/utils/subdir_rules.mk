################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
utils/%.obj: ../utils/%.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs910/ccs/tools/compiler/ti-cgt-arm_18.12.2.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/Users/Luiz/workspace_v9/JubileuBot" --include_path="C:/Users/Luiz/workspace_v9/JubileuBot/utils" --include_path="C:/Users/Luiz/workspace_v9/JubileuBot" --include_path="C:/Users/Luiz/workspace_v9/JubileuBot/driverlib" --include_path="C:/Users/Luiz/workspace_v9/JubileuBot/FreeRTOS/include" --include_path="C:/Users/Luiz/workspace_v9/JubileuBot/FreeRTOS/portable/CCS/ARM_CM4F" --include_path="C:/ti/ccs910/ccs/tools/compiler/ti-cgt-arm_18.12.2.LTS/include" --define=ccs="ccs" --define=PART_TM4C1294NCPDT -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="utils/$(basename $(<F)).d_raw" --obj_directory="utils" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


