################################################################################
# Automatically-generated file. Do not edit!
################################################################################

PROJ 	= STM32L152-EVAL
PRODDIR = C:\Program Files\TASKING\carm v4.4r1

-include "..\makefile.init"

all :

-include "subdir.mk"
-include "CMSIS\subdir.mk"
-include "StdPeriph_Driver\subdir.mk"
-include "User\subdir.mk"
-include "..\makefile.defs"

EXT_LIBRARIES =

all : "STM32L152-EVAL.abs"

"STM32L152-EVAL.abs" : $(OBJ_FILES) $(EXT_LIBRARIES) ".STM32L152-EVAL.abs.opt"
	@echo Linking to ${@F}
	@"${PRODDIR}\bin\ccarm" -f ".STM32L152-EVAL.abs.opt"

".STM32L152-EVAL.abs.opt" : .refresh
	@argfile ".STM32L152-EVAL.abs.opt" -o "STM32L152-EVAL.abs" $(OBJ_FILES) $(EXT_LIBRARIES) -Cstm32l152vb -t -Wl-D__DEVICE_LSL_FILE=stm32l1xx.lsl -Wl-D__FLASH_SIZE=128k -Wl-D__SRAM_SIZE=16k -Wl-D__EEPROM_SIZE=4k "../TASKING/stm32l1xx_md.lsl" -Wl-OtZxycL --no-map-file -Wl--map-file="${PROJ}.mapxml":XML -Wl-mcrfiklSmNOduQ -Wl--error-limit=42 -g -Wl--dep-file=".STM32L152-EVAL.abs.d" -Wl--make-target="STM32L152-EVAL.abs"
DEPENDENCY_FILES += ".STM32L152-EVAL.abs.d"
-include $(DEPENDENCY_FILES)

$(EXT_LIBRARIES) : .dummy

GENERATED_FILES += "STM32L152-EVAL.abs" "STM32L152-EVAL*.sre" "STM32L152-EVAL*.hex" "STM32L152-EVAL.map*" "STM32L152-EVAL.mdf" ".STM32L152-EVAL.abs.opt" ".STM32L152-EVAL.abs.d"
-include "..\makefile.targets"

clean : ".clean.opt"
	@echo Cleaning
	@"${PRODDIR}\bin\rm" -f -m ".clean.opt"

".clean.opt" : .refresh
	@argfile ".clean.opt" $(GENERATED_FILES) $(OPTIONAL_FILES)

GENERATED_FILES += ".clean.opt"
OPTIONAL_FILES +=  "*.pre" "*.lst"


.PHONY : .dummy .refresh
