################################################################################
# Automatically-generated file. Do not edit!
################################################################################

C_FILES += "C:/Users/hmidenen/Desktop/Sanity Check/AN2668_FW_V3.0.0/Project/ADCOversampling-STM32L/TriangularDitherMethod/src/main.c"
OBJ_FILES += "User\main.obj"
"User\main.obj" : "C:/Users/hmidenen/Desktop/Sanity Check/AN2668_FW_V3.0.0/Project/ADCOversampling-STM32L/TriangularDitherMethod/src/main.c" "User\.main.obj.opt"
	@echo Compiling ${<F}
	@"${PRODDIR}\bin\ccarm" -f "User\.main.obj.opt"

"User\.main.obj.opt" : .refresh
	@argfile "User\.main.obj.opt" -o "User\main.obj" "C:/Users/hmidenen/Desktop/Sanity Check/AN2668_FW_V3.0.0/Project/ADCOversampling-STM32L/TriangularDitherMethod/src/main.c" -Cstm32l152vb -t -Wa-gAHLs -Wa--no-warnings -Wa--error-limit=42 -DVECT_TAB_FLASH -DUSE_STDPERIPH_DRIVER -DUSE_STM32L152_EVAL -DSTM32L1XX_MD -I..\..\..\inc -I..\..\..\..\..\..\Libraries\STM32L1xx_StdPeriph_Driver\inc -I..\..\..\..\..\..\Libraries\CMSIS\Device\ST\STM32L1xx\Include -I..\..\..\..\..\..\Libraries\CMSIS\Include --language=-gcc,-volatile,+strings -O3 --tradeoff=4 --compact-max-size=200 -g -Wc-w557 -Wc-w529 -Wc-w588 --source -c --dep-file="User\.main.obj.d" -Wc--make-target="User\main.obj"
DEPENDENCY_FILES += "User\.main.obj.d"

C_FILES += "C:/Users/hmidenen/Desktop/Sanity Check/AN2668_FW_V3.0.0/Project/ADCOversampling-STM32L/TriangularDitherMethod/src/oversampling.c"
OBJ_FILES += "User\oversampling.obj"
"User\oversampling.obj" : "C:/Users/hmidenen/Desktop/Sanity Check/AN2668_FW_V3.0.0/Project/ADCOversampling-STM32L/TriangularDitherMethod/src/oversampling.c" "User\.oversampling.obj.opt"
	@echo Compiling ${<F}
	@"${PRODDIR}\bin\ccarm" -f "User\.oversampling.obj.opt"

"User\.oversampling.obj.opt" : .refresh
	@argfile "User\.oversampling.obj.opt" -o "User\oversampling.obj" "C:/Users/hmidenen/Desktop/Sanity Check/AN2668_FW_V3.0.0/Project/ADCOversampling-STM32L/TriangularDitherMethod/src/oversampling.c" -Cstm32l152vb -t -Wa-gAHLs -Wa--no-warnings -Wa--error-limit=42 -DVECT_TAB_FLASH -DUSE_STDPERIPH_DRIVER -DUSE_STM32L152_EVAL -DSTM32L1XX_MD -I..\..\..\inc -I..\..\..\..\..\..\Libraries\STM32L1xx_StdPeriph_Driver\inc -I..\..\..\..\..\..\Libraries\CMSIS\Device\ST\STM32L1xx\Include -I..\..\..\..\..\..\Libraries\CMSIS\Include --language=-gcc,-volatile,+strings -O3 --tradeoff=4 --compact-max-size=200 -g -Wc-w557 -Wc-w529 -Wc-w588 --source -c --dep-file="User\.oversampling.obj.d" -Wc--make-target="User\oversampling.obj"
DEPENDENCY_FILES += "User\.oversampling.obj.d"

C_FILES += "C:/Users/hmidenen/Desktop/Sanity Check/AN2668_FW_V3.0.0/Project/ADCOversampling-STM32L/TriangularDitherMethod/src/stm32l1xx_it.c"
OBJ_FILES += "User\stm32l1xx_it.obj"
"User\stm32l1xx_it.obj" : "C:/Users/hmidenen/Desktop/Sanity Check/AN2668_FW_V3.0.0/Project/ADCOversampling-STM32L/TriangularDitherMethod/src/stm32l1xx_it.c" "User\.stm32l1xx_it.obj.opt"
	@echo Compiling ${<F}
	@"${PRODDIR}\bin\ccarm" -f "User\.stm32l1xx_it.obj.opt"

"User\.stm32l1xx_it.obj.opt" : .refresh
	@argfile "User\.stm32l1xx_it.obj.opt" -o "User\stm32l1xx_it.obj" "C:/Users/hmidenen/Desktop/Sanity Check/AN2668_FW_V3.0.0/Project/ADCOversampling-STM32L/TriangularDitherMethod/src/stm32l1xx_it.c" -Cstm32l152vb -t -Wa-gAHLs -Wa--no-warnings -Wa--error-limit=42 -DVECT_TAB_FLASH -DUSE_STDPERIPH_DRIVER -DUSE_STM32L152_EVAL -DSTM32L1XX_MD -I..\..\..\inc -I..\..\..\..\..\..\Libraries\STM32L1xx_StdPeriph_Driver\inc -I..\..\..\..\..\..\Libraries\CMSIS\Device\ST\STM32L1xx\Include -I..\..\..\..\..\..\Libraries\CMSIS\Include --language=-gcc,-volatile,+strings -O3 --tradeoff=4 --compact-max-size=200 -g -Wc-w557 -Wc-w529 -Wc-w588 --source -c --dep-file="User\.stm32l1xx_it.obj.d" -Wc--make-target="User\stm32l1xx_it.obj"
DEPENDENCY_FILES += "User\.stm32l1xx_it.obj.d"


GENERATED_FILES += "User\main.obj" "User\.main.obj.opt" "User\.main.obj.d" "User\main.src" "User\main.lst" "User\oversampling.obj" "User\.oversampling.obj.opt" "User\.oversampling.obj.d" "User\oversampling.src" "User\oversampling.lst" "User\stm32l1xx_it.obj" "User\.stm32l1xx_it.obj.opt" "User\.stm32l1xx_it.obj.d" "User\stm32l1xx_it.src" "User\stm32l1xx_it.lst"
