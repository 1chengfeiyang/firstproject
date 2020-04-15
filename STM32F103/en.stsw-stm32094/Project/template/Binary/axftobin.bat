
if exist C:\Keil\ARM\BIN40\fromelf.exe (
if exist .\..\MDK-ARM\STM32100B-EVAL\Project.axf (C:\Keil\ARM\BIN40\fromelf.exe ".\..\MDK-ARM\STM32100B-EVAL\Project.axf" --bin --output ".\..\MDK-ARM\STM32100B-EVAL\Project.bin")
if exist .\..\MDK-ARM\STM3210C-EVAL\Project.axf (C:\Keil\ARM\BIN40\fromelf.exe ".\..\MDK-ARM\STM3210C-EVAL\Project.axf" --bin --output ".\..\MDK-ARM\STM3210C-EVAL\Project.bin")  
if exist .\..\MDK-ARM\STM3210B-EVAL\Project.axf (C:\Keil\ARM\BIN40\fromelf.exe ".\..\MDK-ARM\STM3210B-EVAL\Project.axf" --bin --output ".\..\MDK-ARM\STM3210B-EVAL\Project.bin")
if exist .\..\MDK-ARM\STM3210E-EVAL\Project.axf (C:\Keil\ARM\BIN40\fromelf.exe ".\..\MDK-ARM\STM3210E-EVAL\Project.axf" --bin --output ".\..\MDK-ARM\STM3210E-EVAL\Project.bin")
if exist .\..\MDK-ARM\STM3210E-EVAL_XL\Project.axf (C:\Keil\ARM\BIN40\fromelf.exe ".\..\MDK-ARM\STM3210E-EVAL_XL\Project.axf" --bin --output ".\..\MDK-ARM\STM3210E-EVAL_XL\Project.bin")
 )

pause

