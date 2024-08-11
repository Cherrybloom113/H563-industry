SET PATH=C:\Keil_v5\ARM\ARMCLANG\Bin;C:\Program Files (x86)\Common Files\Oracle\Java\java8path;C:\Program Files (x86)\Common Files\Oracle\Java\javapath;C:\Program Files\Python310\Scripts\;C:\Program Files\Python310\;C:\Program Files (x86)\VMware\VMware Workstation\bin\;C:\Program Files (x86)\Common Files\Intel\Shared Libraries\redist\intel64\compiler;C:\WINDOWS\system32;C:\WINDOWS;C:\WINDOWS\System32\Wbem;C:\WINDOWS\System32\WindowsPowerShell\v1.0\;C:\WINDOWS\System32\OpenSSH\;C:\Program Files (x86)\NVIDIA Corporation\PhysX\Common;C:\WINDOWS\system32;C:\WINDOWS;C:\WINDOWS\System32\Wbem;C:\WINDOWS\System32\WindowsPowerShell\v1.0\;C:\WINDOWS\System32\OpenSSH\;C:\Program Files\Polyspace\R2020b\runtime\win64;C:\Program Files\Polyspace\R2020b\bin;C:\Program Files\Polyspace\R2020b\polyspace\bin;C:\Program Files\MySQL\MySQL Server 8.0\bin;C:\Program Files\dotnet\;C:\Program Files\CodeBlocks\MinGW\bin;C:\Program Files (x86)\pcsuite\;C:\Program Files\NVIDIA Corporation\NVIDIA App\NvDLISR;C:\Program Files (x86)\IVI Foundation\VISA\WinNT\Bin\;C:\Program Files\IVI Foundation\VISA\Win64\Bin\;C:\Program Files (x86)\National Instruments\Shared\LabVIEW CLI;C:\Program Files (x86)\IVI Foundation\IVI\Bin\;C:\Program Files\IVI Foundation\IVI\Bin\;C:\Program Files (x86)\IVI Foundation\VISA\WinNT\Bin;D:\PyCharm 2024.1.4\bin;C:\Program Files\Git\cmd;C:\Program Files\CodeBlocks\MinGW\bin;C:\Users\liaos\AppData\Local\Microsoft\WindowsApps;E:\Microsoft VS Code\bin;C:\Users\liaos\.dotnet\tools;C:\intelFPGA\18.0\modelsim_ase\win32aloem;D:\PyCharm Community Edition 2024.1.4\bin;C:\Users\liaos\AppData\Local\GitHubDesktop\bin
SET CPU_TYPE=STM32H563RIVx
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=boot_demo
SET CPU_CLOCK=0x017D7840
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"boot_demo\startup_stm32h563xx._ac"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"boot_demo\main.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"boot_demo\gpio.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"boot_demo\memorymap.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"boot_demo\stm32h5xx_it.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"boot_demo\stm32h5xx_hal_msp.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"boot_demo\stm32h5xx_hal_cortex.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"boot_demo\stm32h5xx_hal_rcc.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"boot_demo\stm32h5xx_hal_rcc_ex.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"boot_demo\stm32h5xx_hal_flash.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"boot_demo\stm32h5xx_hal_flash_ex.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"boot_demo\stm32h5xx_hal_gpio.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"boot_demo\stm32h5xx_hal_dma.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"boot_demo\stm32h5xx_hal_dma_ex.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"boot_demo\stm32h5xx_hal_pwr.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"boot_demo\stm32h5xx_hal_pwr_ex.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"boot_demo\stm32h5xx_hal.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"boot_demo\stm32h5xx_hal_exti.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"boot_demo\stm32h5xx_hal_tim.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"boot_demo\stm32h5xx_hal_tim_ex.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"boot_demo\system_stm32h5xx.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmLink" --Via "boot_demo\boot_demo.lnp"
"C:\Keil_v5\ARM\ARMCLANG\Bin\fromelf.exe" "boot_demo\boot_demo.axf" --i32combined --output "boot_demo\boot_demo.hex"
fromelf --bin --output h5_app.bin boot_demo\boot_demo.axf
