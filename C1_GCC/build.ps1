# C1 motion control - GCC build script (no make required)
#
# Usage:  powershell -ExecutionPolicy Bypass -File build.ps1
# Output: build\C1.elf  build\C1.hex  build\C1.bin

param(
    [string]$GccPath = 'D:\xpack-arm-none-eabi-gcc-13.3.1-1.1\bin'
)

$ErrorActionPreference = 'Stop'

$root  = Split-Path -Parent $MyInvocation.MyCommand.Path
$keil  = Join-Path (Split-Path -Parent $root) 'C1_Keil'
$build = Join-Path $root 'build'

$cc   = Join-Path $GccPath 'arm-none-eabi-gcc.exe'
$objc = Join-Path $GccPath 'arm-none-eabi-objcopy.exe'
$size = Join-Path $GccPath 'arm-none-eabi-size.exe'
if (-not (Test-Path $cc)) { throw "arm-none-eabi-gcc not found: $cc (use -GccPath)" }

New-Item -ItemType Directory -Force -Path $build | Out-Null

$includes = @(
    "$root\cmsis",
    "$keil\Core\Inc",
    "$keil\Drivers\STM32F4xx_HAL_Driver\inc",
    "$keil\Drivers\CMSIS",
    "$keil\Middlewares\FreeRTOS\include",
    "$keil\Middlewares\FreeRTOS\portable\GCC\ARM_CM4F",
    "$keil\Middlewares\FreeRTOS"
) | ForEach-Object { "-I$_" }

$sources = @(
    "$keil\Core\Src\main.c",
    "$keil\Core\Src\stm32f4xx_it.c",
    "$keil\Core\Src\stm32f4xx_hal_msp.c",
    "$keil\Core\Src\protocol.c",
    "$keil\Core\Src\motor.c",
    "$keil\Core\Src\encoder.c",
    "$keil\Core\Src\app_tasks.c",
    "$keil\Drivers\CMSIS\system_stm32f4xx.c",
    "$keil\Middlewares\FreeRTOS\tasks.c",
    "$keil\Middlewares\FreeRTOS\queue.c",
    "$keil\Middlewares\FreeRTOS\list.c",
    "$keil\Middlewares\FreeRTOS\timers.c",
    "$keil\Middlewares\FreeRTOS\portable\GCC\ARM_CM4F\port.c",
    "$keil\Middlewares\FreeRTOS\portable\MemMang\heap_4.c"
)
$hal = @('hal','hal_cortex','hal_rcc','hal_rcc_ex','hal_gpio','hal_dma','hal_dma_ex',
         'hal_tim','hal_tim_ex','hal_uart','hal_iwdg','hal_pwr','hal_pwr_ex',
         'hal_flash','hal_flash_ex','hal_flash_ramfunc','hal_exti')
foreach ($h in $hal) { $sources += "$keil\Drivers\STM32F4xx_HAL_Driver\src\stm32f4xx_$h.c" }

$cpu = @('-mcpu=cortex-m4','-mthumb','-mfpu=fpv4-sp-d16','-mfloat-abi=hard')
$cflags = $cpu + @('-DUSE_HAL_DRIVER','-DSTM32F407xx') + $includes +
          @('-std=gnu11','-O2','-g3','-ffunction-sections','-fdata-sections',
            '-Wall','-Wno-unused-parameter','-Wno-invalid-utf8','-Wno-unsafe-buffer-usage')

Write-Host ("== compiling {0} C files ==" -f $sources.Count) -ForegroundColor Cyan
$objects = @()
$failed = 0
foreach ($src in $sources) {
    $obj = Join-Path $build ([IO.Path]::GetFileNameWithoutExtension($src) + '.o')
    $objects += $obj
    $ErrorActionPreference = 'Continue'
    $out = & $cc -c @cflags $src -o $obj 2>&1
    $code = $LASTEXITCODE
    $ErrorActionPreference = 'Stop'
    if ($code -ne 0) {
        Write-Host ("FAILED: {0}" -f $src) -ForegroundColor Red
        $out | Select-Object -First 15 | ForEach-Object { Write-Host "  $_" }
        $failed++
    }
}
if ($failed -gt 0) { throw ("{0} file(s) failed to compile" -f $failed) }

$asm    = Join-Path $root 'startup_stm32f407xx_gcc.s'
$asmObj = Join-Path $build 'startup_stm32f407xx_gcc.o'
& $cc -c @cpu '-DSTM32F407xx' $asm -o $asmObj
if ($LASTEXITCODE -ne 0) { throw "startup assembly failed" }
$objects += $asmObj

Write-Host "== linking ==" -ForegroundColor Cyan
$elf  = Join-Path $build 'C1.elf'
$ld   = Join-Path $root 'STM32F407ZGTx_FLASH.ld'
$map  = Join-Path $build 'C1.map'
$ldflags = $cpu + @("-T$ld", '--specs=nano.specs', '--specs=nosys.specs',
                    "-Wl,-Map=$map,--cref", '-Wl,--gc-sections', '-Wl,--print-memory-usage')
& $cc @objects @ldflags '-lc' '-lm' '-lnosys' -o $elf
if ($LASTEXITCODE -ne 0) { throw "link failed" }

& $objc -O ihex   $elf (Join-Path $build 'C1.hex')
& $objc -O binary -S $elf (Join-Path $build 'C1.bin')

Write-Host "== result ==" -ForegroundColor Green
& $size $elf
foreach ($n in @('C1.elf','C1.hex','C1.bin')) {
    $p = Join-Path $build $n
    if (Test-Path $p) { "{0,-10} {1,10:N0} bytes" -f $n, (Get-Item $p).Length }
}
