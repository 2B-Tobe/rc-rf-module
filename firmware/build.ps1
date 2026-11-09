# build.ps1 - builds rc_module_trx from a PowerShell prompt (Windows).
#
#   .\build.ps1        run it in the project folder, or give its full path
#
# What it does:
#   1. Marks app_process.c as changed. The protocol files (rlink.c, nrf24_rx.c,
#      frsky_d8.c, frsky_d16.c, dsm_tx.c, dsm_rx.c, dsm_cap.c, sbus_out.c,
#      rx_store.c, bay_serial.c and etx_multi.c) are #included into app_process.c, so without this the build
#      would not notice that one of them changed.
#   2. Finds CMake: first the copy installed by Simplicity Studio or the Silicon
#      Labs VS Code extension (%USERPROFILE%\.silabs\slt\installs\conan\p\...),
#      otherwise a CMake on PATH (3.25 or newer). Nothing is tied to one PC or
#      one user name.
#   3. Configures and builds cmake_gcc\. The compiler and Ninja are located by
#      cmake_gcc\toolchain.cmake, which the IDE generates when the project is
#      imported.
#
# Result: cmake_gcc\build\base\rc_module_trx.hex
# Flash:  pyocd flash -t efr32mg24a420f1536im48 cmake_gcc\build\base\rc_module_trx.hex   (from the VS Code terminal)

$ErrorActionPreference = 'Stop'
$proj  = $PSScriptRoot
$conan = Join-Path $env:USERPROFILE '.silabs\slt\installs\conan\p'

if (-not (Test-Path (Join-Path $proj 'cmake_gcc\CMakePresets.json'))) {
  throw "cmake_gcc\ is missing. Import rc_module_trx.slcp once in Simplicity Studio or VS Code (Silicon Labs extension); that generates cmake_gcc\. Then run build.ps1 again."
}

# 1. force the rebuild of app_process.c and everything #included into it
(Get-Item (Join-Path $proj 'app_process.c')).LastWriteTime = Get-Date

# 2. CMake: the Silicon Labs copy first, then PATH
$cmake = $null
if (Test-Path $conan) {
  $cmake = Get-ChildItem -Path (Join-Path $conan '*\p\bin\cmake.exe') -ErrorAction SilentlyContinue |
           Sort-Object LastWriteTime -Descending | Select-Object -First 1 -ExpandProperty FullName
}
if (-not $cmake) {
  $c = Get-Command cmake -ErrorAction SilentlyContinue
  if ($c) { $cmake = $c.Source }
}
if (-not $cmake) {
  throw "CMake not found. Install Simplicity Studio 6 or VS Code with the Silicon Labs extension (Simplicity SDK 2026.6.1, Arm GCC 14.2), import rc_module_trx.slcp, then run build.ps1 again."
}

# the Silicon Labs Ninja and Arm GCC go on PATH too, as the IDE does it
$dirs = @(Split-Path $cmake)
if (Test-Path $conan) {
  Get-ChildItem -Path (Join-Path $conan '*\p\ninja.exe'), (Join-Path $conan '*\p\bin\arm-none-eabi-gcc.exe') -ErrorAction SilentlyContinue |
    ForEach-Object { $dirs += $_.DirectoryName }
}
$env:Path = (($dirs | Select-Object -Unique) -join ';') + ';' + $env:Path
Write-Host "CMake: $cmake"

# 3. configure and build
Push-Location (Join-Path $proj 'cmake_gcc')
try {
  & $cmake --preset project
  if ($LASTEXITCODE -ne 0) { throw "CMake configure failed (see above)." }
  & $cmake --build --preset default_config
  if ($LASTEXITCODE -ne 0) { throw "Build failed (see above)." }
} finally {
  Pop-Location
}
Write-Host ("Built: " + (Join-Path $proj 'cmake_gcc\build\base\rc_module_trx.hex'))
