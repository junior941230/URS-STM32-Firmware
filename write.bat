@echo off
setlocal

cd /d "%~dp0"
set "CMAKE=%LOCALAPPDATA%\stm32cube\bundles\cmake\4.3.1+st.1\bin\cmake.exe"

echo [1/3] Configuring Debug build...
"%CMAKE%" --preset Debug
if errorlevel 1 exit /b %errorlevel%

echo [2/3] Building firmware...
"%CMAKE%" --build --preset Debug
if errorlevel 1 exit /b %errorlevel%

echo [3/3] Flashing firmware...
"C:\Users\junio\AppData\Local\stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe" -c port=SWD -w ".\build\Debug\URS.elf" -v -rst
exit /b %errorlevel%
