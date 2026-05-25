@echo off
setlocal

cd /d "%~dp0"

if exist "MDK-ARM\light_test.uvprojx" (
  start "" "MDK-ARM\light_test.uvprojx"
  exit /b 0
)

echo MDK-ARM\light_test.uvprojx was not found.
pause
exit /b 1
