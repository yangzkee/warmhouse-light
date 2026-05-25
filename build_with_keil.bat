@echo off
setlocal

cd /d "%~dp0"

set "UV4="
for %%P in (
  "%KEIL_ROOT%\UV4\UV4.exe"
  "%KEIL_ROOT%\UV4.exe"
  "C:\Keil_v5\UV4\UV4.exe"
  "C:\Keil\UV4\UV4.exe"
) do (
  if exist "%%~P" set "UV4=%%~P"
)

if "%UV4%"=="" (
  for /f "delims=" %%P in ('where UV4.exe 2^>nul') do (
    if "%UV4%"=="" set "UV4=%%P"
  )
)

if "%UV4%"=="" (
  echo Keil UV4.exe was not found.
  echo Open MDK-ARM\light_test.uvprojx in Keil uVision and build target "light_test" manually.
  pause
  exit /b 1
)

echo Using Keil:
echo %UV4%
echo.
echo Rebuilding target "light_test"...
"%UV4%" -r "MDK-ARM\light_test.uvprojx" -t "light_test" -o "build_log.txt"
set "BUILD_EXIT=%ERRORLEVEL%"

echo.
if "%BUILD_EXIT%"=="0" (
  echo Build finished with no errors or warnings.
) else if "%BUILD_EXIT%"=="1" (
  echo Build finished with warnings only.
) else (
  echo Build failed. See build_log.txt.
)

if exist "MDK-ARM\light_test\light_test.hex" (
  if not exist "firmware" mkdir "firmware"
  copy /Y "MDK-ARM\light_test\light_test.hex" "firmware\light.hex" >nul
  echo HEX output:
  echo MDK-ARM\light_test\light_test.hex
  echo Synced:
  echo firmware\light.hex
) else (
  echo HEX output was not found.
)

pause
exit /b %BUILD_EXIT%
