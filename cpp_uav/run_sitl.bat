@echo off
setlocal

set BUILD_TYPE=Release
set EXE=build\bin\%BUILD_TYPE%\uav_system.exe
set CONFIG=config\sitl.yaml

if not exist "%EXE%" (
    echo Executable bulunamadi, once build ediliyor...
    call build.bat %BUILD_TYPE%
    if errorlevel 1 exit /b 1
)

echo.
echo ========================================
echo   UAV v10 - SITL Mode
echo ========================================
echo   Config: %CONFIG%
echo   Pixhawk: udpin:127.0.0.1:14550
echo   GCS:     tcpin:127.0.0.1:14552
echo.
echo   ArduPilot SITL baslatin:
echo     sim_vehicle.py -v ArduPlane --map --console
echo.
echo   Ctrl+C ile durdurun.
echo ========================================
echo.

"%EXE%" "%CONFIG%"
