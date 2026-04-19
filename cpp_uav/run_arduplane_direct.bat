@echo off
setlocal

REM ============================================================
REM   ArduPlane SITL - DOGRUDAN MOD (MAVProxy bypass)
REM ============================================================
REM
REM Bu script arduplane.exe'yi dogrudan calistirir, MAVProxy yok.
REM Test araclari TCP 5760'a baglanmali:
REM   py tests\sitl_local_control_test.py tcp:127.0.0.1:5760
REM
REM Birden fazla client gerekiyorsa run_arduplane_sitl.bat
REM kullan (MAVProxy ile multi-out).
REM ============================================================

set CYGWIN_ROOT=C:\cygwin64
set MINTTY=%CYGWIN_ROOT%\bin\mintty.exe
set AP_DIR=/cygdrive/c/Users/Engin/Desktop/Nara/v10/cpp_uav_external/ardupilot_repo
set AP_BIN=%~dp0..\cpp_uav_external\ardupilot_repo\build\sitl\bin\arduplane.exe

if not exist "%MINTTY%" (
    echo HATA: Cygwin bulunamadi: %CYGWIN_ROOT%
    exit /b 1
)

if not exist "%AP_BIN%" (
    echo HATA: arduplane.exe bulunamadi: %AP_BIN%
    exit /b 1
)

echo.
echo ========================================
echo   ArduPlane SITL (direct mode, multi-out)
echo ========================================
echo   Binary  : %AP_BIN%
echo   Home    : 37.8488,-122.3942,0,0
echo.
echo   serial0 / TCP server : 0.0.0.0:5760
echo                          -> Python test script (tcp:127.0.0.1:5760)
echo   serial1 / TCP server : 0.0.0.0:5762
echo                          -> Mission Planner (TCP client to localhost:5762)
echo   serial2 / UDP-out    : 127.0.0.1:14550
echo                          -> opsiyonel ek dinleyici (UDP listen :14550)
echo.
echo   Yeni Cygwin penceresi aciliyor...
echo.
echo   Mission Planner'da:  Connect -> TCP -> Server: 127.0.0.1, Port: 5762
echo   Python test:         py tests\sitl_local_control_test.py tcp:127.0.0.1:5760
echo.
echo ========================================
echo.

start "ArduPlane SITL (direct)" "%MINTTY%" -h always -t "ArduPlane SITL (direct)" /bin/bash -lc ^
  "cd %AP_DIR% && ./build/sitl/bin/arduplane --model plane --speedup 1 --home 37.8488,-122.3942,0,0 --defaults Tools/autotest/models/plane.parm --serial0=tcp:5760 --serial1=tcp:5762 --serial2=udpclient:127.0.0.1:14550; echo; echo SITL exited - press Enter to close; read"

echo SITL baslatildi.
echo   - Mission Planner: TCP -> 127.0.0.1:5762
echo   - py tests\sitl_local_control_test.py tcp:127.0.0.1:5760
echo.
endlocal
