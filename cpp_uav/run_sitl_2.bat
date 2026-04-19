@echo off
setlocal

REM ============================================================
REM   ArduPlane SITL #2  (instance 1)
REM ============================================================
REM   Cikislar:
REM     TCP 5770  -> Python test scripti / cpp_uav (tcp:127.0.0.1:5770)
REM     TCP 5772  -> Mission Planner    (TCP client -> 127.0.0.1:5772)
REM     UDP 14560 -> ek dinleyici / GCS (udpin:127.0.0.1:14560)
REM
REM   #1 ile ayni anda calistirilabilir; -I1 ile log/sim portlari
REM   ayri, fence/param dosyalari da ayri klasorde tutulur.
REM ============================================================

set CYGWIN_ROOT=C:\cygwin64
set MINTTY=%CYGWIN_ROOT%\bin\mintty.exe
set AP_DIR=/cygdrive/c/Users/Engin/Desktop/Nara/v10/cpp_uav_external/ardupilot_repo
set AP_BIN=%~dp0..\cpp_uav_external\ardupilot_repo\build\sitl\bin\arduplane.exe

set INSTANCE=1
REM Home'u biraz oteleyelim ki harita uzerinde 3 ucagi ayirt edebilelim
set HOME_POS=37.8520,-122.3942,0,90
set TCP_PRIMARY=5770
set TCP_GCS=5772
set UDP_OUT=14560

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
echo   ArduPlane SITL #2 (instance %INSTANCE%)
echo ========================================
echo   Home          : %HOME_POS%
echo   Python TCP    : 127.0.0.1:%TCP_PRIMARY%
echo   MissionPlnnr  : 127.0.0.1:%TCP_GCS%   (TCP client)
echo   UDP-out       : 127.0.0.1:%UDP_OUT%   (udpin)
echo ========================================
echo.

start "ArduPlane SITL #2" "%MINTTY%" -h always -t "ArduPlane SITL #2 (I=%INSTANCE%)" /bin/bash -lc ^
  "cd %AP_DIR% && ./build/sitl/bin/arduplane --model plane -I%INSTANCE% --speedup 1 --home %HOME_POS% --defaults Tools/autotest/models/plane.parm --serial0=tcp:%TCP_PRIMARY% --serial1=tcp:%TCP_GCS% --serial2=udpclient:127.0.0.1:%UDP_OUT%; echo; echo SITL #2 exited - press Enter to close; read"

echo SITL #2 baslatildi.
echo   - Python:         py tests\sitl_local_control_test.py tcp:127.0.0.1:%TCP_PRIMARY%
echo   - MissionPlanner: Connect -> TCP -> 127.0.0.1 : %TCP_GCS%
echo   - UDP listener  : udpin:127.0.0.1:%UDP_OUT%
echo.
endlocal
