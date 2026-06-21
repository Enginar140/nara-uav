@echo off
setlocal

REM ============================================================
REM   ArduPlane SITL launcher (custom feature/local-control fork)
REM ============================================================
REM
REM Bu script Cygwin altinda sim_vehicle.py ile bizim build
REM ettigimiz arduplane.exe'yi calistirir. Yeni bir Cygwin
REM terminal penceresi acilir, MAVProxy konsolu burada olur.
REM
REM Kullanim:
REM   run_arduplane_sitl.bat              -> default (San Francisco home)
REM   run_arduplane_sitl.bat CMAC         -> baska home location
REM
REM Calisan SITL UDP 14550'dan MAVLink yayar; cpp_uav bunu
REM udpin:127.0.0.1:14550 olarak dinler.
REM ============================================================

set CYGWIN_ROOT=C:\cygwin64
set MINTTY=%CYGWIN_ROOT%\bin\mintty.exe
set BASH=%CYGWIN_ROOT%\bin\bash.exe
set AP_DIR=/cygdrive/c/Users/Engin/Desktop/Nara/v10/cpp_uav_external/ardupilot_repo
set AP_BIN=%~dp0..\cpp_uav_external\ardupilot_repo\build\sitl\bin\arduplane.exe

set HOME_LOCATION=%1
if "%HOME_LOCATION%"=="" set HOME_LOCATION=--custom-location=37.8488,-122.3942,0,0

if not exist "%MINTTY%" (
    echo HATA: Cygwin bulunamadi: %CYGWIN_ROOT%
    echo       Kurulum: install_cygwin.ps1
    exit /b 1
)

if not exist "%AP_BIN%" (
    echo HATA: arduplane.exe bulunamadi: %AP_BIN%
    echo       Once Cygwin'de build edin:
    echo         cd %AP_DIR%
    echo         python3 ./waf configure --board sitl
    echo         python3 ./waf plane
    exit /b 1
)

echo.
echo ========================================
echo   ArduPlane SITL (feature/local-control)
echo ========================================
echo   Binary  : %AP_BIN%
echo   Home    : %HOME_LOCATION%
echo   MAVLink : udpout 127.0.0.1:14550 (cpp_uav)
echo   Python  : udpout 127.0.0.1:14552 (tests/sitl_local_control_test.py)
echo   GCS     : udpout 127.0.0.1:14551 (Mission Planner / QGC)
echo.
echo   Yeni Cygwin penceresi aciliyor...
echo   Durdurmak icin o pencerede Ctrl+C.
echo ========================================
echo.

start "ArduPlane SITL" "%MINTTY%" -h always -t "ArduPlane SITL" /bin/bash -lc ^
  "cd %AP_DIR% && python3 Tools/autotest/sim_vehicle.py -v ArduPlane --no-rebuild --console %HOME_LOCATION% --out=127.0.0.1:14550 --out=127.0.0.1:14551 --out=127.0.0.1:14552"

echo SITL baslatildi. cpp_uav icin ayri terminalde:
echo   run_sitl.bat
echo.
endlocal
