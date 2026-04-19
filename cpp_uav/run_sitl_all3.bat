@echo off
setlocal

REM ============================================================
REM   3 ArduPlane SITL'i paralel baslatir
REM     #1 -> instance 0  TCP 5760/5762  UDP 14550
REM     #2 -> instance 1  TCP 5770/5772  UDP 14560
REM     #3 -> instance 2  TCP 5780/5782  UDP 14570
REM
REM   Her SITL kendi Cygwin penceresinde acilir.
REM ============================================================

echo.
echo [1/3] SITL #1 (instance 0) baslatiliyor...
start "" /D "%~dp0" cmd /c "%~dp0run_sitl_1.bat"

echo [2/3] 3 sn bekleniyor (port baglama icin)...
timeout /t 3 /nobreak >nul

echo [2/3] SITL #2 (instance 1) baslatiliyor...
start "" /D "%~dp0" cmd /c "%~dp0run_sitl_2.bat"

echo [3/3] 3 sn bekleniyor...
timeout /t 3 /nobreak >nul

echo [3/3] SITL #3 (instance 2) baslatiliyor...
start "" /D "%~dp0" cmd /c "%~dp0run_sitl_3.bat"

echo.
echo ========================================
echo   3 SITL instance baslatildi
echo ========================================
echo   #1  Python tcp:127.0.0.1:5760  MP tcp:5762  UDP 14550
echo   #2  Python tcp:127.0.0.1:5770  MP tcp:5772  UDP 14560
echo   #3  Python tcp:127.0.0.1:5780  MP tcp:5782  UDP 14570
echo ========================================
echo.

endlocal
