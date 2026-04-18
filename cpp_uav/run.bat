@echo off
setlocal

set CONFIG=%1
set BUILD_TYPE=%2
if "%BUILD_TYPE%"=="" set BUILD_TYPE=Release

set EXE=build\bin\%BUILD_TYPE%\uav_system.exe

if not exist "%EXE%" (
    echo Executable bulunamadi: %EXE%
    echo Once build edin: build.bat
    exit /b 1
)

echo.
echo ========================================
echo   UAV v10 - Calistiriliyor
echo ========================================

if "%CONFIG%"=="" (
    echo   Config: default (otomatik arama)
    echo.
    "%EXE%"
) else (
    if not exist "%CONFIG%" (
        echo   HATA: Config dosyasi bulunamadi: %CONFIG%
        exit /b 1
    )
    echo   Config: %CONFIG%
    echo.
    "%EXE%" "%CONFIG%"
)
