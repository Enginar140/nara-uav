@echo off
setlocal

set BUILD_TYPE=%1
if "%BUILD_TYPE%"=="" set BUILD_TYPE=Release

echo.
echo ========================================
echo   UAV v10 - Build (%BUILD_TYPE%)
echo ========================================
echo.

if not exist build (
    echo [1/2] CMake configure...
    cmake -B build -DCMAKE_BUILD_TYPE=%BUILD_TYPE%
    if errorlevel 1 (
        echo HATA: CMake configure basarisiz!
        exit /b 1
    )
) else (
    echo [1/2] Build dizini mevcut, configure atlaniyor...
    echo       (Temiz build icin: clean.bat)
)

echo [2/2] Building...
cmake --build build --config %BUILD_TYPE% --parallel
if errorlevel 1 (
    echo HATA: Build basarisiz!
    exit /b 1
)

echo.
echo ========================================
echo   Build basarili!
echo   Cikti: build\bin\%BUILD_TYPE%\uav_system.exe
echo ========================================
echo.
