@echo off
setlocal enabledelayedexpansion

REM ============================================================
REM   OpenCV (Windows prebuilt) kurulum yardimcisi
REM ------------------------------------------------------------
REM   - Bu script vcpkg/system kurulumu YAPMAZ; sadece resmi
REM     prebuilt zip'i indirip C:\opencv altina acar.
REM   - CMake otomatik C:\opencv\build altinda OpenCVConfig.cmake
REM     arar (CMakeLists.txt icindeki HINTS listesinden bu
REM     yol birinci sirada).
REM ============================================================

set OPENCV_VERSION=4.10.0
set OPENCV_URL=https://github.com/opencv/opencv/releases/download/%OPENCV_VERSION%/opencv-%OPENCV_VERSION%-windows.exe
set OPENCV_DIR=C:\opencv
set OPENCV_INSTALLER=%TEMP%\opencv-%OPENCV_VERSION%-windows.exe

if exist "%OPENCV_DIR%\build\OpenCVConfig.cmake" (
    echo OpenCV zaten kurulu: %OPENCV_DIR%\build
    echo CMake icin: -DOpenCV_DIR=%OPENCV_DIR%\build
    exit /b 0
)

echo.
echo OpenCV %OPENCV_VERSION% indiriliyor (~250 MB) ...
echo URL: %OPENCV_URL%
echo Hedef: %OPENCV_INSTALLER%
echo.

powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12;" ^
  "Invoke-WebRequest -Uri '%OPENCV_URL%' -OutFile '%OPENCV_INSTALLER%' -UseBasicParsing"
if errorlevel 1 (
    echo HATA: Indirme basarisiz. Manuel indirip C:\opencv altina ac:
    echo   %OPENCV_URL%
    exit /b 1
)

echo.
echo Self-extractor calistiriliyor (hedef: C:\) ...
echo Pencere acilacak; "Extract to: C:\" sec ve Extract'a tikla.
"%OPENCV_INSTALLER%"

echo.
if exist "%OPENCV_DIR%\build\OpenCVConfig.cmake" (
    echo BASARI: %OPENCV_DIR%\build\OpenCVConfig.cmake bulundu.
    echo Simdi build et:
    echo   cd cpp_uav
    echo   cmake -S . -B build
    echo   cmake --build build --config Release --target uav_system
    echo.
    echo Calistirmadan once OpenCV runtime DLL'lerini PATH'e ekle:
    echo   set PATH=%OPENCV_DIR%\build\x64\vc16\bin;%%PATH%%
) else (
    echo UYARI: %OPENCV_DIR%\build bulunamadi. Self-extractor'da hedef
    echo klasoru "C:\" sectiginden emin ol ve scripti tekrar calistir.
)

endlocal
