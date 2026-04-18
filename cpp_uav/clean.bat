@echo off
echo Cleaning build directory...
if exist build (
    rmdir /s /q build
    echo Build dizini silindi.
) else (
    echo Zaten temiz.
)
echo.
