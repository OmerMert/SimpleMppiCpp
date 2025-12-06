@echo off
REM === CMake Build & Run Script ===

REM 1. Remove existing build directory if it exists
if exist build (
    rd /s /q build
)

del /Q *.exe
del /Q *.pdb

REM 2. Create build directory
mkdir build
cd build

REM 3. Configure the project with CMake
cmake ..

REM 4. Build the project (use Debug or Release as needed)
cmake --build . --config Debug

REM 5. Run the executable (adjust the target name if different)
echo.
echo ======= Running program =======
echo.

cd ..

start "simulator" python mppi_visualizer.py

REM Wait for a moment to ensure the executable is ready
timeout /t 2 >nul 

if exist MppiCpp.exe (

    
    start "controller" MppiCpp.exe
) else (
    echo Error: Executable object is not exist!
    pause
    exit /b
)


