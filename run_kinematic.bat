@echo off
REM ============================================================
REM  Phase 1: kinematic-bicycle Isaac Sim <-> C++ MPPI bridge.
REM  Launches MppiCpp.exe automatically (no need to run it separately).
REM
REM  Usage:
REM    run_kinematic.bat                       (uses REF_PATH_FILE from config.json)
REM    run_kinematic.bat --headless            (no GUI window)
REM    run_kinematic.bat data\path_sine.csv    (override the path)
REM ============================================================
setlocal
cd /d "%~dp0"

REM Keep Isaac's temp/cache off the small C: drive
if not exist "D:\isaac_tmp" mkdir "D:\isaac_tmp"
set "PYTHONUNBUFFERED=1"
set "TEMP=D:\isaac_tmp"
set "TMP=D:\isaac_tmp"

REM Isaac Sim's bundled Python (edit if your build path differs)
set "ISAAC_PY=D:\isaacsim\_build\windows-x86_64\release\python.bat"

if not exist "%ISAAC_PY%" (
    echo ERROR: Isaac Sim python not found: %ISAAC_PY%
    echo Edit ISAAC_PY in this script to point to your build.
    exit /b 1
)
if not exist "%~dp0MppiCpp.exe" (
    echo ERROR: MppiCpp.exe not found. Build the C++ project first ^(build.bat^).
    exit /b 1
)

call "%ISAAC_PY%" "%~dp0isaacsim_bridge.py" %*
endlocal
