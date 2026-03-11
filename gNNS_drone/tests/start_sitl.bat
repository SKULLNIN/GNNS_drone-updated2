@echo off
REM gNNS Drone — SITL Setup for Windows
REM =====================================
REM This script helps set up ArduPilot SITL for testing.
REM
REM OPTION 1: Docker (RECOMMENDED)
REM OPTION 2: WSL (if you have WSL installed)
REM OPTION 3: Mission Planner (GUI)

echo.
echo ===================================================
echo   gNNS Drone — SITL Simulation Setup
echo ===================================================
echo.

REM Check if Docker is available
where docker >nul 2>nul
if %ERRORLEVEL% EQU 0 (
    echo [OK] Docker found!
    echo.
    echo Starting ArduPilot SITL via Docker...
    echo   This will download ~500MB on first run.
    echo   SITL will be available at tcp:127.0.0.1:5760
    echo.
    echo   Press Ctrl+C to stop.
    echo.
    docker run -it --rm ^
        -p 5760:5760 ^
        -p 5762:5762 ^
        -p 5763:5763 ^
        radarku/ardupilot-sitl ^
        /ardupilot/Tools/autotest/sim_vehicle.py ^
        -v ArduCopter ^
        --no-mavproxy ^
        -l -35.363262,149.165237,584,353 ^
        --out=tcpin:0.0.0.0:5760 ^
        --out=tcpin:0.0.0.0:5762
) else (
    echo [!] Docker not found.
    echo.
    echo OPTION 1: Install Docker Desktop
    echo   https://docs.docker.com/desktop/install/windows/
    echo.
    echo OPTION 2: Use WSL
    echo   1. Open WSL terminal
    echo   2. Install ArduPilot:
    echo      git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
    echo      cd ardupilot
    echo      Tools/environment_install/install-prereqs-ubuntu.sh -y
    echo      . ~/.profile
    echo   3. Run SITL:
    echo      cd ArduCopter
    echo      sim_vehicle.py --console --map --out=tcp:0.0.0.0:5762
    echo.
    echo OPTION 3: Mission Planner
    echo   1. Download: https://firmware.ardupilot.org/Tools/MissionPlanner/
    echo   2. Open Mission Planner
    echo   3. Go to Simulation tab
    echo   4. Select "Multirotor" and start
    echo.
)

pause
