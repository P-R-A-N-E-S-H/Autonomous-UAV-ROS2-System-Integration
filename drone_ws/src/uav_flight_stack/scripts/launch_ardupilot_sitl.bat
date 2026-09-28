@echo off
TITLE ArduPilot SITL Flight Stack Runner (Member 2)
echo ================================================================================
echo   STARTING ARDUPILOT SITL SIMULATION (X-QUADROTOR)
echo ================================================================================

python -m dronekit_sitl copter --home=0.0,0.0,0.0,0.0
pause
