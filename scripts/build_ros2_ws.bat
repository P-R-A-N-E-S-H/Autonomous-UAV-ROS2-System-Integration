@echo off
TITLE Build Autonomous UAV ROS 2 Workspace (Windows)
echo ================================================================================
echo   BUILDING AUTONOMOUS UAV ROS 2 WORKSPACE
echo ================================================================================

cd /d "%~dp0..\drone_ws"

echo Checking ROS 2 environment...
call colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Build failed or ROS 2 colcon not in path.
    pause
    exit /b 1
)

echo [SUCCESS] Workspace built successfully!
call install\setup.bat
pause
