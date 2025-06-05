@echo off
setlocal enabledelayedexpansion

echo AirSim Python Package Installer
echo =================================
echo.

if not exist "scripts" (
    echo ERROR: scripts directory not found in current location.
    echo Please run this script from the AirSim root directory.
    goto :error_exit
)

python --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python not found. Please install Python and add it to PATH.
    goto :error_exit
)

pip --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: pip not found. Please install pip.
    goto :error_exit
)

echo Installing dependencies...
pip install numpy msgpack-rpc-python opencv-python >nul 2>&1
if errorlevel 1 (
    echo WARNING: Some dependencies may have failed to install.
)

echo Setting up AirSim package...
set "AIRSIM_PATH=%CD%\scripts"

for /f "delims=" %%i in ('python -c "import sys; print([p for p in sys.path if 'site-packages' in p and ('local' in p or 'user' in p)][0])" 2^>nul') do set "SITE_PACKAGES=%%i"

if defined SITE_PACKAGES (
    echo !AIRSIM_PATH! > "!SITE_PACKAGES!\airsim.pth"
    echo Package configured at: !SITE_PACKAGES!\airsim.pth
) else (
    echo WARNING: Could not auto-configure. Manual setup required:
    echo Add this line to your Python scripts:
    echo   import sys; sys.path.append(r'!AIRSIM_PATH!')
)

echo.
echo Verifying installation...
python -c "import airsim; print('AirSim package ready')" 2>nul
if errorlevel 1 (
    echo FAILED: Import verification failed
    goto :error_exit
) else (
    echo SUCCESS: Installation completed
)

echo.
pause
exit /b 0

:error_exit
echo.
pause
exit /b 1