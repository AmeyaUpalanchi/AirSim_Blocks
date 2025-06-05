@echo off
setlocal EnableDelayedExpansion

:: Batch script to execute keyboard_control.py from the scripts/multirotor directory
:: Date: June 05, 2025
:: Usage: run_drone_control.bat [--verbose]
::   --verbose: Enable velocity logging in keyboard_control.py for debugging

:: Define paths
:: Determine the root directory (where this batch script is located)
set "ROOT_DIR=%~dp0"
set "ROOT_DIR=%ROOT_DIR:~0,-1%"  :: Remove trailing backslash
set "BASE_DIR=%ROOT_DIR%\scripts\multirotor"
set "SCRIPT_NAME=keyboard_control.py"
set "FULL_SCRIPT_PATH=%BASE_DIR%\%SCRIPT_NAME%"
set "PYTHON_EXEC=python"

:: Optional: Path to virtual environment activation script (uncomment if using a virtual env)
:: set "VENV_ACTIVATE=%BASE_DIR%\venv\Scripts\activate.bat"

:: Parse command-line arguments
set "VERBOSE_MODE="
:parse_args
if "%~1"=="" goto end_parse
if /i "%~1"=="--verbose" (
    set "VERBOSE_MODE=--verbose"
    echo Verbose mode enabled: Velocity logging will be active.
)
shift
goto parse_args
:end_parse

:: Display script header
echo.
echo Drone Control Script Launcher
echo =============================
echo Root Directory: %ROOT_DIR%
echo Base Directory: %BASE_DIR%
echo Script: %SCRIPT_NAME%
echo Full Script Path: %FULL_SCRIPT_PATH%
echo.

:: Check if the base directory exists
if not exist "%BASE_DIR%" (
    echo Error: Directory %BASE_DIR% does not exist.
    echo Ensure the scripts/multirotor directory exists relative to %ROOT_DIR%.
    goto error_exit
)

:: Navigate to the multirotor directory
echo Navigating to %BASE_DIR%...
cd /d "%BASE_DIR%"
if errorlevel 1 (
    echo Error: Could not navigate to %BASE_DIR%.
    echo Please ensure you have access permissions.
    goto error_exit
)

:: Check if the Python script exists
if not exist "%SCRIPT_NAME%" (
    echo Error: %SCRIPT_NAME% not found in %BASE_DIR%.
    echo Ensure the script file exists in the specified directory.
    goto error_exit
)

:: Check if Python is available
%PYTHON_EXEC% --version >nul 2>&1
if errorlevel 1 (
    echo Error: Python not found. Ensure Python is installed and added to your PATH.
    echo Alternatively, modify PYTHON_EXEC in this script to the full path of your Python executable.
    goto error_exit
)

:: Run the Python script with optional arguments
echo Executing %SCRIPT_NAME%...
echo.
%PYTHON_EXEC% "%FULL_SCRIPT_PATH%" %VERBOSE_MODE%
if errorlevel 1 (
    echo Error: Failed to execute %SCRIPT_NAME%.
    echo Check the script output above for details.
    goto error_exit
)

:: Success message
echo.
echo Execution completed successfully.
goto normal_exit

:error_exit
echo.
echo Script execution failed. See error messages above.
set "EXIT_CODE=1"
goto final_exit

:normal_exit
echo.
echo Script execution completed.
set "EXIT_CODE=0"
goto final_exit

:final_exit
echo Press any key to exit...
pause >nul
exit /b %EXIT_CODE%

endlocal