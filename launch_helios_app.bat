@echo off
echo Starting HELIOS BMS Monitor...

:: Activate the virtual environment located in the .venv folder
call ".\.venv\Scripts\activate.bat"
if %ERRORLEVEL% NEQ 0 (
    echo Error activating virtual environment from .venv folder.
    echo Please make sure the virtual environment is correctly set up.
    pause
    exit /b 1
)

:: Launch the application
echo Launching HELIOS BMS Monitor...
python tkinter-plot-adaptation.py

:: If an error occurs during execution, pause to show error output
if %ERRORLEVEL% NEQ 0 (
    echo Application exited with error code %ERRORLEVEL%
    pause
    exit /b %ERRORLEVEL%
)

:: Clean exit
exit /b 0
