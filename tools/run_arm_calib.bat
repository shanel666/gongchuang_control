@echo off
rem Arm calibration host GUI: lift stepper + 3 servos in one window
rem Firmware: DEBUG_MODE = 16 (DEBUG_MODE_ARM_ALL)
rem Stepper: addr 6 | Servos: ROT=PC9  GRIP=PC8  LIFT=PC7
setlocal
cd /d "%~dp0.."

set "PYEXE="
set "NEED_SERIAL="

call :try_py python
if not defined PYEXE call :try_py py
if not defined PYEXE call :try_py python3
if not defined PYEXE goto :no_python
goto :run

:try_py
%1 -c "import tkinter" >nul 2>&1
if errorlevel 1 exit /b 0
set "PYEXE=%1"
%1 -c "import serial" >nul 2>&1
if errorlevel 1 set "NEED_SERIAL=1"
exit /b 0

:run
if defined NEED_SERIAL (
    echo.
    echo [INFO] pyserial is missing, installing ...
    "%PYEXE%" -m pip install pyserial
    if errorlevel 1 (
        echo [ERROR] pip install failed. Run: "%PYEXE%" -m pip install pyserial
        pause
        exit /b 1
    )
)
echo.
"%PYEXE%" tools\arm_calib_host.py %*
set "RC=%ERRORLEVEL%"
echo.
if "%~1"=="" pause
exit /b %RC%

:no_python
echo.
echo [ERROR] No Python 3 with tkinter found. Install Python 3 from python.org
echo         (tick "tcl/tk and IDLE"), then: python -m pip install pyserial
echo.
pause
exit /b 1