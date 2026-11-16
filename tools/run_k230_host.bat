@echo off
rem 双击运行「假主控」：电脑冒充 F103，通过 CH340 跟 K230 对话
rem 需要 Python 3 + pyserial（缺 pyserial 会自动帮你装）
setlocal
cd /d "%~dp0.."

set "PYEXE="
call :try_py python
if not defined PYEXE call :try_py py
if not defined PYEXE call :try_py python3

if not defined PYEXE goto :no_pyserial
goto :run

rem ---------- 试着用某个解释器，能 import serial 就采用 ----------
:try_py
%1 -c "import serial" >nul 2>&1
if not errorlevel 1 set "PYEXE=%1"
exit /b 0

rem ---------- 有 python 但没 pyserial：自动装 ----------
:no_pyserial
where python >nul 2>&1
if errorlevel 1 goto :no_python
echo.
echo [INFO] Python 已找到，但没有 pyserial，正在安装 ...
python -m pip install pyserial
if errorlevel 1 goto :pip_fail
set "PYEXE=python"
goto :run

:no_python
echo.
echo [ERROR] 没找到 Python 3。
echo         请先装 Python 3：https://www.python.org/downloads/
echo         安装时勾上 "Add Python to PATH"。
goto :end

:pip_fail
echo.
echo [ERROR] 自动安装失败，请手动执行：
echo         python -m pip install pyserial
goto :end

:run
echo.
"%PYEXE%" tools\k230_host.py %*
set "RC=%ERRORLEVEL%"
echo.
rem 双击（没有参数）时停在窗口里，方便看输出；带参数（比如 --selftest）就不停
if "%~1"=="" pause
exit /b %RC%

:end
pause
exit /b 1
