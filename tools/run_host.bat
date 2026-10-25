@echo off
rem 双击运行无线串口上位机（需要 Python 3 + pyserial）
cd /d "%~dp0"
python chassis_host.py
if errorlevel 1 pause
