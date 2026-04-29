@echo off
setlocal
cd /d "%~dp0"
py -m pip install --upgrade pip
py -m pip install -r requirements-can-monitor.txt
echo.
echo Install complete.
echo Run start_can_monitor.bat to open the GUI.
pause
