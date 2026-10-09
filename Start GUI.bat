@echo off
cd /d "%~dp0"
python -m pip install --quiet pyserial
python gui.py
if errorlevel 1 pause
