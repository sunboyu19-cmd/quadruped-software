@echo off
"%~dp0..\.tools\Scripts\python.exe" "%~dp0tools\web_panel.py"
if errorlevel 1 pause
