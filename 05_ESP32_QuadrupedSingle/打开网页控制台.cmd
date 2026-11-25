@echo off
setlocal
set "QUAD_PYTHON=%~dp0..\.tools\Scripts\python.exe"
if exist "%QUAD_PYTHON%" goto launch
set "QUAD_PYTHON=%~dp0..\..\..\..\¹²Ãæ±àÂëÆ÷\encoder-bench\.venv\Scripts\python.exe"
if exist "%QUAD_PYTHON%" goto launch
echo Python environment missing. Please restore the project Python environment.
pause
exit /b 1
:launch
"%QUAD_PYTHON%" "%~dp0tools\web_panel.py"
if errorlevel 1 pause