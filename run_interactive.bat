@echo off
setlocal
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0run_simulation.ps1"
set "simulation_exit=%errorlevel%"
if not "%simulation_exit%"=="0" pause
exit /b %simulation_exit%
