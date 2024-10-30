@echo off
cd /d "%~dp0"
setlocal enabledelayedexpansion
mkdir docs

:: Check for administrative privileges
net session >nul 2>&1
if %errorlevel% neq 0 (
    echo This script requires administrator privileges. Restarting with elevated permissions...
    powershell -Command "Start-Process '%0' -Verb RunAs"
    exit /b
)

echo Deleting PlantUML jar file...
del plantuml.jar

echo Download and install jq
curl -sL -o jq.exe https://github.com/stedolan/jq/releases/download/jq-1.6/jq-win64.exe

echo Extract download URL that ends with "plantuml.jar" from JSON response using jq
for /f "delims=" %%a in ('curl -s https://api.github.com/repos/plantuml/plantuml/releases/latest ^| jq -r ".assets[] | select(.name | endswith(\"plantuml.jar\")) | .browser_download_url"') do (
    set download_url=%%a
)

echo Download plantuml.jar
curl -sL -o plantuml.jar "%download_url%"

echo Download URL: %download_url%
echo PlantUML downloaded successfully!

echo Deleting PlantUML jar file...
del jq.exe

:: Install Chocolatey
IF NOT EXIST "%ProgramData%\chocolatey\bin\choco.exe" (
    powershell -NoProfile -InputFormat None -ExecutionPolicy Bypass -Command "iex ((New-Object System.Net.WebClient).DownloadString('https://chocolatey.org/install.ps1'))"
    echo Please restart your command prompt for PATH updates to take effect if Chocolatey was newly installed.
    pause
)

:: Check if Doxygen is installed, install if not
choco list -l doxygen >nul 2>&1
if %errorlevel% neq 0 (
    echo Installing Doxygen...
    choco install doxygen.install -y
)

REM Run Doxygen with Doxyfile
if exist Doxyfile (
    echo Running Doxygen with Doxyfile...
    doxygen Doxyfile
) else (
    echo Doxyfile not found. Please make sure Doxyfile exists in the script directory.
)

pause
