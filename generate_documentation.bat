@echo off
echo Checking for curl...

mkdir docs

REM Check if curl is available
curl --version >nul 2>&1
if errorlevel 1 (
    echo curl is not installed.
    echo Redirecting to download page...
    start "" "https://curl.se/windows/"
    echo Please download the curl executable from the opened page and place it in your PATH or in the same directory as this script.
    pause
    exit /b
)

REM Check if plantuml.jar exists
if exist plantuml.jar (
    echo plantuml.jar already exists. Skipping download...
) else (
    echo Downloading plantuml.jar...
    curl -L -o plantuml.jar https://sourceforge.net/projects/plantuml/files/latest/download

    if exist plantuml.jar (
        echo Download complete: plantuml.jar
    ) else (
        echo Download failed. Exiting.
        pause
        exit /b
    )
)

REM Run Doxygen with Doxyfile
echo Running Doxygen with Doxyfile...
doxygen Doxyfile

pause