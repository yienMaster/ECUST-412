@echo off
setlocal enabledelayedexpansion
cd /d "%~dp0"
for %%i in ("%cd%") do set "folderName=%%~nxi"
set /p "userInput=请输入新的工程名称: "
set "sourceFile=.project"
set "tempFile=temp.txt"
set "oldString=%folderName%"
set "newString=%userInput%"
if exist "%tempFile%" del /Q "%tempFile%"
for /f "delims=" %%i in ('type "%sourceFile%"') do (
    set "line=%%i"
    set "modified=!line!"
    if defined modified (
        set "modified=!modified:%oldString%=%newString%!"
        echo !modified!>>"%tempFile%"
    ) else (
        echo %%i>>"%tempFile%"
    )
)
move /y "%tempFile%" "%sourceFile%" >nul 2>&1

set "sourceFile=.cproject"
if exist "%tempFile%" del /Q "%tempFile%"
for /f "delims=" %%i in ('type "%sourceFile%"') do (
    set "line=%%i"
    set "modified=!line!"
    if defined modified (
        set "modified=!modified:%oldString%=%newString%!"
        echo !modified!>>"%tempFile%"
    ) else (
        echo %%i>>"%tempFile%"
    )
)
move /y "%tempFile%" "%sourceFile%" >nul 2>&1


endlocal

