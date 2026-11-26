@echo off
chcp 65001 >nul
title ERM diagnostic build + flash

set "GITBASH=C:\Program Files\Git\bin\bash.exe"
if not exist "%GITBASH%" set "GITBASH=C:\Program Files (x86)\Git\bin\bash.exe"
if not exist "%GITBASH%" (
  echo [ERROR] Git Bash not found.
  pause
  exit /b 1
)

echo ============================================
echo  ERM diagnostic firmware - build and flash
echo ============================================
echo.

"%GITBASH%" -c "cd /c/Users/user/Desktop/CM_Lab/2026_summer/firmware/erm_frimware && bash flash.sh"

echo.
echo ============================================
echo  DONE  (exit code = %errorlevel%^)
echo ============================================
pause
