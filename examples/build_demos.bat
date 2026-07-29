@echo off
setlocal enabledelayedexpansion

cd /d "%~dp0"

rem Use the project virtual environment for all python/pip/pyinstaller calls
set "VENV_PY=%~dp0..\.venv\Scripts\python.exe"
if not exist "%VENV_PY%" (
    echo [ERROR] Virtual environment python not found:
    echo   %VENV_PY%
    exit /b 1
)

echo ==========================================
echo Check pyinstaller...
echo ==========================================
"%VENV_PY%" -m PyInstaller --version >nul 2>&1
if errorlevel 1 (
    echo [ERROR] pyinstaller not found in the virtual environment.
    echo Please install it first:
    echo   "%VENV_PY%" -m pip install pyinstaller
    exit /b 1
)

echo.
echo ==========================================
echo Clean all Python bytecode caches...
echo ==========================================
for /d /r ".." %%d in (__pycache__) do (
    if exist "%%d" (
        echo Removing "%%d"
        rd /s /q "%%d"
    )
)
del /s /q "..\*.pyc" >nul 2>&1
del /s /q "..\*.pyo" >nul 2>&1

echo.
echo ==========================================
echo Remove old PyInstaller build artifacts...
echo ==========================================
for %%F in (DemoNewEMG DemoNewEEG DemoNewPPG SynchroniSDKPython_DemoNewEMG SynchroniSDKPython_DemoNewEEG SynchroniSDKPython_DemoNewPPG) do (
    if exist "build\%%F" (
        echo Removing "build\%%F"
        rd /s /q "build\%%F"
    )
    if exist "dist\%%F" (
        echo Removing "dist\%%F"
        rd /s /q "dist\%%F"
    )
    if exist "%%F.spec" (
        echo Removing "%%F.spec"
        del /f /q "%%F.spec"
    )
)

del /f /q "..\*.spec" >nul 2>&1

"%VENV_PY%" -m pip uninstall -U synchroni-sensor-sdk

"%VENV_PY%" -m pip install -U sensor-sdk

echo.
echo ==========================================
echo Build demos with PyInstaller...
echo ==========================================
set "COMMON_OPTS=--clean --noconfirm --onefile"

"%VENV_PY%" -m PyInstaller %COMMON_OPTS% --additional-hooks-dir "%~dp0..\sensor\pyinstaller_hooks" --name DemoNewEMG SynchroniSDKPython_DemoNewEMG.py
if errorlevel 1 goto :error

"%VENV_PY%" -m PyInstaller %COMMON_OPTS% --additional-hooks-dir "%~dp0..\sensor\pyinstaller_hooks" --name DemoNewEEG SynchroniSDKPython_DemoNewEEG.py
if errorlevel 1 goto :error

"%VENV_PY%" -m PyInstaller %COMMON_OPTS% --additional-hooks-dir "%~dp0..\sensor\pyinstaller_hooks" --name DemoNewPPG SynchroniSDKPython_DemoNewPPG.py
if errorlevel 1 goto :error

echo.
echo ==========================================
echo All demos built successfully.
echo Output: %cd%\dist
 echo ==========================================
goto :eof

:error
echo.
echo ==========================================
echo [ERROR] Build failed!
echo ==========================================
exit /b 1
