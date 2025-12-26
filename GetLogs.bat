@echo off
set DEVICE_IP=192.168.43.1:5555
set REMOTE_LOGS=/sdcard/FIRST/logs
set LOCAL_LOGS=logs

echo Connecting to Control Hub...

:: Check if adb is available
where adb >nul 2>&1
if errorlevel 1 (
    echo ERROR: adb not found. Make sure Android Platform Tools are installed and in PATH.
    pause
    exit /b 1
)

:: Connect to device
adb connect %DEVICE_IP% >nul 2>&1
if errorlevel 1 (
    echo ERROR: Failed to connect to %DEVICE_IP%.
    pause
    exit /b 1
)

:: Verify device is listed
adb devices | findstr /C:"%DEVICE_IP%" >nul
if errorlevel 1 (
    echo ERROR: Device not detected after connect.
    pause
    exit /b 1
)

echo Connected successfully.
echo.

:: Delete local logs folder if it exists
if exist "%LOCAL_LOGS%" (
    echo Deleting existing local logs folder...
    rmdir /s /q "%LOCAL_LOGS%"
    if errorlevel 1 (
        echo ERROR: Failed to delete local logs folder.
        pause
        exit /b 1
    )
)

:: Check if logs folder exists on device
adb shell "[ -d %REMOTE_LOGS% ]" >nul 2>&1
if errorlevel 1 (
    echo ERROR: Logs folder not found on device: %REMOTE_LOGS%
    pause
    exit /b 1
)

echo Pulling logs...
adb pull %REMOTE_LOGS% "%LOCAL_LOGS%"
if errorlevel 1 (
    echo ERROR: Failed to pull logs.
    pause
    exit /b 1
)

echo.
echo Done! Logs are in the "%LOCAL_LOGS%" folder.
pause
