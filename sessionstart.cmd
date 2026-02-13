@echo off
setlocal
set SCRIPT_DIR=%~dp0
call "%SCRIPT_DIR%codex.cmd" %*
exit /b %ERRORLEVEL%
