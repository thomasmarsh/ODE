@echo off
echo.
echo. - Configuring ODe VC6 Workspace -
echo.
copy _configs\config-double.h ..\include\ode\config.h

rem remove the Debug and Release dirs to force a complete rebuild

del /Q Debug
del /Q Release

rem done.