@echo off

set MATLAB=F:\tools\Matlab

cd .

if "%1"=="" (F:\tools\Matlab\bin\win64\gmake -f ekf_13state_rtw.mk all) else (F:\tools\Matlab\bin\win64\gmake -f ekf_13state_rtw.mk %1)
@if errorlevel 1 goto error_exit

exit /B 0

:error_exit
echo The make command returned an error of %errorlevel%
An_error_occurred_during_the_call_to_make
