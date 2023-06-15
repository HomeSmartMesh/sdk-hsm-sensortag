cd %~dp0\..\..\scripts\
call on.cmd
cd %~dp0
west flash
cd %~dp0\..\..\scripts\
call off.cmd
cd %~dp0
