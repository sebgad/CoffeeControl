@echo off
REM expected tool to be in  \Arduino\coffee_ctrl_main\tools
REM 		  and build in  \Arduino\build
:start
cd..\..
cd build
REM FOR %%? IN ("C:\Programmieren\Arduino\build\coffee_ctrl_main.ino.bin") DO (
FOR %%? IN (coffee_ctrl_main.ino.bin) DO (
    ECHO Fully File Path    : %%~f?
	ECHO Last-Modified Date : %%~t?
)
echo[
certutil -hashfile coffee_ctrl_main.ino.bin MD5
echo[
set choice=
set /p choice="Do you want to restart? Press 'y' and enter for Yes: "
if not '%choice%'=='' set choice=%choice:~0,1%
if '%choice%'=='y' goto start