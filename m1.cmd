@PATH=C:\Program Files\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin;C:\Program Files\Atmel\Studio\7.0\shellutils;C:\Program Files\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\avr\bin
@SET mk="C:\Program Files\Atmel\Studio\7.0\shellutils\make.exe"
@REM SET mk=make.exe
@REM SET mk_opt=--makefile=Makefile1 
%mk% %mk_opt% clean && %mk% %mk_opt% all && %mk% %mk_opt% program