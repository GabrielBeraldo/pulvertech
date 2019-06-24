
color a
./avrdude/ avrdude -C./avrdude.conf -v -patmega328p -cusbasp -Pusb -Uflash:w:./.pioenvs\uno/firmware.hex:i

pause