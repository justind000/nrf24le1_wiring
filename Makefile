CCFLAGS =-I ./include/ -L ./lib/* --std-sdcc99 --model-large --code-loc 0x0000 --code-size 0x4000 --xram-loc 0x0000 --xram-size 0x400
main: main.c
	sdcc $(CCFLAGS) main.c
	packihx main.ihx > main.hex
	perl ./utils/Programmer.pl main.hex /dev/ttyACM0
	sudo gnome-terminal -e 'minicom --device /dev/ttyACM0'
