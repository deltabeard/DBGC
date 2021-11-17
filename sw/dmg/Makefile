RGBLINK := rgblink
RGBASM  := rgbasm
RGBFIX  := rgbfix


all: hello-world.gb
hello-world.gb: hello-world.o
hello-world.o: hello-world.asm


%.gb: %.o
	$(RGBLINK) -o $@ $<
	$(RGBFIX) -v -p 0xFF $@

%.o: %.asm
	$(RGBASM) -L -o $@ $<
