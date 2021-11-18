RGBLINK := rgblink
RGBASM  := rgbasm
RGBFIX  := rgbfix

all: hello-world.gb
hello-world.gb: hello-world.o
hello-world.o: hello-world.asm

%.gb: %.o
	$(RGBLINK) --dmg --pad 0xFF --nopad --tiny\
		--map $(@:.gb=.map) --sym $(@:.gb=.sym) --output $@ $<
	$(RGBFIX) --validate --title $(@:.gb=) --non-japanese $@

%.o: %.asm
	$(RGBASM) -Wall -Wextra --preserve-ld --output $@ $<
