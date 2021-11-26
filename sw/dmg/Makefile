RGBLINK := rgblink
RGBASM  := rgbasm
RGBFIX  := rgbfix
RGBGFX  := rgbgfx

all: hello-world.gb gen_charmap.exe
gen_charmap.exe: gen_charmap.o
hello-world.gb: hello-world.o
hello-world.o: hello-world.asm F77SMC6_8x8_mini.1bpp

%.gb: %.o
	$(RGBLINK) --dmg --pad 0xFF --nopad --tiny\
		--map $(@:.gb=.map) --sym $(@:.gb=.sym) --output $@ $<
	$(RGBFIX) --validate --title $(@:.gb=) --non-japanese $@

%.o: %.asm
	$(RGBASM) -Wall -Wextra --output $@ $<

%.exe: %.o
	$(CC) -Wall -Os -s -o $@ $<

%.1bpp %.tilemap: %.png
	$(RGBGFX) --depth 1 --output-tilemap --output $@ $<

%.2bpp %.tilemap: %.png
	$(RGBGFX) --depth 2 --output-tilemap --output $@ $<

clean:
	$(RM) *.gb *.o *.sym *.map
