CC := lcc
CFLAGS := -Wa-l -Wl-m -Wl-j -Wm-p -Wm-ynGBMGR -Wl-yt0		\
	  -D_loop_forever_ram=0xD000 -Wl-g_loop_forever_ram=0xD000

all: gb_manager.gb
gb_manager.gb: src/gb_manager.c
	$(CC) $(CFLAGS) -o $@ $^
