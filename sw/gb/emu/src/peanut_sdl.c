/**
 * MIT License
 * Copyright (c) 2018 Mahyar Koshkouei
 *
 * An example of using the peanut_gb.h library. This example application uses
 * SDL2 to draw the screen and get input.
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <SDL.h>

#include "peanut_gb.h"

struct priv_t
{
	/* Pointer to allocated memory holding GB file. */
	uint8_t *rom;

	/* Colour palette for each BG, OBJ0, and OBJ1. */
	uint16_t selected_palette[3][4];
	uint16_t fb[LCD_HEIGHT][LCD_WIDTH];
};

/**
 * Returns a byte from the ROM file at the given address.
 */
uint8_t gb_rom_read(struct gb_s *gb, const uint_fast32_t addr)
{
	const struct priv_t * const p = gb->direct.priv;
	return p->rom[addr];
}

/**
 * Returns a byte from the cartridge RAM at the given address.
 */
uint8_t gb_cart_ram_read(struct gb_s *gb, const uint_fast32_t addr)
{
	return 0xFF;
}

/**
 * Writes a given byte to the cartridge RAM at the given address.
 */
void gb_cart_ram_write(struct gb_s *gb, const uint_fast32_t addr,
		       const uint8_t val)
{
	(void) addr;
	(void) val;
}

/**
 * Returns a pointer to the allocated space containing the ROM. Must be freed.
 */
uint8_t *read_rom_to_ram(const char *file_name)
{
	FILE *rom_file = fopen(file_name, "rb");
	size_t rom_size;
	uint8_t *rom = NULL;

	if(rom_file == NULL)
		return NULL;

	fseek(rom_file, 0, SEEK_END);
	rom_size = ftell(rom_file);
	rewind(rom_file);
	rom = malloc(rom_size);

	if(fread(rom, sizeof(uint8_t), rom_size, rom_file) != rom_size)
	{
		free(rom);
		fclose(rom_file);
		return NULL;
	}

	fclose(rom_file);
	return rom;
}

/**
 * Handles an error reported by the emulator. The emulator context may be used
 * to better understand why the error given in gb_err was reported.
 */
void gb_error(struct gb_s *gb, const enum gb_error_e gb_err, const uint16_t val)
{
	struct priv_t *priv = gb->direct.priv;

	switch(gb_err)
	{
	case GB_INVALID_OPCODE:
		/* We compensate for the post-increment in the __gb_step_cpu
		 * function. */
		fprintf(stdout, "Invalid opcode %#04x at PC: %#06x, SP: %#06x\n",
			val,
			gb->cpu_reg.pc - 1,
			gb->cpu_reg.sp);
		break;

	/* Ignoring non fatal errors. */
	case GB_INVALID_WRITE:
	case GB_INVALID_READ:
		return;

	default:
		printf("Unknown error");
		break;
	}

	fprintf(stderr, "Exiting.");
	exit(EXIT_FAILURE);
}

/**
 * Automatically assigns a colour palette to the game using a given game
 * checksum.
 * TODO: Not all checksums are programmed in yet because I'm lazy.
 */
void auto_assign_palette(struct priv_t *priv, uint8_t game_checksum)
{
	size_t palette_bytes = 3 * 4 * sizeof(uint16_t);

	const uint16_t palette[3][4] =
	{
		{ 0x7FFF, 0x5294, 0x294A, 0x0000 },
		{ 0x7FFF, 0x5294, 0x294A, 0x0000 },
		{ 0x7FFF, 0x5294, 0x294A, 0x0000 }
	};
	memcpy(priv->selected_palette, palette, palette_bytes);
}

#if ENABLE_LCD
/**
 * Draws scanline into framebuffer.
 */
void lcd_draw_line(struct gb_s *gb, const uint8_t pixels[160],
		   const uint_least8_t line)
{
	struct priv_t *priv = gb->direct.priv;

	for(unsigned int x = 0; x < LCD_WIDTH; x++)
	{
		priv->fb[line][x] = priv->selected_palette
				    [(pixels[x] & LCD_PALETTE_ALL) >> 4]
				    [pixels[x] & 3];
	}
}
#endif

/**
 * Saves the LCD screen as a 15-bit BMP file.
 */
void save_lcd_bmp(struct gb_s* gb, uint16_t fb[LCD_HEIGHT][LCD_WIDTH])
{
	/* Should be enough to record up to 828 days worth of frames. */
	static uint_fast32_t file_num = 0;
	char file_name[32];
	char title_str[16];
	FILE* f;

	snprintf(file_name, 32, "%.16s_%010ld.bmp",
		 gb_get_rom_name(gb, title_str), file_num);

	f = fopen(file_name, "wb");

	const uint8_t bmp_hdr_rgb555[] = {
		0x42, 0x4d, 0x36, 0xb4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x36, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0xa0, 0x00,
		0x00, 0x00, 0x70, 0xff, 0xff, 0xff, 0x01, 0x00, 0x10, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0xb4, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00
	};

	fwrite(bmp_hdr_rgb555, sizeof(uint8_t), sizeof(bmp_hdr_rgb555), f);
	fwrite(fb, sizeof(uint16_t), LCD_HEIGHT * LCD_WIDTH, f);
	fclose(f);

	file_num++;

	/* Each dot shows a new frame being saved. */
	putc('.', stdout);
	fflush(stdout);
}

int main(int argc, char **argv)
{
	struct gb_s gb;
	struct priv_t priv =
	{
		.rom = NULL,
	};
	const double target_speed_ms = 1000.0 / VERTICAL_SYNC;
	double speed_compensation = 0.0;
	SDL_Window *window;
	SDL_Renderer *renderer;
	SDL_Texture *texture;
	SDL_Event event;
	SDL_GameController *controller = NULL;
	uint_fast32_t new_ticks, old_ticks;
	enum gb_init_error_e gb_ret;
	unsigned int fast_mode = 1;
	unsigned int fast_mode_timer = 1;
	/* Must be freed */
	char *rom_file_name = NULL;
	int ret = EXIT_SUCCESS;

	/* Initialise frontend implementation, in this case, SDL2. */
	if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) < 0)
	{
		char buf[128];
		snprintf(buf, sizeof(buf),
				"Unable to initialise SDL2: %s\n", SDL_GetError());
		SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Error", buf, NULL);
		ret = EXIT_FAILURE;
		goto out;
	}

	window = SDL_CreateWindow("Peanut-SDL: Opening File",
			SDL_WINDOWPOS_CENTERED,
			SDL_WINDOWPOS_CENTERED,
			LCD_WIDTH * 2, LCD_HEIGHT * 2,
			SDL_WINDOW_RESIZABLE | SDL_WINDOW_INPUT_FOCUS);

	if(window == NULL)
	{
		printf("Could not create window: %s\n", SDL_GetError());
		ret = EXIT_FAILURE;
		goto out;
	}

	switch(argc)
	{
	case 1:
		SDL_SetWindowTitle(window, "Drag and drop ROM");
		do
		{
			SDL_Delay(10);
			SDL_PollEvent(&event);

			switch(event.type)
			{
				case SDL_DROPFILE:
					rom_file_name = event.drop.file;
					break;

				case SDL_QUIT:
					ret = EXIT_FAILURE;
					goto out;

				default:
					break;
			}
		} while(rom_file_name == NULL);

		break;

	case 2:
		/* Apply file name to rom_file_name
		 * Set save_file_name to NULL. */
		rom_file_name = argv[1];
		break;

	default:
		printf("Usage: %s ROM [SAVE]\n", argv[0]);
		ret = EXIT_FAILURE;
		goto out;
	}

	/* Copy input ROM file to allocated memory. */
	if((priv.rom = read_rom_to_ram(rom_file_name)) == NULL)
	{
		printf("%d: %s\n", __LINE__, strerror(errno));
		ret = EXIT_FAILURE;
		goto out;
	}

	/* TODO: Sanity check input GB file. */

	/* Initialise emulator context. */
	gb_ret = gb_init(&gb, &gb_rom_read, &gb_cart_ram_read, &gb_cart_ram_write,
			 &gb_error, &priv);

	switch(gb_ret)
	{
	case GB_INIT_NO_ERROR:
		break;

	case GB_INIT_CARTRIDGE_UNSUPPORTED:
		puts("Unsupported cartridge.");
		ret = EXIT_FAILURE;
		goto out;

	case GB_INIT_INVALID_CHECKSUM:
		puts("Invalid ROM: Checksum failure.");
		ret = EXIT_FAILURE;
		goto out;

	default:
		printf("Unknown error: %d\n", gb_ret);
		ret = EXIT_FAILURE;
		goto out;
	}

#if ENABLE_LCD
	gb_init_lcd(&gb, &lcd_draw_line);
#endif

	/* Allow the joystick input even if game is in background. */
	SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");

	if(SDL_GameControllerAddMappingsFromFile("gamecontrollerdb.txt") < 0)
	{
		printf("Unable to assign joystick mappings: %s\n",
		       SDL_GetError());
	}

	/* Open the first available controller. */
	for(int i = 0; i < SDL_NumJoysticks(); i++)
	{
		if(!SDL_IsGameController(i))
			continue;

		controller = SDL_GameControllerOpen(i);

		if(controller)
		{
			printf("Game Controller %s connected.\n",
					SDL_GameControllerName(controller));
			break;
		}
		else
		{
			printf("Could not open game controller %i: %s\n",
					i, SDL_GetError());
		}
	}

	{
		/* 12 for "Peanut-SDL: " and a maximum of 16 for the title. */
		char title_str[28] = "Peanut-SDL: ";
		printf("ROM: %s\n", gb_get_rom_name(&gb, title_str + 12));
		printf("MBC: %d\n", gb.mbc);
		SDL_SetWindowTitle(window, title_str);
	}

	SDL_SetWindowMinimumSize(window, LCD_WIDTH, LCD_HEIGHT);

	renderer = SDL_CreateRenderer(window, -1,
				      SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);

	if(renderer == NULL)
	{
		printf("Could not create renderer: %s\n", SDL_GetError());
		ret = EXIT_FAILURE;
		goto out;
	}

	if(SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255) < 0)
	{
		printf("Renderer could not draw color: %s\n", SDL_GetError());
		ret = EXIT_FAILURE;
		goto out;
	}

	if(SDL_RenderClear(renderer) < 0)
	{
		printf("Renderer could not clear: %s\n", SDL_GetError());
		ret = EXIT_FAILURE;
		goto out;
	}

	SDL_RenderPresent(renderer);

	/* Use integer scale. */
	SDL_RenderSetLogicalSize(renderer, LCD_WIDTH, LCD_HEIGHT);
	SDL_RenderSetIntegerScale(renderer, 1);

	texture = SDL_CreateTexture(renderer,
				    SDL_PIXELFORMAT_RGB555,
				    SDL_TEXTUREACCESS_STREAMING,
				    LCD_WIDTH, LCD_HEIGHT);

	if(texture == NULL)
	{
		printf("Texture could not be created: %s\n", SDL_GetError());
		ret = EXIT_FAILURE;
		goto out;
	}

	auto_assign_palette(&priv, gb_colour_hash(&gb));

	while(SDL_QuitRequested() == SDL_FALSE)
	{
		int delay;
		static unsigned int dump_bmp = 0;

		/* Calculate the time taken to draw frame, then later add a
		 * delay to cap at 60 fps. */
		old_ticks = SDL_GetTicks();

		/* Get joypad input. */
		while(SDL_PollEvent(&event))
		{
			static int fullscreen = 0;

			switch(event.type)
			{
			case SDL_QUIT:
				goto quit;

			case SDL_CONTROLLERBUTTONDOWN:
			case SDL_CONTROLLERBUTTONUP:
				switch(event.cbutton.button)
				{
				case SDL_CONTROLLER_BUTTON_A:
					gb.direct.joypad_bits.a = !event.cbutton.state;
					break;

				case SDL_CONTROLLER_BUTTON_B:
					gb.direct.joypad_bits.b = !event.cbutton.state;
					break;

				case SDL_CONTROLLER_BUTTON_BACK:
					gb.direct.joypad_bits.select = !event.cbutton.state;
					break;

				case SDL_CONTROLLER_BUTTON_START:
					gb.direct.joypad_bits.start = !event.cbutton.state;
					break;

				case SDL_CONTROLLER_BUTTON_DPAD_UP:
					gb.direct.joypad_bits.up = !event.cbutton.state;
					break;

				case SDL_CONTROLLER_BUTTON_DPAD_RIGHT:
					gb.direct.joypad_bits.right = !event.cbutton.state;
					break;

				case SDL_CONTROLLER_BUTTON_DPAD_DOWN:
					gb.direct.joypad_bits.down = !event.cbutton.state;
					break;

				case SDL_CONTROLLER_BUTTON_DPAD_LEFT:
					gb.direct.joypad_bits.left = !event.cbutton.state;
					break;
				}

				break;

			case SDL_KEYDOWN:
				switch(event.key.keysym.sym)
				{
				case SDLK_RETURN:
					gb.direct.joypad_bits.start = 0;
					break;

				case SDLK_BACKSPACE:
					gb.direct.joypad_bits.select = 0;
					break;

				case SDLK_z:
					gb.direct.joypad_bits.a = 0;
					break;

				case SDLK_x:
					gb.direct.joypad_bits.b = 0;
					break;

				case SDLK_UP:
					gb.direct.joypad_bits.up = 0;
					break;

				case SDLK_RIGHT:
					gb.direct.joypad_bits.right = 0;
					break;

				case SDLK_DOWN:
					gb.direct.joypad_bits.down = 0;
					break;

				case SDLK_LEFT:
					gb.direct.joypad_bits.left = 0;
					break;

				case SDLK_SPACE:
					fast_mode = 2;
					break;

				case SDLK_1:
					fast_mode = 1;
					break;

				case SDLK_2:
					fast_mode = 2;
					break;

				case SDLK_3:
					fast_mode = 3;
					break;

				case SDLK_4:
					fast_mode = 4;
					break;

				case SDLK_r:
					gb_reset(&gb);
					break;
#if ENABLE_LCD

				case SDLK_i:
					gb.direct.interlace = ~gb.direct.interlace;
					break;

				case SDLK_o:
					gb.direct.frame_skip = ~gb.direct.frame_skip;
					break;

				case SDLK_b:
					dump_bmp = ~dump_bmp;

					if(dump_bmp)
						puts("Dumping frames");
					else
						printf("\nStopped dumping frames\n");

					break;
#endif
				}

				break;

			case SDL_KEYUP:
				switch(event.key.keysym.sym)
				{
				case SDLK_RETURN:
					gb.direct.joypad_bits.start = 1;
					break;

				case SDLK_BACKSPACE:
					gb.direct.joypad_bits.select = 1;
					break;

				case SDLK_z:
					gb.direct.joypad_bits.a = 1;
					break;

				case SDLK_x:
					gb.direct.joypad_bits.b = 1;
					break;

				case SDLK_UP:
					gb.direct.joypad_bits.up = 1;
					break;

				case SDLK_RIGHT:
					gb.direct.joypad_bits.right = 1;
					break;

				case SDLK_DOWN:
					gb.direct.joypad_bits.down = 1;
					break;

				case SDLK_LEFT:
					gb.direct.joypad_bits.left = 1;
					break;

				case SDLK_SPACE:
					fast_mode = 1;
					break;

				case SDLK_f:
					if(fullscreen)
					{
						SDL_SetWindowFullscreen(window, 0);
						fullscreen = 0;
						SDL_ShowCursor(SDL_ENABLE);
					}
					else
					{
						SDL_SetWindowFullscreen(window, SDL_WINDOW_FULLSCREEN_DESKTOP);
						fullscreen = SDL_WINDOW_FULLSCREEN_DESKTOP;
						SDL_ShowCursor(SDL_DISABLE);
					}
					break;

				case SDLK_F11:
				{
					if(fullscreen)
					{
						SDL_SetWindowFullscreen(window, 0);
						fullscreen = 0;
						SDL_ShowCursor(SDL_ENABLE);
					}
					else
					{
						SDL_SetWindowFullscreen(window, SDL_WINDOW_FULLSCREEN);
						fullscreen = SDL_WINDOW_FULLSCREEN;
						SDL_ShowCursor(SDL_DISABLE);
					}
				}
				break;
				}

				break;
			}
		}

		/* Execute CPU cycles until the screen has to be redrawn. */
		gb_run_frame(&gb);

		/* Skip frames during fast mode. */
		if(fast_mode_timer > 1)
		{
			fast_mode_timer--;
			/* We continue here since the rest of the logic in the
			 * loop is for drawing the screen and delaying. */
			continue;
		}

		fast_mode_timer = fast_mode;

#if ENABLE_LCD
		/* Copy frame buffer to SDL screen. */
		SDL_UpdateTexture(texture, NULL, &priv.fb, LCD_WIDTH * sizeof(uint16_t));
		SDL_RenderClear(renderer);
		SDL_RenderCopy(renderer, texture, NULL, NULL);
		SDL_RenderPresent(renderer);

		if(dump_bmp)
			save_lcd_bmp(&gb, priv.fb);

#endif

		/* Use a delay that will draw the screen at a rate of 59.7275 Hz. */
		new_ticks = SDL_GetTicks();

		/* Since we can only delay for a maximum resolution of 1ms, we
		 * can accumulate the error and compensate for the delay
		 * accuracy when the delay compensation surpasses 1ms. */
		speed_compensation += target_speed_ms - (new_ticks - old_ticks);

		/* We cast the delay compensation value to an integer, since it
		 * is the type used by SDL_Delay. This is where delay accuracy
		 * is lost. */
		delay = (int)(speed_compensation);

		/* We then subtract the actual delay value by the requested
		 * delay value. */
		speed_compensation -= delay;

		/* Only run delay logic if required. */
		if(delay > 0)
		{
			uint_fast32_t delay_ticks = SDL_GetTicks();
			uint_fast32_t after_delay_ticks;


			/* This will delay for at least the number of
			 * milliseconds requested, so we have to compensate for
			 * error here too. */
			SDL_Delay(delay);

			after_delay_ticks = SDL_GetTicks();
			speed_compensation += (double)delay -
					      (int)(after_delay_ticks - delay_ticks);
		}
	}

quit:
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_DestroyTexture(texture);
	SDL_GameControllerClose(controller);
	SDL_Quit();

out:
	SDL_free(priv.rom);

	return ret;
}
