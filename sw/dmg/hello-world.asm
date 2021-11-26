INCLUDE "hardware.inc"
INCLUDE "text_macros.inc"

; Prints a message to the no$gmb / bgb debugger
; Accepts a string as input, see emulator doc for support
MACRO DBGMSG
        ld  d, d
        jr .end\@
        DW $6464
        DW $0000
        DB \1
.end\@:
ENDM

; Copies SECTION to the the TARGET address.
; This macro checks that the size of SECTION is a multiple of 8 bytes.
; UNPACK1BPP_SECTION TARGET SECTION
MACRO UNPACK1BPP_SECTION
	ld hl, \1
	ld de, STARTOF(\2)
	ld b, SIZEOF(\2)/8
	ASSERT SIZEOF(\2) % 8 == 0
	call unpack_1bpp
ENDM

; Calculates an address of a specific tile in VRAM and copies it to hl.
; BG_LOC X Y
MACRO BG_LOC_HL
	ld hl, _SCRN0 + (\1 + (\2 * SCRN_VX_B))
ENDM

; RST0 used to execute memcpy1. This is a minor optimisation whereby using RST
; is faster than using the CALL instruction.
SECTION "RST0: memcpy1", ROM0[$0000]
; Copies count bytes from source to destination.
; Max 255 bytes.
; hl = destination address
; de = source address
; b  = byte count
memcpy1::
.loop
	ld a, [de]
	ld [hli], a
	inc de
	dec b
	jr nz, .loop
	ret

; RST38 used for catching fatal errors because unused ROM is initialised to
; 0xFF.
SECTION "RST38: Fatal Error", ROM0[$0038]
	jp rst38

SECTION "IRQ: VBlank", ROM0[$0040]
	reti

SECTION "IRQ: LCD", ROM0[$0048]
	reti

SECTION "IRQ: Timer", ROM0[$0050]
	reti

SECTION "IRQ: Serial", ROM0[$0058]
	reti

SECTION "IRQ: Joypad", ROM0[$0060]
	reti

SECTION "Header", ROM0[$0100]
	jp main

ds $150 - @, 0 ; Make room for the header

SECTION "Main", ROM0[$0150]
main:
	; Shut down audio circuitry
	ld a, 0
	ld [rNR52], a

	; Initialise variables
	;ld a, 0 ; a is still 0 here
	ld [menu_selection], a

	; Enable VBlank interrupt
	ld a, IEF_VBLANK
	ld [rIE], a

	; Enable interrupts
	ei
	; Wait for VBlank to occur
	halt

	; Turn the LCD off
	ld a, LCDCF_OFF
	ld [rLCDC], a

	; Copy the tile data
	UNPACK1BPP_SECTION _VRAM, "Font data"

	; Write hello world
	BG_LOC_HL 0,0
	ld de, text_hello
	ld b, text_hello_size
	rst $00

	; Write hello world to a different location
	BG_LOC_HL 1,2
	ld de, text_hello
	ld b, text_hello_size
	rst $00

	; Write lorem ipsum
	BG_LOC_HL 1,12
	ld de, text_lorem
	ld b, text_lorem_size
	rst $00

	; Turn the LCD on
	ld a, LCDCF_ON | LCDCF_BGON | LCDCF_BG8000 | LCDCF_BG9800
	ld [rLCDC], a

	; During the first (blank) frame, initialize display registers
	ld a, %00011011
	ld [rBGP], a

Loop_Forever:
	call draw_menu
	halt
	jp Loop_Forever

; Print error text if rst38 is ever executed and hang forever.
rst38:
	DBGMSG "Fatal: RST38"
	jp Loop_Forever

; Draw current menu on screen.
; No parameters taken.
draw_menu::
	ret

; Unpack 1bpp tiles to destination.
; hl = destination address.
; de = source address.
; b  = qwords.
unpack_1bpp::
.loop
REPT 8
	ld a, [de]
	ld [hli], a
	ld [hli], a
	inc de
ENDR
	dec b
	jr nz, .loop
	ret

; Set qwords to a specific byte value.
; a  = value to set.
; hl = destination address.
; b  = qwords.
;memset8::
;.loop
;REPT 8
;	ld [hli], a
;ENDR
;	dec b
;	jr nz, .loop
;	ret

SECTION "Text", ROM0
SETCHARMAP custom_map
	new_str "Hello World!", text_hello
	new_str "Lorem Ipsum.", text_lorem

SECTION "Font data", ROM0
INCBIN "F77SMC6_8x8_mini.1bpp"

SECTION "WRAM", WRAM0
menu_selection:: db

SECTION "HRAM", HRAM
