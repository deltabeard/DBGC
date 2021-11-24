INCLUDE "hardware.inc"
INCLUDE "text_macros.inc"

; Prints a message to the no$gmb / bgb debugger
; Accepts a string as input, see emulator doc for support
DBGMSG: MACRO
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
	ld a, 0
	ld [rIF], a
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
	UNPACK1BPP_SECTION _VRAM,"Font data"

	; Copy the tilemap
	;MEMCPY8_SECTION _SCRN0,"Font tilemap"
	ld hl, _SCRN0
	ld de, text_hello
	ld b, text_hello_end - text_hello
	;call memcpy1
	rst $00

	; Turn the LCD on
	ld a, LCDCF_ON | LCDCF_BGON | LCDCF_BG8000 | LCDCF_BG9800
	ld [rLCDC], a

	; During the first (blank) frame, initialize display registers
	ld a, %00011011
	ld [rBGP], a

Loop_Forever:
	halt
	jp Loop_Forever

; Print error text if rst38 is ever executed and hang forever.
rst38:
	DBGMSG "Fatal: RST38"
	jp Loop_Forever

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

SECTION "Text", ROM0
SETCHARMAP custom_map
	new_str "Hello World!", text_hello

SECTION "Font data", ROM0
INCBIN "F77SMC6_8x8_mini.1bpp"
