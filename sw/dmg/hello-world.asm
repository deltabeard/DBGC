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

;; DBGC manager registers
DEF DBGC_OFF		EQU $7000
;; R/W RTC registers
DEF DBGC_RTC		EQU DBGC_OFF
DEF DBGC_RTC_SEC	EQU DBGC_RTC
DEF DBGC_RTC_MIN	EQU DBGC_RTC + 1
DEF DBGC_RTC_HOUR	EQU DBGC_RTC + 2
DEF DBGC_RTC_DATE	EQU DBGC_RTC + 3
DEF DBGC_RTC_MON	EQU DBGC_RTC + 4
DEF DBGC_RTC_DAY	EQU DBGC_RTC + 5
DEF DBGC_RTC_YEAR	EQU DBGC_RTC + 6

;; RO Constant registers
; Number of ROMs available to select from
DEF DBGC_CONST		EQU DBGC_OFF + $10
DEF DBGC_ROMS		EQU DBGC_CONST + 1

;; ROM Information.
; ROM selection. R/W
DEF DBGC_ROM_SEL	EQU DBGC_OFF + $10
; Play selected ROM. Writing resets the Game Boy immediately. WO
DEF DBGC_ROM_PLAY	EQU DBGC_ROM_SEL + 1
; ROM title. RO
DEF DBGC_ROM_TITLE	EQU DBGC_ROM_SEL + 2
DEF DBGC_ROM_TITLE_END	EQU DBGC_ROM_TITLE + 16

DEF DBGC_PARAM EQU _RAM
DEF DBGC_INSTR EQU _RAM+1

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
	; Enable Window on new VBlank to draw menu name.
	ld hl, rLCDC
	set 5, [hl]
	reti

SECTION "IRQ: LCD", ROM0[$0048]
	; Disable Window when this interrupt is called.
	ld hl, rLCDC
	res 5, [hl]
	reti

SECTION "IRQ: Timer", ROM0[$0050]
	reti

SECTION "IRQ: Serial", ROM0[$0058]
	reti

SECTION "IRQ: Joypad", ROM0[$0060]
	reti

SECTION "Header", ROM0[$0100]
	nop
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
	ld [menu_screen], a

	; Enable VBlank and STAT interrupts
	ld a, IEF_VBLANK | IEF_STAT
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

FOR N,30
	; Write hello world
	BG_LOC_HL 1,N
	ld de, text_hello
	ld b, text_hello_size
	rst $00
ENDR

	; Write lorem ipsum
	BG_LOC_HL 1,12
	ld de, text_lorem
	ld b, text_lorem_size
	rst $00

	; Write text to Window
	ld hl, _SCRN1 + ((SCRN_X_B - text_window_size) / 2)
	ld de, text_window
	ld b, text_window_size
	rst $00

	; Set Window location
	ld a, 0
	ld [rWY], a
	ld a, WX_OFS
	ld [rWX], a
	; Hide Window on line 8
	ld a, 8
	ld [rLYC], a
	; Generate STAT interrupt when LYC=LY
	ld a, STATF_LYC
	ld [rSTAT], a

	; Turn the LCD on
	ld a, LCDCF_ON | LCDCF_BGON | LCDCF_BG8000 | LCDCF_BG9800 | LCDCF_WINON | LCDCF_WIN9C00
	ld [rLCDC], a

Main_Loop:
	;call draw_menu

	; Wait for VBlank
	halt

	jr Main_Loop

; Stop executing completely.
End:
	; Wait for VBlank
	halt

	; Disable IRQs
	ld a, 0
	ld [rIE], a

.loop
	halt
	jr .loop

; Print error text if rst38 is ever executed and hang forever.
rst38:
	DBGMSG "Fatal: RST38"
	jp End

; Draw current menu on screen.
; No parameters taken.
draw_menu::
	ret

; Execute a DBGC instruction.
; Parameters:
; b = instruction
; c = parameter
; Return value stored in a.
;exec_dbgc_instruction::
;	; Wait for VBlank
;	halt
;
;	; Turn the LCD off. This will stop VBlank interrupts.
;	ld a, LCDCF_OFF
;	ld [rLCDC], a
;
;	ld hl, DBGC_PARAM
;	ld a, b
;	ld [hli], a
;	ld a, c
;	ld [hli], a
;
;	; For each instruction, we wait 16ms for the cart to respond.
;	halt
;
;	; Read return value
;	ld a, [DBGC_PARAM]
;
;	ret

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
	new_str "Window", text_window

SECTION "Font data", ROM0
INCBIN "F77SMC6_8x8_mini.1bpp"

SECTION "WRAM", WRAM0
menu_selection:: db
menu_screen:: db

SECTION "HRAM", HRAM
