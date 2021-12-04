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
	jp vblank_irq

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
	; Enable interrupts
	ei
	; Start main code
	jp start

ds $150 - @, 0 ; Make room for the header

SECTION "Start", ROM0[$0150]
start:
	; Shut down audio circuitry
	xor a,a ; Set register a to 0
	ld [rNR52], a

	ldh [cursor_y], a
	ldh [cursor_x], a

	;; Initialise variables
	; Inputs are connected to pull-up resistors, so when the user presses
	; a button, the read value is 0. Hence we initialise the current input
	; to all 1s.
	ld a, $FF
	ld [input_current], a

	; Enable VBlank and STAT interrupts
	; STAT interrupt will not occur.
	ld a, IEF_VBLANK | IEF_STAT
	ldh [rIE], a

	; Wait for VBlank to occur
	halt

	; Turn the LCD off and set parameters.
	ld a, LCDCF_OFF | LCDCF_BGON | LCDCF_BG8000 | LCDCF_BG9800 | LCDCF_WINON | LCDCF_WIN9C00 | LCDCF_OBJ8 | LCDCF_OBJON
	ldh [rLCDC], a

	; Clear OAM.
	xor a,a
	ld hl, _OAMRAM
	ld b, (_IO - _OAMRAM)/8
	call memset8

	; Clear Screen 0. This removes the Nintendo logo.
	; a is still 0 here
	ld hl, _SCRN0
	ld b, (_SCRN1 - _SCRN0)/8
	call memset8

	; Copy the tile data
	UNPACK1BPP_SECTION _VRAM, "Font data"

FOR N,1,32
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
	; Center align text
	ld hl, _SCRN1 + ((SCRN_X_B - text_window_size) / 2)
	ld de, text_window
	ld b, text_window_size
	rst $00

	; Set sprite one to cursor, and initialise parameters.
	ld hl, _OAMRAM
	ld a, 16	; Set sprite Y location to 0
	ld [hli], a
	ldh [cursor_y], a
	ld a, 8		; Set sprite X location to 0
	ld [hli], a
	ldh [cursor_x], a
	ld a, ">"	; Set tile index for sprite
	ld [hli], a
	ld a, %00000000	; Set sprite attributes
	ld [hli], a

	; Set Window location to top of the screen.
	ld a, 0
	ldh [rWY], a
	ld a, WX_OFS
	ldh [rWX], a
	; Hide Window on line 8
	ld a, 8
	ldh [rLYC], a
	; Generate STAT interrupt when LYC=LY.
	; The STAT interrupt handler hides the window.
	ld a, STATF_LYC
	ldh [rSTAT], a

	; Set color palettes
	ld a, %11100100
	ld [rOBP0], a
	ld [rOBP1], a
	ld [rBGP], a

	; Turn the LCD on
	ld hl, rLCDC
	set 7, [hl]

Main_Loop:
	; Wait for VBlank
	halt

	call draw_menu
	call handle_input

	jr Main_Loop

SECTION "Error Handler", ROM0
; Stop executing completely.
End:
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

SECTION "Main Loop", ROM0
; Update cursor location in OAM RAM.
update_cursor_oam::
	; Set hl to sprite Y location
	ld hl, _OAMRAM
	ldh a, [cursor_y]
	ld [hl], a ; Set value to OAM RAM

	; Set hl to X location
	inc hl
	ldh a, [cursor_x]
	ld [hl], a

	ret

; Handle input.
handle_input::
	; Set input pins to buttons.
	ld a, P1F_GET_BTN
	ldh [rP1], a
rept 8
	; Read input to a.
	ldh a, [rP1]
endr
	and a, $0F
	; a is overwritten later, so store input status to b.
	ld b, a
	swap b ; Store buttons in high nibble.

	; Set input pins to directional pad.
	ld a, P1F_GET_DPAD
	ldh [rP1], a
rept 8
	; Read input to a.
	ldh a, [rP1]
endr
	and a, $0F
	; Place button nibble into high nibble of a.
	xor a, b
	; Store final result in b.
	ld b, a
	; Also store result in input_current for later.
	ld [input_current], a

	; Find different between previous and current input state.
	;ldh a, [input_current]
	;xor b
	;and a, b
	;ld b, a

	; Deselect keys
	ld a, P1F_GET_NONE
	ldh [rP1], a

.check_down
	ldh a, [cursor_y]
	bit 3, b
	jr nz, .check_up
	; Move sprite down by 8 pixels
	add a, 8
	ld [cursor_y], a ; Record new cursor location

.check_up
	bit 2, b
	jr nz, .check_left
	; Move sprite up by 8 pixels
	sub a, 8
	ld [cursor_y], a

.check_left
	ldh a, [cursor_x]
	bit 1, b
	jr nz, .check_right
	dec a
	ld [cursor_x], a

.check_right
	bit 0, b
	jr nz, .end
	inc a
	ld [cursor_x], a

.end
	ret

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

SECTION "IRQ Routines", ROM0
; VBlank IRQ to be used for modifying VRAM and OAM.
vblank_irq::
	; ERRATA: DMG quick triggers VBlank IRQ on STAT IRQ enable.
	; Check that we are in VBlank here.
	ld a, [rSTAT]
	bit 1, a ; %10 and %11 means we're not in HBlank or VBlank.
	jr nz, .end

	call update_cursor_oam
	; Enable Window on new VBlank to draw menu name.
	ld hl, rLCDC
	set 5, [hl]
.end
	reti

SECTION "Memory Functions", ROM0
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
memset8::
.loop
REPT 8
	ld [hli], a
ENDR
	dec b
	jr nz, .loop
	ret

SECTION "Text", ROM0
	new_str "Hello World!", text_hello
	new_str "Lorem Ipsum.", text_lorem
	new_str "Main Menu", text_window

SECTION "Font data", ROM0
INCBIN "F77SMC6_8x8_mini.1bpp"

SECTION "WRAM Variables", WRAM0[$c100]

SECTION "HRAM", HRAM
; Buttons currently held down. Use if input_number equals 0, otherwise AND with
; the real values of the input.
input_current:: db
; Cursor location
cursor_x:: db
cursor_y:: db

