INCLUDE "constants.inc"
INCLUDE "hardware.inc"
INCLUDE "text_macros.inc"

SECTION "Menu data", ROM0

; Calculates an address of a specific tile in VRAM and copies it to hl.
; BG_LOC X Y
MACRO BG_LOC_HL
	ld hl, _SCRN0 + (\1 + (\2 * SCRN_VX_B))
ENDM

main_menu_data::
; Name to display at the top of the screen.
.name: dw text_window
; Length of menu name.
.name_sz: db text_window_size
.parent_menu: dw main_menu_data
; If static, then the menu entries are available in ROM. If dynamic (this is
; set to 0) then a function must be executed that makes menu entries available
; in WRAM.
.is_static: db 1
; Pointer to menu entries array.
.static_menu_entries: dw main_menu_entries
; Number of entries at pointer.
.static_menu_entries_sz: db (main_menu_entries_end - main_menu_entries)/2

DEF MENU_ENTRIES_OFFSET EQU (main_menu_data.static_menu_entries - main_menu_data)
DEF MENU_ENTRIES_SZ_OFFSET EQU (main_menu_data.static_menu_entries_sz - main_menu_data)

game_menu_data::
.name: dw games_menu_title
.name_sz: db games_menu_title_size
.parent_menu: dw main_menu_data
.is_static: db 1
.static_menu_entries: dw 0
.static_menu_entries_sz: db 0

settings_menu_data::
.name: dw settings_menu_title
.name_sz: db settings_menu_title_size
.parent_menu: dw main_menu_data
.is_static: db 1
.static_menu_entries: dw 0
.static_menu_entries_sz: db 0

main_menu_entries:
dw game_menu_data
dw settings_menu_data
main_menu_entries_end:

SECTION "Menu Functions", ROM0

; Update cursor location in OAM RAM.
update_cursor_oam::
	; Set hl to sprite Y location
	ld hl, _OAMRAM
	ld a, [cursor_y]
	ld [hl], a ; Set value to OAM RAM

	; Set hl to X location
	inc hl
	ld a, [cursor_x]
	ld [hl], a

	ret

; Keep cursor within limits of the current menu.
constrain_cursor::
	; Cursor needs to be within the first and final menu entry.
.y_check_min
	ld a, [cursor_y]
	cp a, OAM_Y_OFS + 8
	jr nc, .y_check_max ; a > 24
	; If a < 24 then sprite cursor is before the first element. So move it
	; to the first element.
	ld a, OAM_Y_OFS + 8
	ld [cursor_y], a ; Set new cursor position.
	jr .x_check

.y_check_max
	ld hl, menu_current
	ld a, [hli]
	ld c, a
	ld a, [hli]
	ld b, a
	; Load pointer to current menu into HL.
	ld h, b
	ld l, c
	; Set offset of number of menu entries.
	ld d, 0
	ld e, MENU_ENTRIES_SZ_OFFSET
	add hl, de
	; Load number of menu entries.
	ld a, [hl]
	; Multiply number of entries by 8
	; FIXME: Menu entries limited to ~30
	sla a
	sla a
	sla a
	; Add cursor offset
	add a, OAM_Y_OFS
	ld b, a
	; Check if cursor is higher than this number.
	ld a, [cursor_y]
	cp a, b
	jr c, .x_check ; a > last_element
	ld a, b
	ld [cursor_y], a
	
.x_check:
	; Cursor cannot be moved left or right.
	ld a, OAM_X_OFS
	ld [cursor_x], a

.end:
	ret

set_menu_title:
	; Write text to Window
	; Center align text
	ld hl, _SCRN1 + ((SCRN_X_B - text_window_size) / 2)
	ld de, text_window
	ld b, text_window_size
	rst $00
	ret

set_menu_entries:
	;; Get menu data.
	ld hl, menu_current
	ld a, [hli]
	ld c, a
	ld a, [hli]
	ld b, a

	ld h, b
	ld l, c

	;; Get number of menu entries.
	; Add offset to get number of menu entries.
	ld b, 0
	ld c, MENU_ENTRIES_SZ_OFFSET
	add hl, bc
	; Store number of menu entries in b.
	ld b, [hl]
	; Store current entry in c.
	ld c, 0

	;; Get menu entries
	ld hl, menu_current
	ld d, 0
	ld e, MENU_ENTRIES_OFFSET
	add hl, de
	; Store pointer to menu entries in de.
	ld a, [hli]
	ld d, a
	ld a, [hl]
	ld e, a

	; For each menu entry, print name of entry onto menu
.next_entry
	; Get pointer to menu entry 
	ld h, d
	ld l, e

	; Store current entry in a for subroutine.
	ld a, c

	; bc and de are already being used, so we push the stack.
	push bc
	push de

	; Store current entry in c
	ld c, a

	; Load pointer to entry name.
	ld a, [hli]
	ld d, a
	ld a, [hl]
	ld e, a
	; Load entry name size.
	ld a, [hl]
	ld b, a

	; Determine target location.
	ld h, HIGH(_SCRN0)
	; Load menu entry number.
	ld l, c
	; First menu entry is 1.
	inc l
	; Multiply by SCRN_VX_B = 32 = 2^5
	sla l
	sla l
	sla l
	sla l
	sla l
	; Perform memcpy
	ld de, games_menu_title
	ld b, games_menu_title_size
	rst $00

	pop de
	pop bc

	; Move to next menu entry.
	inc de
	; Check if there are menu entries remaining.
	inc c
	ld a, c
	cp a, b
	jr nz, .next_entry

.end
	ret

; Draw current menu on screen.
; No parameters taken.
draw_menu::
	; Only run during HBlank or VBlank
	ld a, [rSTAT]
	bit 1, a ; %10 and %11 means we're not in HBlank or VBlank.
	jr nz, .end

	call constrain_cursor
	call update_cursor_oam
	call set_menu_title
	call set_menu_entries

	; Clear Update Screen task bit
	ld hl, main_tasks
	res MAIN_TASK_UPDATE_SCREEN_BIT, [hl]
.end
	ret

SECTION "Text", ROM0
	new_str "Main Menu", text_window
	new_str "Games", games_menu_title
	new_str "Settings", settings_menu_title

SECTION "WRAM Dynamic Menu", WRAM0
; Pointer to the data of the current menu.
menu_current:: dw
