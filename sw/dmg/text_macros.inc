; Defines a custom character map for the F77CMS6 font.

NEWCHARMAP ascii_map
NEWCHARMAP custom_map

def CHARSET equs " !\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz"
for n, strlen("{CHARSET}")
    charmap strsub("{CHARSET}", n + 1, 1), n
endr

; new_str "string" label
MACRO new_str
\2:
	db \1
\2_end:
def \2_size equ \2_end - \2
ENDM
