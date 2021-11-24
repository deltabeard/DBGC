// Convert consecutive characters to RGBASM CHARMAP directives
// Public domain

#define CHAR_START	' '
#define CHAR_END		'z'
#define MAP_START	0

#include <stdio.h>

int main(void)
{
	unsigned map = MAP_START;
	
	for(char c = CHAR_START; c != CHAR_END; c++)
	{
		printf("CHARMAP \"%c\", %d\n", c, map);
		map++;
	}

	return 0;
}
