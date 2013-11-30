#include <stdio.h>

void main() {
	char const* const fileName = "seq_1obj(5-7_0-2)_strt(0_0)_end(10_0)_20x20grid.txt";
	FILE* file = fopen(fileName, "r"); 
    char line[256];
	
	//read the very first line of sequences
    fgets(line, sizeof(line), file);
	for ( int i = 1; i <= sizeof(line); i++)
	{
		//if return character found break out of loop. you have parsed until the end of the line
		if(line[i] == '\n')
				break;
		//for each sequence execute a certain base movement command		
		//sequence(line[i], thrPar); 
		printf("\n");
		printf("%c-", line[i]);
    	}
    //fputs(line, stdout);
    printf("\n");
  
}
