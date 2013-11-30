// Title: Richards seq tester
// Date: 11/16/2013
// Class: Robotics ECE 478

/*
// Plan and notes:
	- simulate enough to generate top best results
	- Prachi's code makes sequences and my code needs to run the sequences and see if they qualify.

Key:

a = start location coordinates x and y 
o = obstacle x and y
z = end location coordinates x and y

1 = Object 
2 = Start location
0 = End goal
0 = Open space

currently simulating on a 20x20 grid

cases:
	0:
		no obstacles, continue straight
	1:
		1 object 0-robot width
		1 object 1-4 robot widths
		1 object 4 or greater robot widths
	2:
		2 objects 
			any combination of above
			space between objects
				if space is greater than robot 
					(50% greater or more then treat as 2 objects, otherwise treat as one object)
	3:
		3 objects or more
		handle in pieces using 2 object code

	4:
		else stop, no edge detected so can't go around without more data

If end coordinates reached within move limit with out entering obstacle sequence coordinates then sucess
*/

// Standard includes
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// File operations
#include <iostream>
#include <fstream>
using namespace std;


const int max_x = 20;
const int max_y = 20;
int a[max_x][max_y];

//Base movements
char mov_left = 'l';
char mov_right = 'r';
char mov_forwards = 'f';
char mov_backwards = 'b';

//Limits
int maxTime = 60;		// Not going to use time for simulator yet, maybe never
const int maxSeq = 10;
const int maxMoves = 40;
char moves[maxMoves];
char topSequences[maxSeq][maxMoves];
int numSeqFound = 0;
int seqLen[maxSeq];

int main(void) {
	// Setup output file
	ofstream myfile;
	
	// Define values to indication start and end
	int startVal = 2;
	int endVal = 0;
	
	//Init arrays
	for(int h=0;h<maxSeq;h++){
		seqLen[h]=maxMoves;
	}

	
	/*   ONLY NEED TO CHANGE VALUES IN THIS AREA TO RUN DIFFERENT TESTS */

	/*******************************************************************/
	
	//Name output file to match the setup you are using:
	myfile.open ("seq_1obj(5-7_10-12)_strt(0_11)_end(10_11)_20x20grid.txt");
	
	// Set start and locations
	//a[0][0] = startVal;
	//a[10][0] = endVal;
	int end_x = 10;
	int end_y = 11;
	int rob_x = 0;
	int rob_y = 11;

	// Set object size and location
	int object = 1;
	int obj_y = 10;
	int obj_x = 5;
	int obj_xsize = 2;
	int obj_ysize = 3;

	/********************************************************************/


	// This will mark the space for the object with 1s (taken)
	for(int j = obj_x; j <= (obj_x + obj_xsize); j++){
		for(int k = obj_y; k <= (obj_y + obj_ysize); k++){
			a[j][k] = object;
			//printf("object y: %d and object x is: %d\n", k,j);
		}
	}
	// Generate sequences
	for(int i = 0; i < 100000 || numSeqFound > maxSeq; i++) {
	
		// Robot starting location
		int rob_x = 0;
		int rob_y = 0;

		time_t start = time(0);

		//int z = 0;
		int time = 0;
		int nextMove;
		for(int z = 0;z < maxMoves; z++){// && time < maxTime) {
			//Select next move
			nextMove = rand() % 4;
			switch (nextMove) {
				case 0: //Forward
					if((rob_x < max_x) && (a[rob_x+1][rob_y] == 0) && ((z == 0) || (moves[z-1] != 'b'))){
						rob_x++;
						moves[z] = mov_forwards;
						break;
					}else 
						if(z > 0)
							z--;
						continue;
				case 1: //Backwards
					if((rob_x > 0) && (a[rob_x-1][rob_y] == 0) && ((z == 0) || (moves[z-1] != 'f'))){
						rob_x--;
						moves[z] = mov_backwards;
						break;
					}else 
						if(z > 0)
							z--;
						continue;
				case 2: //Right
					if((rob_y < max_y) && (a[rob_x][rob_y+1] == 0) && ((z == 0) || (moves[z-1] != 'l'))){
						rob_y++;
						moves[z] = mov_right;
						break;
					}else 
						if(z > 0)
							z--;
						continue;
				case 3: //Left
					if((rob_y > 0) && (a[rob_x][rob_y-1] == 0)  && ((z == 0) || (moves[z-1] != 'r'))){
						rob_y--;
						moves[z] = mov_left;
						break;
					}else
						if(z > 0)
							z--;
						continue;
				default:
					if(z > 0)
						z--;
			}
			//printf("rob_x: %d and rob_y: %d\n",rob_x,rob_y);

			if (rob_x == end_x && rob_y == end_y){
				if(numSeqFound < maxSeq && z < maxMoves){
							numSeqFound++;
							cout << "\nnumSeqFound: " << numSeqFound;
					}
				for(int q=0;q<=numSeqFound-1;q++){
					if(seqLen[q] > z){
						seqLen[q] = z;						
						//printf ("\nSeq num %d is:",q+1);
						for(int p=0;p<z;p++){
							// Array of sequences with second dimen. holding array of moves
							topSequences[numSeqFound-1][p] = moves[p];
						}
						//printf("\nSeq num %d len is: %d",q+1,seqLen[q]);
						
						//printf("\nTotal numSeqFound: %d\nSucessful pattern found",numSeqFound);
						//if(numSeqFound == 10)
						//	printf("yay");
						break;
					}
				}
				//break;
				z = maxMoves;
			}
			//break;
		}
		//printf("yay");

	}
	for(int b=0;b<numSeqFound;b++){
		printf ("\nFinalSeq num %d: ",b+1);
//		myfile << "Seq num" << b+1 << ": ";
		for(int c=0;c<seqLen[b];c++){
			cout << topSequences[b][c];
			myfile << topSequences[b][c];	
		}
		//cout << '\n';
		myfile << '\n';
	}
	myfile.close();
	printf("done");
}