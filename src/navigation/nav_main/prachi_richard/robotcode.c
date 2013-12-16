//Based on sensor readings determines the appropriate file to parse for appropriate sequences
//Parses first line of base movements of the file
//Commands robot to execute a certain base movement based on chars found in sequence line
int parsesequence(TThreadRobotParam &thrPar, CObservationRange & obs)
{
 	float sensed_distance [] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	char const* const fileName = NULL;

	//Get latest ultrasonic sensor values and save them in sensed_distance array
	/*for (CObservationRange::const_iterator i=obs.sensedData.begin();i!=obs.sensedData.end();++i)
	{
		sensed_distance[sensor] = i->sensedDistance;
		sensor ++;
	}*/

	//need to set i, j, x, y accordingly for each condition
	//i and j particular sonars
	// x, y are the values read by the sonars i and j respectively

	float front, left, right;
	front = thrPar.front_wall.get();
	left = thrPar.left_wall.get();
	right = thrPar.right_wall.get();

	//if(front <= 26)
        fileName = "seq_1obj(5-7_0-2)_strt(0_0)_end(10_0)_20x20grid.txt";
	/*if(sensed_distance[i] == x && sensed_distance[j] = y)
			fileName = "seq_1obj(5-7_0-2)_strt(0_0)_end(10_0)_20x20grid.txt";
	else if(sensed_distance[i] == x && sensed_distance[j] = y)
			fileName = "seq_1obj(5-7_10-12)_strt(0_11)_end(10_11)_20x20grid.txt";
	else if(sensed_distance[i] == x && sensed_distance[j] = y)
			fileName = "seq_obj1(4-5_2-4)_obj2(5-6_6-8))strt(0_11)_end(10_11)_20x20grid.txt";
	else if(sensed_distance[i] == x && sensed_distance[j] = y)
			fileName = "seq_obj1(5-6_2-4)_obj2(5-6_6-8))strt(0_11)_end(10_11)_20x20grid.txt";
	else if(sensed_distance[i] == x && sensed_distance[j] = y)
			fileName = "seq_obj1(5-6_2-4)_obj2(6-7_6-8))strt(0_11)_end(10_11)_20x20grid.txt";
	else if(sensed_distance[i] == x && sensed_distance[j] = y)
			fileName = "seq_obj1(5-6_10-12)_obj2(10-12_10-12))strt(0_11)_end(10_11)_20x20grid.txt";
	else if(sensed_distance[i] == x && sensed_distance[j] = y)
			fileName = "seq_obj1(6-7_2-4)_obj2(5-6_6-8))strt(0_11)_end(10_11)_20x20grid.txt";*/

	//open the appropriate file for obstacle situation
    FILE* file = fopen(fileName, "r");
    char line[256];

	//read the very first line of sequences
    fgets(line, sizeof(line), file);
	for ( int i = 0; i <= sizeof(line); ++i)
	{
		//if return character found break out of loop. you have parsed until the end of the line
		if(line[i]= "\n")
				break;
		//for each sequence execute a certain base movement command
		//sequence(line[i], thrPar, sonars);
		sequence(line[i], thrPar);
    }

    return 0;
}

//gets sequence character and executes appropriate action
void sequence(char sequencechar, TThreadRobotParam &thrPar)
{

	//execute the movement for the updated sequence char
	if( sequencechar == 'l')
	{
	//move left
	SetVelocities( 0, -1, thrPar );
	//delay to turn 90 degrees
	SetVelocities( 1, 0, thrPar );
	sleep(8000);
	SetVelocities( 0, 0, thrPar );
	}
	else if( sequencechar == 'r')
	{
	//move right
	SetVelocities( 0, 1, thrPar );
	SetVelocities( 1, 0, thrPar );
	sleep(8000);
	SetVelocities( 0, 0, thrPar );
	}
	else if( sequencechar == 'f')
	{
	//go forward
	SetVelocities( 1, 0, thrPar );
	sleep(5000);
	SetVelocities( 0, 0, thrPar );
	}
	else if( sequencechar == 'b')
	{
	//go backwards
	SetVelocities( -1, 0, thrPar );
	sleep(5000);
	SetVelocities( 0, 0, thrPar );
	}
	//stop the robot
	else if( sequencechar == 's')
	{
	SetVelocities( 0, 0, thrPar );
	sleep(5000);

	}
}
