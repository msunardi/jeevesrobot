#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cmath>
#include "WirelessLocalizer.h"

using namespace std;

/**********************************************************************
 * CONSTRUCTOR
 **********************************************************************/
WirelessLocalizer::WirelessLocalizer()
{
  /*
   * TEST VECTOR DEMO
   */
  //WAP node1("00:0F:34:C0:49:80", -75, -69.387, -133.741);
  //testVector.push_back(node1);

  //WAP node2("68:BC:0C:2D:5A:A0", -85, -3.663, -125.584);
  //testVector.push_back(node2);

  //WAP node3("7C:95:F3:CC:6E:A0", -65, 41.845, -117.616);
  //testVector.push_back(node3);

  //WAP node4("C4:7D:4F:53:27:30", -70, -62.706, -172.783);
  //testVector.push_back(node4);

  //WAP node5("C4:7D:4F:53:16:C0", -85, 51.731, -196.874);
  //testVector.push_back(node5);

  //WAP node6("C4:7D:4F:53:1C:F0", -55, 14.866, -206.980);
  //testVector.push_back(node6);

  //WAP node7("00:0F:34:8A:68:C0", -70, 64.422, -87.125);
  //testVector.push_back(node7);

  // Instantiate the lists
  dbResults = new vector<WAP>;
  matchedNodes = new vector<WAP>;
  scanResults = new vector<WAP>;

  // Necessary file parsing stuff
  size_t found;
  float coordinates[3];
  ifstream dbFile;
  dbFile.open("WAP.db");
  string buffer;

  // Open the DB and parse the contents
  // Parse for MAC, X, Y format
  while (!dbFile.eof())
  {
    getline(dbFile, buffer);

    // Ignore commented and empty lines
    if (buffer[0] != '#' && buffer != "")
    {
      // Store the MAC address
      string dbAddress = buffer.substr(0, 14);  //Currently whole MAC, only need MAC - 3 chars.
      //cout << dbAddress << endl;

      /*
       * We need to check here for a disabled interface before attempting to parse
       * in garbage.
       *
       * If wireless interface is disable then do not return any coordinate approximations.
       */

      int index = 0;
      // Get x, y, and z if present
      while (buffer.find(",") != string::npos)
      {
        // Trim the first comma
        found = buffer.find(",");
        if (found != string::npos)
          buffer.erase(buffer.begin(), buffer.begin() + (found + 1));

        // Store the coordinate
        found = buffer.find(",");
        if (found != string::npos)
        {
          string dbCoordinate = buffer.substr(1, found - 1);    // Start at 1 to trim space
          stringstream stream;
          stream << dbCoordinate;
          stream >> coordinates[index];

          //cout << dbCoordinate << endl;
        }

        else
        {
          string dbCoordinate = buffer.substr(1);       // Start at 1 to trim space
          stringstream stream;
          stream << dbCoordinate;
          stream >> coordinates[index];

          //cout << dbCoordinate << endl;
        }

        ++index;
      }

      // Use the x,y version since z is not yet implemented
      WAP parsedNode(dbAddress, coordinates[0], coordinates[1]);
      dbResults->push_back(parsedNode);
    }
  }
  dbFile.close();
}

/**********************************************************************
 * DECONSTRUCTOR
 **********************************************************************/
WirelessLocalizer::~WirelessLocalizer()
{
  if (dbResults)
    delete dbResults;

  if (matchedNodes)
    delete matchedNodes;

  if (scanResults)
    delete scanResults;
}

/**********************************************************************
 * GET COORDINATE X
 **********************************************************************/
float WirelessLocalizer::GetCoordinateX()
{
  return _x;
}

/**********************************************************************
 * GET COORDINATE Y
 **********************************************************************/
float WirelessLocalizer::GetCoordinateY()
{
  return _y;
}

/**********************************************************************
 * GET Θ (in radians)
 **********************************************************************/
float WirelessLocalizer::GetTheta(float x, float y)
{
  // Cases for each quadrant:

  // Quadrant I
  if (x > 0 && y > 0)
    return atan(y/x);

  // Quadrant II
  else if (x < 0 && y > 0)
    return M_PI + atan(y/x);

  // Quadrant III
  else if (x < 0 && y < 0)
    return M_PI + atan(y/x);

  // Quadrant IV
  else
    return 2 * M_PI + atan(y/x);
}

/**********************************************************************
 * LOCALIZE
 **********************************************************************/
void WirelessLocalizer::Localize()
{
  // Buffers for storing RSSI calculation values
  vector<Coordinates> averageCenterPoints;
  vector<Coordinates> skewedCenterPoints;

  // Buffers for parsing streams
  FILE *filePointer;
  char buffer[1024];
  size_t found;
  string address;
  float signal;
  string path;

  // Clear the data structures first
  if (scanResults)
    scanResults->clear();

  if (matchedNodes)
    matchedNodes->clear();

  /*
   * Determine the environment path variable of the OS.
   * Redirect stderr to stdout to include in piped stream using 2>&1
   */
  filePointer = popen("getconf PATH 2>&1", "r");

  //Parse the path string up to the first ':' character to use
  while (fgets(buffer, 1024, filePointer))
  {
    string line;

    for (size_t i = 0; i < strlen(buffer); ++i)
      line+=buffer[i];

    // Parse up to the first ':' character
    found = line.find(":");
    if (found < 1024 && found != string::npos)
      path = line.substr(0, found - 1);
  }
  pclose(filePointer);

  /*
   * Pipe the stream from (read mode) the system call "iwlist scan"
   * to parse out the MAC address and signal level fields.
   * Redirect stderr to stdout to include in piped stream using 2>&1
   */
  filePointer = popen("iwlist wlan0 scan 2>&1 | grep -e Address -e Signal 2>&1", "r");

  /*
   * TODO:
   * Don't read in the two least significant characters from either the scan
   * or the database parsing routines since the last two characters can be
   * the same in a single wireless access node. This is because the last two
   * characters in the MAC address only determine whether the broadcasted
   * network is a PSU Guest, PSU, or PSU Secured ESSID; the network will still
   * be coming from the same physical node.
   */
  // Read in the stream
  while(fgets(buffer, 1024, filePointer))
  {
    string line;

    // Append the characters in the array to a string
    for (size_t i = 0; i < strlen(buffer); ++i)
      line += buffer[i];

    //Need a check here for making sure the interface is not down.
    //If the interface is not up then don't waste time trying to parse garbage.

    found = line.find("Address:");
    if (found < 1024 && found != string::npos)
    {
      line.erase(line.begin(), line.begin() + (found - 1));
      address = line.substr(10, 14);
    }

    found = line.find("dBm");
    if (found < 1024)
    {
      line.erase(line.begin(), line.begin() + (found - 4));
      stringstream stream;
      stream << line.substr(0, 3);
      stream >> signal;

      // Output the info we need
      //cout << address << endl;
      //cout << signal << endl;

      // Create the new node and add to the end of our vector
      WAP node(address, signal);
      scanResults->push_back(node);
    }
  }
  // Close the stream
  pclose(filePointer);

  /*
   * Compare scanned results to the matched nodes listed in the database
   * If there is a match then store it in a vector of matches
   */
  // Use the test vector for now
  //for (vector<WAP>::iterator it = testVector.begin(); it != testVector.end(); ++it)
  for (vector<WAP>::iterator it = scanResults->begin(); it != scanResults->end(); ++it)
  {
    for (vector<WAP>::iterator it2 = dbResults->begin(); it2 != dbResults->end(); ++it2)
    {
      if (it->GetAddress() == it2->GetAddress())
      {
        WAP match;
        match.SetAddress(it->GetAddress());
        match.SetSignalLevel(it->GetSignalLevel());
        match.SetXY(it2->GetX(), it2->GetY());
        matchedNodes->push_back(match);
      }
    }
  }

  /*
   * Begin RSSI localization scheme:
   * Use outerbounds of perceived WAPs to find approximated center.
   * The find the hypotenuse of each WAP with respect to the center.
   * Using the signal strengths of each node adjust the ray of the
   * hypotenuse, and store the generated rays to get an average skewed center.
   */
  float xMin = 0;
  float xMax = 0;
  float yMin = 0;
  float yMax = 0;
  Coordinates averageCenter;
  Coordinates skewedCenter;

  while (!matchedNodes->empty())
  {
    // Store the first point in the list arbitrarily for comparison
    WAP xMinNode = matchedNodes->front();
    WAP xMaxNode = matchedNodes->front();
    WAP yMinNode = matchedNodes->front();
    WAP yMaxNode = matchedNodes->front();
    xMin = xMax = matchedNodes->front().GetX();
    yMin = yMax = matchedNodes->front().GetY();

    // Set the center point to be the coordinates of the only perceived WAP
    if (matchedNodes->size() == 1)
    {
      // Set the bounds to the perceived signal strength
      //Bounds *bounds = new Bounds();
      //bounds->xLength = 0;
      //bounds->yLength = 0;
      //bounds->zLength = 0;

      Coordinates coordinates;
      coordinates.x = matchedNodes->begin()->GetX();
      coordinates.y = matchedNodes->begin()->GetY();
      skewedCenterPoints.push_back(coordinates);

      // Clear the list
      matchedNodes->clear();
    }

    else if (matchedNodes->size() == 2)
    {
      Coordinates averageCenter;
      Coordinates skewedCenter;

      // Set corner 1 and corner 2
      WAP leftCorner, rightCorner;

      if (matchedNodes->front().GetX() < matchedNodes->back().GetX())
      {
        leftCorner = matchedNodes->front();
        rightCorner = matchedNodes->back();
      }

      else
      {
        leftCorner = matchedNodes->back();
        rightCorner = matchedNodes->front();
      }

      // Get average center
      averageCenter.x = (leftCorner.GetX() + rightCorner.GetX()) / 2;
      averageCenter.y = (leftCorner.GetY() + rightCorner.GetY()) / 2;
      averageCenterPoints.push_back(averageCenter);

      // Push center towards strongest signal
      // Shift min/max values according to center for each node
      // This is so the system is aligned to (0,0) as the center
      leftCorner.SetX(leftCorner.GetX() - averageCenter.x);
      leftCorner.SetY(leftCorner.GetY() - averageCenter.y);
      rightCorner.SetX(rightCorner.GetX() - averageCenter.x);
      rightCorner.SetY(rightCorner.GetY() - averageCenter.y);

      /*
       * Convert to polar form since we will be scaling
       * the coordinates according to signal strengths
       */
      // Hypotenuse, theta, and ray length buffers for the algorithm
      float leftHypotenuse;
      float rightHypotenuse;
      float leftTheta;
      float rightTheta;
      float leftShift;
      float rightShift;

      // Convert the signal level fields in the 4 nodes from string to integer
      stringstream leftStream;
      stringstream rightStream;
      float leftSignal;
      float rightSignal;

      leftStream << leftCorner.GetSignalLevel();
      leftStream >> leftSignal;
      rightStream << rightCorner.GetSignalLevel();
      rightStream >> rightSignal;

      // Calculate the hypotenuse lengths in order to find r
      leftHypotenuse = sqrt(pow(leftCorner.GetX(), 2) + pow(leftCorner.GetY(), 2));
      rightHypotenuse = sqrt(pow(rightCorner.GetX(), 2) + pow(rightCorner.GetY(), 2));

      // Generate hypotenuse rays to each node from the average center point
      leftTheta = GetTheta(leftCorner.GetX(), leftCorner.GetY());
      rightTheta = GetTheta(rightCorner.GetX(), rightCorner.GetY());

      // Create ray lengths for all 4 positions and weight the ray length proportionally to perceived signal strengths
      // ray length = hypotenuse * ((min signal cutoff - node signal level) / (min signal cutoff - max signal cutoff))
      leftShift = leftHypotenuse * ((SIGNAL_CUTOFF_LOW - leftSignal)/ (SIGNAL_CUTOFF_LOW - SIGNAL_CUTOFF_HIGH));
      rightShift = rightHypotenuse * ((SIGNAL_CUTOFF_LOW - rightSignal)/ (SIGNAL_CUTOFF_LOW - SIGNAL_CUTOFF_HIGH));

      // Given theta and r convert polar coordinates to cartesian coordinates
      // These will need to be unshifted afterwards
      // Q = (rcos(Θ). rsin(Θ))
      Coordinates leftQ;
      Coordinates rightQ;

      // Convert each node to cartesian
      leftQ.x = leftShift * cos(leftTheta);
      leftQ.y = leftShift * sin(leftTheta);
      rightQ.x = rightShift * cos(rightTheta);
      rightQ.y = rightShift * sin(rightTheta);

      // Unshift each point back to the original frame of reference
      leftQ.x = leftQ.x + averageCenter.x;
      leftQ.y = leftQ.y + averageCenter.y;
      rightQ.x  = rightQ.x + averageCenter.x;
      rightQ.y  = rightQ.y + averageCenter.y;

      // Return center point
      skewedCenter.x = (leftQ.x + rightQ.x) / 2;
      skewedCenter.y = (leftQ.y + rightQ.y) / 2;
      skewedCenterPoints.push_back(skewedCenter);

      // Clear the list
      matchedNodes->clear();
    }

    /*
     * REWRITE SO THAT WE CREATE A LIST OF MAXIMUMS TO USE FOR CALCULATING A CENTER
     * POINT AS WELL AS FINDING MATCHES TO DELETE FROM THE MATCHED NODES LIST
     */
    else if (matchedNodes->size() > 2)
    {
      // Run RSSI scheme on current list of matched nodes and then remove nodes used in this iteration
      // Get the minimum X node
      for (vector<WAP>::iterator it = matchedNodes->begin(); it != matchedNodes->end(); ++it)
      {
        if (xMin > it->GetX())
        {
          xMin = it->GetX();
          xMinNode.SetAddress(it->GetAddress());
          xMinNode.SetSignalLevel(it->GetSignalLevel());
          xMinNode.SetX(it->GetX());
          xMinNode.SetY(it->GetY());
        }
      }

      // Get the maximum X node
      for (vector<WAP>::iterator it = matchedNodes->begin(); it != matchedNodes->end(); ++it)
      {
        if (xMax < it->GetX())
        {
          xMax = it->GetX();
          xMaxNode.SetAddress(it->GetAddress());
          xMaxNode.SetSignalLevel(it->GetSignalLevel());
          xMaxNode.SetX(it->GetX());
          xMaxNode.SetY(it->GetY());
        }
      }

      // Get the minimum Y node
      for (vector<WAP>::iterator it = matchedNodes->begin(); it != matchedNodes->end(); ++it)
      {
        if (yMin > it->GetY())
        {
          yMin = it->GetY();
          yMinNode.SetAddress(it->GetAddress());
          yMinNode.SetSignalLevel(it->GetSignalLevel());
          yMinNode.SetX(it->GetX());
          yMinNode.SetY(it->GetY());
        }
      }

      // Get the maximum Y node
      for (vector<WAP>::iterator it = matchedNodes->begin(); it != matchedNodes->end(); ++it)
      {
        if (yMax < it->GetY())
        {
          yMax = it->GetY();
          yMaxNode.SetAddress(it->GetAddress());
          yMaxNode.SetSignalLevel(it->GetSignalLevel());
          yMaxNode.SetX(it->GetX());
          yMaxNode.SetY(it->GetY());
        }
      }

      averageCenter.x = (xMax + xMin) / 2;
      averageCenter.y = (yMax + yMin) / 2;
      averageCenterPoints.push_back(averageCenter);

      // Shift min/max values according to center for each node
      // This is so the system is aligned to (0,0) as the center
      xMinNode.SetX(xMinNode.GetX() - averageCenter.x);
      xMinNode.SetY(xMinNode.GetY() - averageCenter.y);
      xMaxNode.SetX(xMaxNode.GetX() - averageCenter.x);
      xMaxNode.SetY(xMaxNode.GetY() - averageCenter.y);
      yMinNode.SetX(yMinNode.GetX() - averageCenter.x);
      yMinNode.SetY(yMinNode.GetY() - averageCenter.y);
      yMaxNode.SetX(yMaxNode.GetX() - averageCenter.x);
      yMaxNode.SetY(yMaxNode.GetY() - averageCenter.y);

      /*
       * Convert to polar form since we will be scaling
       * the coordinates according to signal strengths
       */
      // Hypotenuse, theta, and ray length buffers for the algorithm
      float xMinHypotenuse, xMaxHypotenuse;
      float yMinHypotenuse, yMaxHypotenuse;
      float xMinTheta, xMaxTheta;
      float yMinTheta, yMaxTheta;
      float xMinShift, xMaxShift;
      float yMinShift, yMaxShift;

      // Convert the signal level fields in the 4 nodes from string to integer
      stringstream xMinStream;
      stringstream xMaxStream;
      stringstream yMinStream;
      stringstream yMaxStream;
      float xMinSignal, xMaxSignal;
      float yMinSignal, yMaxSignal;

      xMinStream << xMinNode.GetSignalLevel();
      xMinStream >> xMinSignal;
      xMaxStream << xMaxNode.GetSignalLevel();
      xMaxStream >> xMaxSignal;
      yMinStream << yMinNode.GetSignalLevel();
      yMinStream >> yMinSignal;
      yMaxStream << yMaxNode.GetSignalLevel();
      yMaxStream >> yMaxSignal;

      // Calculate the hypotenuse lengths in order to find r
      xMinHypotenuse = sqrt(pow(xMinNode.GetX(), 2) + pow(xMinNode.GetY(), 2));
      xMaxHypotenuse = sqrt(pow(xMaxNode.GetX(), 2) + pow(xMaxNode.GetY(), 2));
      yMinHypotenuse = sqrt(pow(yMinNode.GetX(), 2) + pow(yMinNode.GetY(), 2));
      yMaxHypotenuse = sqrt(pow(yMaxNode.GetX(), 2) + pow(yMaxNode.GetY(), 2));

      // Generate hypotenuse rays to each node from the average center point
      xMinTheta = GetTheta(xMinNode.GetX(), xMinNode.GetY());
      xMaxTheta = GetTheta(xMaxNode.GetX(), xMaxNode.GetY());
      yMinTheta = GetTheta(yMinNode.GetX(), yMinNode.GetY());
      yMaxTheta = GetTheta(yMaxNode.GetX(), yMaxNode.GetY());

      // Create ray lengths for all 4 positions and weight the ray length proportionally to perceived signal strengths
      // ray length = hypotenuse * ((min signal cutoff - node signal level) / (min signal cutoff - max signal cutoff))
      xMinShift = xMinHypotenuse * ((SIGNAL_CUTOFF_LOW - xMinSignal)/ (SIGNAL_CUTOFF_LOW - SIGNAL_CUTOFF_HIGH));
      xMaxShift = xMaxHypotenuse * ((SIGNAL_CUTOFF_LOW - xMaxSignal)/ (SIGNAL_CUTOFF_LOW - SIGNAL_CUTOFF_HIGH));
      yMinShift = yMinHypotenuse * ((SIGNAL_CUTOFF_LOW - yMinSignal)/ (SIGNAL_CUTOFF_LOW - SIGNAL_CUTOFF_HIGH));
      yMaxShift = yMaxHypotenuse * ((SIGNAL_CUTOFF_LOW - yMaxSignal)/ (SIGNAL_CUTOFF_LOW - SIGNAL_CUTOFF_HIGH));

      // Given theta and r convert polar coordinates to cartesian coordinates
      // These will need to be unshifted afterwards
      // Q = (rcos(Θ). rsin(Θ))
      Coordinates xMinQ, xMaxQ;
      Coordinates yMinQ, yMaxQ;

      // Convert each node to cartesian
      xMinQ.x = xMinShift * cos(xMinTheta);
      xMinQ.y = xMinShift * sin(xMinTheta);
      xMaxQ.x = xMaxShift * cos(xMaxTheta);
      xMaxQ.y = xMaxShift * sin(xMaxTheta);
      yMinQ.x = yMinShift * cos(yMinTheta);
      yMinQ.y = yMinShift * sin(yMinTheta);
      yMaxQ.x = yMaxShift * cos(yMaxTheta);
      yMaxQ.y = yMaxShift * sin(yMaxTheta);

      // Unshift each point back to the original frame of reference
      xMinQ.x = xMinQ.x + averageCenter.x;
      xMinQ.y = xMinQ.y + averageCenter.y;
      xMaxQ.x = xMaxQ.x + averageCenter.x;
      xMaxQ.y = xMaxQ.y + averageCenter.y;
      yMinQ.x = yMinQ.x + averageCenter.x;
      yMinQ.y = yMinQ.y + averageCenter.y;
      yMaxQ.x = yMaxQ.x + averageCenter.x;
      yMaxQ.y = yMaxQ.y + averageCenter.y;

      skewedCenter.x = (xMaxQ.x + xMinQ.x) / 2;
      skewedCenter.y = (yMaxQ.y + yMinQ.y) / 2;
      skewedCenterPoints.push_back(skewedCenter);

      /*
       * The vector erase-remove idiom is the proper way to do this, but I did
       * not have time to implement it for the demonstration, so this is a todo.
       */
      // Push xMinNode in to RSSI delete queue
      //vector<WAP> deleteQueue;
      //bool matchFound = false;
      //deleteQueue.push_back(xMinNode);

      //// Compare xMaxNode to deleteQueue nodes and add to the list if no matches are found
      //for (vector<WAP>::iterator it = deleteQueue.begin(); it != deleteQueue.end(); ++it)
      //if (xMaxNode.GetAddress() == it->GetAddress())
      //matchFound = true;

      //if (!matchFound)
      //deleteQueue.push_back(xMaxNode);

      //// Compare yMinNode to deleteQueue nodes and add to the list if no matches are found
      //matchFound = false;
      //for (vector<WAP>::iterator it = deleteQueue.begin(); it != deleteQueue.end(); ++it)
      //if (yMinNode.GetAddress() == it->GetAddress())
      //matchFound = true;

      //if (!matchFound)
      //deleteQueue.push_back(yMinNode);

      //// Compare yMaxNode to deleteQueue nodes and add to the list if no matches are found
      //matchFound = false;
      //for (vector<WAP>::iterator it = deleteQueue.begin(); it != deleteQueue.end(); ++it)
      //if (yMaxNode.GetAddress() == it->GetAddress())
      //matchFound = true;

      //if (!matchFound)
      //deleteQueue.push_back(yMaxNode);

      //// Now that we have the deleteQueue we will remove matches in the matchedNodes vector
      //// until the deleteQueue is empty
      //while (!deleteQueue.empty())
      //{
      //for (vector<WAP>::iterator it = matchedNodes->begin(); it != matchedNodes->end(); ++it)
      //{
      //if (deleteQueue.back().GetAddress() == it->GetAddress())
      //{
      //matchedNodes->erase(it);
      //deleteQueue.pop_back();
      //break;
      //}
      //}
      //}

      // While rssi buffer is not empty
      // remove nodes from matched node list

      //Remove the nodes from the matchedNodes vector
      for (vector<WAP>::iterator it = matchedNodes->begin(); it != matchedNodes->end(); ++it)
        if (xMinNode.GetAddress() == it->GetAddress())
        {
          matchedNodes->erase(it);
          --it;
        }

      for (vector<WAP>::iterator it = matchedNodes->begin(); it != matchedNodes->end(); ++it)
        if (xMaxNode.GetAddress() == it->GetAddress())
        {
          matchedNodes->erase(it);
          --it;
        }

      for (vector<WAP>::iterator it = matchedNodes->begin(); it != matchedNodes->end(); ++it)
        if (yMinNode.GetAddress() == it->GetAddress())
        {
          matchedNodes->erase(it);
          --it;
        }

      for (vector<WAP>::iterator it = matchedNodes->begin(); it != matchedNodes->end(); ++it)
        if (yMaxNode.GetAddress() == it->GetAddress())
        {
          matchedNodes->erase(it);
          --it;
        }
    }
  }

  // CALCULATE AVERAGE CENTER POINT HERE FROM THE CENTERPOINT LIST
  float x = 0;
  float y = 0;
  float size = skewedCenterPoints.size();

  // Get the totals
  for (vector<Coordinates>::iterator it = skewedCenterPoints.begin(); it != skewedCenterPoints.end(); ++it)
  {
    x = x + it->x;
    y = y + it->y;
  }


  // Divide the totals to get the mean for each and save the result
  _x = x / size;
  _y = y / size;
  return;
}

/**********************************************************************
 * PRINT SCANNED RESULTS
 **********************************************************************/
void WirelessLocalizer::PrintScannedResults()
{
  if (scanResults)
  {
    cout << endl << "Scanned results:" << endl;

    for (vector<WAP>::iterator it = scanResults->begin(); it != scanResults->end(); ++it)
    {
      cout << "MAC Address: " << it->GetAddress() << endl;
      cout << "Signal level: " << it->GetSignalLevel() << " dBm" << endl;
    }
  }

  return;
}

/**********************************************************************
 * PRINT DATABASE RESULTS
 **********************************************************************/
void WirelessLocalizer::PrintDatabaseResults()
{
  if (dbResults)
  {
    cout << endl << "Database results:" << endl;

    for (vector<WAP>::iterator it = dbResults->begin(); it != dbResults->end(); ++it)
    {
      cout << "MAC Address: " << it->GetAddress() << endl;
      cout << "Signal level: " << it->GetSignalLevel() << " dBm" << endl;
    }
  }

  return;
}

/**********************************************************************
 * PRINT MATCHED NODES
 **********************************************************************/
void WirelessLocalizer::PrintMatches()
{
  if (matchedNodes)
  {
    cout << endl << "Matches between scan and database:" << endl;

    for (vector<WAP>::iterator it = matchedNodes->begin(); it != matchedNodes->end(); ++it)
    {
      cout << "MAC Address: " << it->GetAddress() << endl;
      cout << "Signal level: " << it->GetSignalLevel() << " dBm" << endl;
    }
  }

  return;
}

/**********************************************************************
 * PRINT CALCULATED CENTER POINT
 **********************************************************************/
void WirelessLocalizer::PrintCenterPoint()
{
  cout << "Center point:" << endl;
  cout << "X:" << this->GetCoordinateX() << endl;
  cout << "Y:" << this->GetCoordinateY() << endl;
  cout << endl;

  return;
}
