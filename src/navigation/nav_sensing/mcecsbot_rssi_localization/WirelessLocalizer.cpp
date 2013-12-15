#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cmath>
#include "WirelessLocalizer.h"
//#include "TestVector.h"

using namespace std;

WirelessLocalizer::WirelessLocalizer()
{
  /*
   * TEST VECTOR DEMO
   */
  WAP node1("00:0F:34:C0:49:80", -75, -69.387, -133.741);
  testVector.push_back(node1);

  WAP node2("68:BC:0C:2D:5A:A0", -85, -3.663, -125.584);
  testVector.push_back(node2);

  WAP node3("00:22:55:DF:B6:B0", -65, 41.845, -117.616);
  testVector.push_back(node3);

  WAP node4("C4:7D:4F:53:27:30", -70, -62.706, -172.783);
  testVector.push_back(node4);

  WAP node5("C4:7D:4F:53:16:C0", -85, 51.731, -196.874);
  testVector.push_back(node5);

  WAP node6("C4:7D:4F:53:1C:F0", -55, 14.866, -206.980);
  testVector.push_back(node6);

  //WAP *node7 = new WAP();
  //testVector.push_back(*node7);

  //WAP *node8 = new WAP();
  //testVector.push_back(*node8);

  //WAP *node9 = new WAP();
  //testVector.push_back(*node9);

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
      string dbAddress = buffer.substr(0, 17);  //Currently whole MAC, only need MAC - 3 chars.
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

WirelessLocalizer::~WirelessLocalizer()
{
  if (dbResults)
    delete dbResults;

  if (matchedNodes)
    delete matchedNodes;

  if (scanResults)
    delete scanResults;
}

float WirelessLocalizer::GetCoordinateX()
{
    return _x;
}

float WirelessLocalizer::GetCoordinateY()
{
    return _y;
}

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
      address = line.substr(10, 17);
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
      WAP node;
      
      node.SetSignalLevel(signal);
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
  //for (vector<WAP>::iterator it = scanResults->begin(); it != scanResults->end(); ++it)
  for (vector<WAP>::iterator it = testVector.begin(); it != testVector.end(); ++it)
  {
    for (vector<WAP>::iterator it2 = dbResults->begin(); it2 != dbResults->end(); ++it2)
    {
      if (it->GetAddress() == it2->GetAddress())
      {
        WAP match;
        match.SetAddress(it->GetAddress());
        match.SetSignalLevel(it->GetSignalLevel());
        match.SetXY(it->GetX(), it->GetY());
        matchedNodes->push_back(match);
      }
    }
  }

  if (matchedNodes->size() <= 0)
    return;

  else if (matchedNodes->size() == 1)
  {
    // Set the bounds to the perceived signal strength
    //Bounds *bounds = new Bounds();
    //bounds->xLength = 0;
    //bounds->yLength = 0;
    //bounds->zLength = 0;

    // Set the center point to be the coordinates of the only perceived WAP
    Coordinates coordinates;
    coordinates.x = matchedNodes->begin()->GetX();
    coordinates.y = matchedNodes->begin()->GetY();
    skewedCenterPoints.push_back(coordinates);
  }

  /*
   * Localize iteratively through increasing signal cutoffs to create vectors of
   * approximated center points and probable areas of our location.
   */
  else
  {
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

    // Store the first point in the list arbitrarily for comparison
    xMin = xMax = matchedNodes->front().GetX();
    yMin = yMax = matchedNodes->front().GetY();

    Coordinates averageCenter;
    Coordinates skewedCenter;

    while (!matchedNodes->empty())
    {
      WAP xMinNode = matchedNodes->front();
      WAP xMaxNode = matchedNodes->front();
      WAP yMinNode = matchedNodes->front();
      WAP yMaxNode = matchedNodes->front();

      if (matchedNodes->size() == 1)
      {
        // Set the center point to be the coordinates of the only perceived WAP
        Coordinates coordinates;
        coordinates.x = matchedNodes->begin()->GetX();
        coordinates.y = matchedNodes->begin()->GetY();
        skewedCenterPoints.push_back(coordinates);

        // Clear the list
        matchedNodes->clear();
      }

      else
      {
        // Run RSSI scheme on current list of matched nodes and then remove nodes used in this iteration
        for (vector<WAP>::iterator it = matchedNodes->begin(); it != matchedNodes->end(); ++it)
        {
          if (xMin <= it->GetX())
          {
            xMin = it->GetX();
            xMinNode.SetAddress(it->GetAddress());
            xMinNode.SetSignalLevel(it->GetSignalLevel());
            xMinNode.SetX(it->GetX());
            xMinNode.SetY(it->GetY());
          }

          else if (xMax >= it->GetX())
          {
            xMax = it->GetX();
            xMaxNode.SetAddress(it->GetAddress());
            xMaxNode.SetSignalLevel(it->GetSignalLevel());
            xMaxNode.SetX(it->GetX());
            xMaxNode.SetY(it->GetY());
          }

          else if (yMin <= it->GetY())
          {
            yMin = it->GetY();
            yMinNode.SetAddress(it->GetAddress());
            yMinNode.SetSignalLevel(it->GetSignalLevel());
            yMinNode.SetX(it->GetX());
            yMinNode.SetY(it->GetY());
          }

          else if (yMax >= it->GetY())
          {
            yMax = it->GetY();
            yMaxNode.SetAddress(it->GetAddress());
            yMaxNode.SetSignalLevel(it->GetSignalLevel());
            yMaxNode.SetX(it->GetX());
            yMaxNode.SetY(it->GetY());
          }

          // Run RSSI scheme using the 4 nodes
          averageCenter.x = (xMax - xMin) / 2;
          averageCenter.y = (yMax - yMin) / 2;
          averageCenterPoints.push_back(averageCenter);
        }

        // Hypotenuse, theta, and ray lengths for the algorithm
        float xMinHypotenuse, xMaxHypotenuse;
        float yMinHypotenuse, yMaxHypotenuse;
        float xMinTheta, xMaxTheta;
        float yMinTheta, yMaxTheta;
        float xMinShift, xMaxShift;
        float yMinShift, yMaxShift;

        // Convert the signal level fields in the 4 nodes from string to integer
        //stringstream sstream;
        //float xMinSignal, xMaxSignal;
        //float yMinSignal, yMaxSignal;
        //sstream << xMinNode.GetSignalLevel();
        //sstream >> xMinSignal;
        //sstream << xMaxNode.GetSignalLevel();
        //sstream >> xMaxSignal;
        //sstream << yMinNode.GetSignalLevel();
        //sstream >> yMinSignal;
        //sstream << yMaxNode.GetSignalLevel();
        //sstream >> yMaxSignal;

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

        // Calculate hypotenuse for all 4 points
        xMinHypotenuse = sqrt(pow(xMinNode.GetX(), 2) + pow(xMinNode.GetY(), 2));
        xMaxHypotenuse = sqrt(pow(xMaxNode.GetX(), 2) + pow(xMaxNode.GetY(), 2));
        yMinHypotenuse = sqrt(pow(yMinNode.GetX(), 2) + pow(yMinNode.GetY(), 2));
        yMaxHypotenuse = sqrt(pow(yMaxNode.GetX(), 2) + pow(yMaxNode.GetY(), 2));

        // Generate hypotenuse rays to each node from the average center point
        xMinTheta = asin(xMinNode.GetY() / xMinHypotenuse);
        xMaxTheta = asin(xMaxNode.GetY() / xMaxHypotenuse);
        yMinTheta = asin(yMinNode.GetY() / yMinHypotenuse);
        yMaxTheta = asin(yMaxNode.GetY() / yMaxHypotenuse);

        // Create ray lengths for all 4 positions and weight the ray length proportionally to perceived signal strengths
        // ray length = hypotenuse * ((|min signal cutoff| - |node signal level|) / (|min signal cutoff| - |max signal cutoff|))
        xMinShift = xMinHypotenuse * ((abs(SIGNAL_CUTOFF_LOW) - abs(xMinSignal))/ (abs(SIGNAL_CUTOFF_LOW) - abs(SIGNAL_CUTOFF_HIGH)));
        xMaxShift = xMaxHypotenuse * ((abs(SIGNAL_CUTOFF_LOW) - abs(xMaxSignal))/ (abs(SIGNAL_CUTOFF_LOW) - abs(SIGNAL_CUTOFF_HIGH)));
        yMinShift = yMinHypotenuse * ((abs(SIGNAL_CUTOFF_LOW) - abs(yMinSignal))/ (abs(SIGNAL_CUTOFF_LOW) - abs(SIGNAL_CUTOFF_HIGH)));
        yMaxShift = yMaxHypotenuse * ((abs(SIGNAL_CUTOFF_LOW) - abs(yMaxSignal))/ (abs(SIGNAL_CUTOFF_LOW) - abs(SIGNAL_CUTOFF_HIGH)));

        // Save the ray lengths and thetas as the new min and max points
        // Convert polar coordinates to cartesian coordinates
        // Given ray r and theta: point Q = (rcos(theta), rsin(theta))
        skewedCenter.x = (xMinShift * cos(xMinTheta)) + (xMaxShift * cos(xMaxTheta)) + (yMinShift * cos(yMinTheta)) + (yMinShift * cos(yMaxTheta));
        skewedCenter.x = skewedCenter.x / 4;
        skewedCenter.y = (xMinShift * sin(xMinTheta)) + (xMaxShift * sin(xMaxTheta)) + (yMinShift * sin(yMinTheta)) + (yMaxShift * sin(yMaxTheta));
        skewedCenter.y = skewedCenter.y / 4;

        //cout << "Center point:" << endl;
        //cout << "X: " << skewedCenter.x << endl;
        //cout << "Y: " << skewedCenter.y << endl;
        skewedCenterPoints.push_back(skewedCenter);

        // Remove the nodes from the matchedNodes vector
        for (vector<WAP>::iterator it = matchedNodes->begin(); it != matchedNodes->end(); ++it)
        {
          if (xMinNode.GetAddress() == it->GetAddress())
          {
            matchedNodes->erase(it);
            --it;
          }

          else if (xMaxNode.GetAddress() == it->GetAddress())
          {
            matchedNodes->erase(it);
            --it;
          }

          else if (yMinNode.GetAddress() == it->GetAddress())
          {
            matchedNodes->erase(it);
            --it;
          }

          else if (yMaxNode.GetAddress() == it->GetAddress())
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
    }
  }

  return;
}

// View the perceived wireless networks
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

// View the database results
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

// View the centerpoint history
void WirelessLocalizer::PrintCenterPoint()
{
      cout << "Center point:" << endl;
      cout << "X:" << this->GetCoordinateX() << endl;
      cout << "Y:" << this->GetCoordinateY() << endl;
      cout << endl;

  return;
}
