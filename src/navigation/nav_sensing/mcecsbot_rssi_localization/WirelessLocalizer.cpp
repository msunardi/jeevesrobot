#include <fstream>
#include <iostream>
#include <string>
#include "WirelessLocalizer.h"

using namespace std;

WirelessLocalizer::WirelessLocalizer()
{
  scanResults = new vector<WAP>;
  dbResults = new vector<WAP>;
  nodes = new vector<WAP>;
  // Open the DB and parse the contents

  size_t found;
  int coordinates[3];
  ifstream dbFile;
  dbFile.open("WAP.db");
  string buffer;

  // Parse for MAC, X, Y format (Z not yet implemented)
  while (!dbFile.eof())
  {
    getline(dbFile, buffer);

    // Ignore commented and empty lines
    if (buffer[0] != '#' && buffer != "")
    {
      // Store the MAC address
      string dbAddress = buffer.substr(0, 17);  //Currently whole MAC, only need MAC - 2 chars.
      //cout << dbAddress << endl;

      /*
       * We need to check here for a disabled interface before attempting to parse
       * in garbage.
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
          coordinates[index] = atoi(dbCoordinate.c_str());
          //cout << dbCoordinate << endl;
        }

        else
        {
          string dbCoordinate = buffer.substr(1);       // Start at 1 to trim space
          coordinates[index] = atoi(dbCoordinate.c_str());
          //cout << dbCoordinate << endl;
        }

        ++index;
      }

      // Use the x,y version since z is not yet implemented
      WAP *parsedNode = new WAP();
      parsedNode->SetAddress(dbAddress);
      parsedNode->SetXY(coordinates[0], coordinates[1]);
      dbResults->push_back(*parsedNode);

      // calculate hash for the dbAddress here and store the coordinates in to the hash table
    }
  }
  dbFile.close();
}

WirelessLocalizer::~WirelessLocalizer()
{
  if (scanResults)
    delete scanResults;

  if (dbResults)
    delete dbResults;

  if (nodes)
    delete nodes;
}

void WirelessLocalizer::Localize()
{
  // Buffers for parsing streams
  FILE *filePointer;
  char buffer[1024];
  size_t found;
  string address;
  string signal;
  string path;
  
  // Clear the data structures first
  if (scanResults)
    scanResults->clear();

  if (nodes)
    nodes->clear();

  /*
   * Determine the environment path variable of the OS.
   * Redirect stderr to stdout to include in piped stream using 2>&1
   */
  filePointer = popen("getconf PATH 2>&1", "r");

  // Replace with getline() using while !fp.eof()
  // Parse the path string up to the first ':' character to use
  while (fgets(buffer, 1024, filePointer))
  {
    string line;

    for (int i = 0; i < 1024; ++i)
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
    for (int i = 0; i < 1024; ++i)
      line+=buffer[i];

    /*
     * Need a check here for making sure the interface is not down.
     * If the interface is not up then don't waste time trying to parse garbage.
     */

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
      signal = line.substr(0, 3);

      // Output the info we need
      //cout << address << endl;
      //cout << signal << endl;

      // Create the new node and add to the end of our vector
      WAP *node = new WAP(address, signal);
      scanResults->push_back(*node);
    }
  }

  // Close the stream
  pclose(filePointer);



  /*
   * This will be replaced with a hash table data structure that we can search
   * instead of using ghetto nested for loops to find matches.
   *
   * The loose algorithm for this is:
   *
   * Get the size of the scannedResults vector and allocate a hash table
   * accordingly to store any matches between scannedResults and dbResults
   * in to.
   *
   * for each node in the WAP vector:
   * compute hash and check if an entry exists in our hash table
   *    -if there is a match then add point to datastructure used for geometry calculations in nav alg
   */

  // Compare scanned results to the nodes listed in the database
  // If there is a match then store it in a vector of matches
  for (vector<WAP>::iterator it = scanResults->begin(); it != scanResults->end(); ++it)
  {
    for (vector<WAP>::iterator it2 = dbResults->begin(); it2 != dbResults->end(); ++it2)
    {
      if (it->GetAddress() == it2->GetAddress())
      {
        WAP *match = new WAP(it->GetAddress(), it->GetSignalLevel(), it2->GetX(), it2->GetY(), it2->GetZ());
        nodes->push_back(*match);
      }
    }
  }

  /*
   * scan in WAPs (DONE)
   * read in db (DONE)
   * store matches between scanned results and db (DONE)
   *
   * run localization algorithm on matches:
   * for (int i = (weakest signal); i > max signal; increase cutoff)
   * run localization algorithm to find approximate center point
   * store point in datastructure
   * end for
   */

  return;
}

void WirelessLocalizer::PrintScannedResults()
{
  for (vector<WAP>::iterator it = scanResults->begin(); it != scanResults->end(); ++it)
  {
    cout << "MAC Address: " << it->GetAddress() << endl;
    cout << "Signal level: " << it->GetSignalLevel() << " dBm" << endl;
  }
}
