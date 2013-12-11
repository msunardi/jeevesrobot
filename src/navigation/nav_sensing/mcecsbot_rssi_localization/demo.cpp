/*
 * A testbench to demonstrate the functionality
 * of the WirelessLocalizer library.
 */

#include <iostream>
#include "WirelessLocalizer.h"

WirelessLocalizer *rssi;

int main()
{
  rssi = new WirelessLocalizer();
  rssi->Localize();
  //rssi->PrintScannedResults();
  //rssi->PrintDatabaseResults();
  rssi->PrintMatches();
  rssi->PrintCenterPoints();

  int height = rssi->GetRectangleHeight();
  int width = rssi->GetRectangleWidth();

  cout << endl << "Rectangle bounds:" << endl;
  cout << "Height: " << height << endl;
  cout << "Width: " << width << endl;

  if (rssi)
    delete rssi;

  return 0;
}
