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
  rssi->PrintCenterPoint();

  if (rssi)
    delete rssi;

  return 0;
}
