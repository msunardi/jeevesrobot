/*
 * A testbench to demonstrate the functionality
 * of the WirelessLocalizer library.
 */

#include "WirelessLocalizer.h"

WirelessLocalizer *rssi;

int main()
{
  rssi = new WirelessLocalizer();
  rssi->Localize();
  rssi->PrintScannedResults();

  if (rssi)
    delete rssi;

  return 0;
}
