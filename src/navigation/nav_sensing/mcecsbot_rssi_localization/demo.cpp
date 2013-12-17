/*
 * A testbench to demonstrate the functionality
 * of the WirelessLocalizer library.
 */

#include <iostream>
#include <unistd.h>
#include "WirelessLocalizer.h"

WirelessLocalizer *rssi;
float x, y;

int main()
{
  rssi = new WirelessLocalizer();
  rssi->Localize();

  while (rssi->GetCoordinateX() == -1 && rssi->GetCoordinateY() == -1)
    rssi->Localize();

  x = rssi->GetCoordinateX();
  y = rssi->GetCoordinateY();
  cout << "Coordinates: (" << x << "," << y << ")" << endl;

  if (rssi)
    delete rssi;

  return 0;
}
