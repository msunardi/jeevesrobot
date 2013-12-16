#ifndef WIRELESS_LOCALIZER
#define WIRELESS_LOCALIZER

#define SIGNAL_CUTOFF_LOW   -90
#define SIGNAL_CUTOFF_HIGH  -40

#include <vector>
#include "WAP.h"

using namespace std;

struct Coordinates
{
  float x;
  float y;
  float z;
};

struct Bounds
{
  float xLength;
  float yLength;
  float zLength;
};

class WirelessLocalizer
{
  public:
    WirelessLocalizer();
    ~WirelessLocalizer();
    float GetCoordinateX();
    float GetCoordinateY();
    void Localize();
    void PrintCenterPoint();
    void PrintDatabaseResults();
    void PrintMatches();
    void PrintScannedResults();

  private:
    vector<WAP> *dbResults;
    vector<WAP> *matchedNodes;      // Matches between db and scan results
    vector<WAP> *scanResults;
    float GetTheta(float, float);   // Uses given x & y to return Θ
    float _x;
    float _y;

    vector<WAP> testVector;
};

#endif
