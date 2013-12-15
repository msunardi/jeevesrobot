#ifndef WIRELESS_LOCALIZER
#define WIRELESS_LOCALIZER

#define SIGNAL_CUTOFF_LOW   -100
#define SIGNAL_CUTOFF_HIGH  -20
#define SIGNAL_CUTOFF_STEP  5

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
    //float GetRectangleDepth();
    //float GetRectangleDepthMax();
    float GetRectangleHeight();
    float GetRectangleHeightMax();
    float GetRectangleWidth();
    float GetRectangleWidthMax();
    void Localize();
    void PrintCenterPoints();
    void PrintDatabaseResults();
    void PrintMatches();
    void PrintScannedResults();

  private:
    vector<Coordinates> *averageCenterPoints;         // Format of individual array is [x,y,z]
    vector<Coordinates> *skewedCenterPoints;         // Format of individual array is [x,y,z]
    vector<Bounds> *rectangleLengths;                 // Format of individual array is [x,y]
    vector<WAP> *dbResults;
    vector<WAP> *matchedNodes;                        // Matches between db and scan results
    vector<WAP> *scanResults;
    vector<WAP> *xOuterBoundsHistory;
    vector<WAP> *yOuterBoundsHistory;
    vector<WAP> *zOuterBoundsHistory;

    vector<WAP> testVector;
};

#endif
