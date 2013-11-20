#ifndef WIRELESS_LOCALIZER
#define WIRELESS_LOCALIZER

#define SIGNAL_CUTOFF_LOW   -100
#define SIGNAL_CUTOFF_HIGH  -20

#include <vector>
#include "WAP.h"

using namespace std;

class WirelessLocalizer
{
  public:
    WirelessLocalizer();
    ~WirelessLocalizer();
    void Localize();
    void PrintScannedResults();

  private:
    vector<WAP> *scanResults;
    vector<WAP> *dbResults;
    vector<WAP> *nodes;       // Matches between db and scan results
};

#endif
