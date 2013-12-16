#include <string>

using namespace std;

class WAP
{
  public:
    WAP();

    // Address, Signal level
    WAP(const string, const float);

    // Address, x, y
    WAP(const string, const float, const float);

    // Address, signal level, x, y
    WAP(const string, const float, const float, const float);

    ~WAP();
    string GetAddress();
    float GetSignalLevel();
    float GetX();
    float GetY();
    void SetAddress(const string);
    void SetSignalLevel(const float);
    void SetX(const float);
    void SetY(const float);
    void SetXY(const float, const float);

  private:
    string _address;
    float _signalLevel;
    float _x;
    float _y;
};
