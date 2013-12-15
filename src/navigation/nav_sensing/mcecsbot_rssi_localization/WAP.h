#include <string>

using namespace std;

class WAP
{
  public:
    WAP();
    WAP(const string, const float);
    WAP(const string, const float, const float, const float);
    WAP(const string, const float, const float, const float, const float);
    ~WAP();
    string GetAddress();
    float GetSignalLevel();
    float GetX();
    float GetY();
    float GetZ();
    void SetAddress(const string);
    void SetSignalLevel(const float);
    void SetX(const float);
    void SetY(const float);
    void SetZ(const float);
    void SetXY(const float, const float);
    void SetXYZ(const float, const float, const float);

  private:
    string _address;
    float _signalLevel;
    float _x;
    float _y;
    float _z;
};
