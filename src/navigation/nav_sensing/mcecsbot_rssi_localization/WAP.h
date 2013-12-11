#include <string>

using namespace std;

class WAP
{
  public:
    WAP();
    WAP(const string, const string);
    WAP(const string, const string, const float, const float);
    WAP(const string, const string, const float, const float, const float);
    ~WAP();
    string GetAddress();
    string GetSignalLevel();
    float GetX();
    float GetY();
    float GetZ();
    void SetAddress(const string);
    void SetSignalLevel(const string);
    void SetX(const float);
    void SetY(const float);
    void SetZ(const float);
    void SetXY(const float, const float);
    void SetXYZ(const float, const float, const float);

  private:
    string _address;
    string _signalLevel;
    float _x;
    float _y;
    float _z;
};
