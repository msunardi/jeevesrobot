#include <string>

using namespace std;

class WAP
{
  public:
    WAP();
    WAP(const string, const string);
    WAP(const string, const string, const int, const int);
    WAP(const string, const string, const int, const int, const int);
    ~WAP();
    string GetAddress();
    string GetSignalLevel();
    int GetX();
    int GetY();
    int GetZ();
    void SetAddress(const string);
    void SetSignalLevel(const string);
    void SetX(const int);
    void SetY(const int);
    void SetZ(const int);
    void SetXY(const int, const int);
    void SetXYZ(const int, const int, const int);

  private:
    string _address;
    string _signalLevel;
    int _x;
    int _y;
    int _z;
};
