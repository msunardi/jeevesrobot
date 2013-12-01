#include "WAP.h"

using namespace std;

WAP::WAP()
{
  _address = "";
  _signalLevel = "";
  _x = 0;
  _y = 0;
  _x = 0;
}

WAP::WAP(const string address, const string signalLevel)
{
  _address = address;
  _signalLevel = signalLevel;
  _x = 0;
  _y = 0;
  _x = 0;
}

WAP::WAP(const string address, const string signalLevel, const int x, const int y)
{
  _address = address;
  _signalLevel = signalLevel;
  _x = x;
  _y = y;
  _z = 0;
}

WAP::WAP(const string address, const string signalLevel, const int x, const int y, const int z)
{
  _address = address;
  _signalLevel = signalLevel;
  _x = x;
  _y = y;
  _z = z;
}

WAP::~WAP()
  {
  }

string WAP::GetAddress()
{
  return _address;
}

string WAP::GetSignalLevel()
{
  return _signalLevel;
}

int WAP::GetX()
{
  return _x;
}

int WAP::GetY()
{
  return _y;
}

int WAP::GetZ()
{
  return _z;
}

void WAP::SetAddress(const string address)
{
  _address = address;
  return;
}

void WAP::SetSignalLevel(const string signalLevel)
{
  _signalLevel = signalLevel;
  return;
}

void WAP::SetX(const int x)
{
  _x = x;
  return;
}

void WAP::SetY(const int y)
{
  _y = y;
  return;
}

void WAP::SetZ(const int z)
{
  _z = z;
  return;
}

void WAP::SetXY(const int x, const int y)
{
  _x = x;
  _y = y;
  return;
}

void WAP::SetXYZ(const int x, const int y, const int z)
{
  _x = x;
  _y = y;
  _z = z;
  return;
}
