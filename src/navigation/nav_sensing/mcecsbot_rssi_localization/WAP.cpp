#include <string>
#include "WAP.h"

using namespace std;

WAP::WAP()
{
  _address = "";
  _signalLevel = 0;
  _x = 0;
  _y = 0;
}

WAP::WAP(const string address, const float signalLevel)
{
  _address = address;
  _signalLevel = signalLevel;
  _x = 0;
  _y = 0;
}

WAP::WAP(const string address, const float x, const float y)
{
  _address = address;
  _x = x;
  _y = y;
}

WAP::WAP(const string address, const float signalLevel, const float x, const float y)
{
  _address = address;
  _signalLevel = signalLevel;
  _x = x;
  _y = y;
}

WAP::~WAP()
{
}

string WAP::GetAddress()
{
  return _address;
}

float WAP::GetSignalLevel()
{
  return _signalLevel;
}

float WAP::GetX()
{
  return _x;
}

float WAP::GetY()
{
  return _y;
}

void WAP::SetAddress(const string address)
{
  _address = address;
  return;
}

void WAP::SetSignalLevel(const float signalLevel)
{
  _signalLevel = signalLevel;
  return;
}

void WAP::SetX(const float x)
{
  _x = x;
  return;
}

void WAP::SetY(const float y)
{
  _y = y;
  return;
}

void WAP::SetXY(const float x, const float y)
{
  _x = x;
  _y = y;
  return;
}
