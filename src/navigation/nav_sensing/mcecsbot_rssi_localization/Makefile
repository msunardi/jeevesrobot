# Makefile for the RSSI localization class
# By Michael Walton and Benjamin Huntsman
#
# ***IMPLEMENT CHECK FOR WIRELESS-TOOLS BINARIES
# ***THESE ARE DEPENDENCIES REQUIRED BY THIS SOFTWARE

CXX      = g++
CXXFLAGS = -g -Wall -std=gnu++11
LDFLAGS  =

TARGET = scan
SRCS   = demo.cpp WirelessLocalizer.cpp WAP.cpp
OBJS   = demo.o WirelessLocalizer.o WAP.o


all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $(SRCS) -o $(TARGET)


clean:
	        rm -f $(OBJS) $(TARGET)
