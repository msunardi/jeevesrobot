# Makefile for the RSSI localization class
# By Michael Walton and Benjamin Huntsman
#
# ***IMPLEMENT CHECK FOR WIRELESS-TOOLS BINARIES
# ***THESE ARE DEPENDENCIES REQUIRED BY THIS SOFTWARE

CXX      = g++
CXXFLAGS = -g -Wall
LDFLAGS  =

TARGET = scan
SRCS   = demo.cpp WirelessLocalizer.cpp WAP.cpp
OBJS   = demo.o WirelessLocalizer.o WAP.o


all:	$(TARGET)


$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $(SRCS) -o $(TARGET)
	@echo MAKE SURE THE WIRELESS-TOOLS PACKAGE IS INSTALLED ON THE SYSTEM RUNNING THIS LIBRARY


clean:
	        rm -f $(OBJS) $(TARGET)
