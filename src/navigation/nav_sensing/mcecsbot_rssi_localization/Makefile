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
