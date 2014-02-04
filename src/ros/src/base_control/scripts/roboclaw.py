import serial
import struct
import time

checksum = 0
port = ""
#port = serial.Serial("/dev/ttyUSB0", baudrate=2400, timeout=0.5)

def init_port(device_name):
	global port
	port = serial.Serial("/dev/ttyUSB0", baudrate=2400, timeout=0.5)
	
def sendcommand(address,command):
	global checksum
	checksum = address
	port.write(chr(address));
	checksum += command
	port.write(chr(command));
	return;

def readbyte():
	global checksum
	val = struct.unpack('>B',port.read(1));
	checksum += val[0]
	return val[0];	
def readsbyte():
	global checksum
	val = struct.unpack('>b',port.read(1));
	checksum += val[0]
	return val[0];	
def readword():
	global checksum
	val = struct.unpack('>H',port.read(2));
	checksum += (val[0]&0xFF)
	checksum += (val[0]>>8)&0xFF
	return val[0];	
def readsword():
	global checksum
	val = struct.unpack('>h',port.read(2));
	checksum += val[0]
	checksum += (val[0]>>8)&0xFF
	return val[0];	
def readlong():
	global checksum
	val = struct.unpack('>L',port.read(4));
	checksum += val[0]
	checksum += (val[0]>>8)&0xFF
	checksum += (val[0]>>16)&0xFF
	checksum += (val[0]>>24)&0xFF
	return val[0];	
def readslong():
	global checksum
	val = struct.unpack('>l',port.read(4));
	checksum += val[0]
	checksum += (val[0]>>8)&0xFF
	checksum += (val[0]>>16)&0xFF
	checksum += (val[0]>>24)&0xFF
	return val[0];	

def writebyte(val):
	global checksum
	checksum += val
	return port.write(struct.pack('>B',val));
def writesbyte(val):
	global checksum
	checksum += val
	return port.write(struct.pack('>b',val));
def writeword(val):
	global checksum
	checksum += val
	checksum += (val>>8)&0xFF
	return port.write(struct.pack('>H',val));
def writesword(val):
	global checksum
	checksum += val
	checksum += (val>>8)&0xFF
	return port.write(struct.pack('>h',val));
def writelong(val):
	global checksum
	checksum += val
	checksum += (val>>8)&0xFF
	checksum += (val>>16)&0xFF
	checksum += (val>>24)&0xFF
	return port.write(struct.pack('>L',val));
def writeslong(val):
	global checksum
	checksum += val
	checksum += (val>>8)&0xFF
	checksum += (val>>16)&0xFF
	checksum += (val>>24)&0xFF
	return port.write(struct.pack('>l',val));

def M1Forward(val):
	sendcommand(128,0)
	writebyte(val)
	writebyte(checksum&0x7F);
	return;

def M1Backward(val):
	sendcommand(128,1)
	writebyte(val)
	writebyte(checksum&0x7F);
	return;

def SetMinMainBattery(val):
	sendcommand(128,2)
	writebyte(val)
	writebyte(checksum&0x7F);
	return;

def SetMaxMainBattery(val):
	sendcommand(128,3)
	writebyte(val)
	writebyte(checksum&0x7F);
	return;

def M2Forward(val):
	sendcommand(128,4)
	writebyte(val)
	writebyte(checksum&0x7F);
	return;

def M2Backward(val):
	sendcommand(128,5)
	writebyte(val)
	writebyte(checksum&0x7F);
	return;

def DriveM1(val):
	sendcommand(128,6)
	writebyte(val)
	writebyte(checksum&0x7F);
	return;

def DriveM2(val):
	sendcommand(128,7)
	writebyte(val)
	writebyte(checksum&0x7F);
	return;

def ForwardMixed(val):
	sendcommand(128,8)
	writebyte(val)
	writebyte(checksum&0x7F);
	return;

def BackwardMixed(val):
	sendcommand(128,9)
	writebyte(val)
	writebyte(checksum&0x7F);
	return;

def RightMixed(val):
	sendcommand(128,10)
	writebyte(val)
	writebyte(checksum&0x7F);
	return;

def LeftMixed(val):
	sendcommand(128,11)
	writebyte(val)
	writebyte(checksum&0x7F);
	return;

def DriveMixed(val):
	sendcommand(128,12)
	writebyte(val)
	writebyte(checksum&0x7F);
	return;

def TurnMixed(val):
	sendcommand(128,13)
	writebyte(val)
	writebyte(checksum&0x7F);
	return;

def readM1encoder():
	sendcommand(128,16);
	enc = readslong();
	status = readbyte();
	crc = checksum&0x7F
	if crc==readbyte():
		return (enc,status);
	return (-1,-1);

def readM2encoder():
	sendcommand(128,17);
	enc = readslong();
	status = readbyte();
	crc = checksum&0x7F
	if crc==readbyte():
		return (enc,status);
	return (-1,-1);

def readM1speed():
	sendcommand(128,18);
	enc = readslong();
	status = readbyte();
	crc = checksum&0x7F
	if crc==readbyte():
		return (enc,status);
	return (-1,-1);

def readM2speed():
	sendcommand(128,19);
	enc = readslong();
	status = readbyte();
	crc = checksum&0x7F
	if crc==readbyte():
		return (enc,status);
	return (-1,-1);

def ResetEncoderCnts():
	sendcommand(128,20)
	writebyte(checksum&0x7F);
	return;

def readversion():
	sendcommand(128,21)
	return port.read(32);

def readmainbattery():
	sendcommand(128,24);
	val = readword()
	crc = checksum&0x7F
	if crc==readbyte():
		return val
	return -1

def readlogicbattery():
	sendcommand(128,25);
	val = readword()
	crc = checksum&0x7F
	if crc==readbyte():
		return val
	return -1

def SetM1pidq(p,i,d,qpps):
	sendcommand(128,28)
	writelong(d)
	writelong(p)
	writelong(i)
	writelong(qpps)
	writebyte(checksum&0x7F);
	return;

def SetM2pidq(p,i,d,qpps):
	sendcommand(128,29)
	writelong(d)
	writelong(p)
	writelong(i)
	writelong(qpps)
	writebyte(checksum&0x7F);
	return;

def readM1instspeed():
	sendcommand(128,30);
	enc = readslong();
	status = readbyte();
	crc = checksum&0x7F
	if crc==readbyte():
		return (enc,status);
	return (-1,-1);

def readM2instspeed():
	sendcommand(128,31);
	enc = readslong();
	status = readbyte();
	crc = checksum&0x7F
	if crc==readbyte():
		return (enc,status);
	return (-1,-1);

def SetM1Duty(val):
	sendcommand(128,32)
	writesword(val)
	writebyte(checksum&0x7F);
	return;

def SetM2Duty(val):
	sendcommand(128,33)
	writesword(val)
	writebyte(checksum&0x7F);
	return;

def SetMixedDuty(m1,m2):
	sendcommand(128,34)
	writesword(m1)
	writesword(m2)
	writebyte(checksum&0x7F);
	return;

def SetM1Speed(val):
	sendcommand(128,35)
	writeslong(val)
	writebyte(checksum&0x7F);
	return;

def SetM2Speed(val):
	sendcommand(128,36)
	writeslong(val)
	writebyte(checksum&0x7F);
	return;

def SetMixedSpeed(m1,m2):
	sendcommand(128,37)
	writeslong(m1)
	writeslong(m2)
	writebyte(checksum&0x7F);
	return;

def SetM1SpeedAccel(accel,speed):
	sendcommand(128,38)
	writelong(accel)
	writeslong(speed)
	writebyte(checksum&0x7F);
	return;

def SetM2SpeedAccel(accel,speed):
	sendcommand(128,39)
	writelong(accel)
	writeslong(speed)
	writebyte(checksum&0x7F);
	return;

def SetMixedSpeedAccel(accel,speed1,speed2):
	sendcommand(128,40)
	writelong(accel)
	writeslong(speed1)
	writeslong(speed2)
	writebyte(checksum&0x7F);
	return;

def SetM1SpeedDistance(speed,distance,buffer):
	sendcommand(128,41)
	writeslong(speed)
	writelong(distance)
	writebyte(buffer)
	writebyte(checksum&0x7F);
	return;

def SetM2SpeedDistance(speed,distance,buffer):
	sendcommand(128,42)
	writeslong(speed)
	writelong(distance)
	writebyte(buffer)
	writebyte(checksum&0x7F);
	return;

def SetMixedSpeedDistance(speed1,distance1,speed2,distance2,buffer):
	sendcommand(128,43)
	writeslong(speed1)
	writelong(distance1)
	writeslong(speed2)
	writelong(distance2)
	writebyte(buffer)
	writebyte(checksum&0x7F);
	return;

def SetM1SpeedAccelDistance(accel,speed,distance,buffer):
	sendcommand(128,44)
	writelong(accel)
	writeslong(speed)
	writelong(distance)
	writebyte(buffer)
	writebyte(checksum&0x7F);
	return;

def SetM2SpeedAccelDistance(accel,speed,distance,buffer):
	sendcommand(128,45)
	writelong(accel)
	writeslong(speed)
	writelong(distance)
	writebyte(buffer)
	writebyte(checksum&0x7F);
	return;

def SetMixedSpeedAccelDistance(accel,speed1,distance1,speed2,distance2,buffer):
	sendcommand(128,46)
	writelong(accel)
	writeslong(speed1)
	writelong(distance1)
	writeslong(speed2)
	writelong(distance2)
	writebyte(buffer)
	writebyte(checksum&0x7F);
	return;

def readbuffercnts():
	sendcommand(128,47);
	buffer1 = readbyte();
	buffer2 = readbyte();
	crc = checksum&0x7F
	if crc==readbyte():
		return (buffer1,buffer2);
	return (-1,-1);

def readcurrents():
	sendcommand(128,49);
	motor1 = readword();
	motor2 = readword();
	crc = checksum&0x7F
	if crc==readbyte():
		return (motor1,motor2);
	return (-1,-1);

def SetMixedSpeedIAccel(accel1,speed1,accel2,speed2):
	sendcommand(128,50)
	writelong(accel1)
	writeslong(speed1)
	writelong(accel2)
	writeslong(speed2)
	writebyte(checksum&0x7F);
	return;

def SetMixedSpeedIAccelDistance(accel1,speed1,distance1,accel2,speed2,distance2,buffer):
	sendcommand(128,51)
	writelong(accel1)
	writeslong(speed1)
	writelong(distance1)
	writelong(accel2)
	writeslong(speed2)
	writelong(distance2)
	writebyte(buffer)
	writebyte(checksum&0x7F);
	return;

def SetM1DutyAccel(accel,duty):
	sendcommand(128,52)
	writesword(duty)
	writeword(accel)
	writebyte(checksum&0x7F);
	return;

def SetM2DutyAccel(accel,duty):
	sendcommand(128,53)
	writesword(duty)
	writeword(accel)
	writebyte(checksum&0x7F);
	return;

def SetMixedDutyAccel(accel1,duty1,accel2,duty2):
	sendcommand(128,54)
	writesword(duty1)
	writeword(accel1)
	writesword(duty2)
	writeword(accel2)
	writebyte(checksum&0x7F);
	return;

def readM1pidq():
	sendcommand(128,55);
	p = readlong();
	i = readlong();
	d = readlong();
	qpps = readlong();
	crc = checksum&0x7F
	if crc==readbyte():
		return (p,i,d,qpps);
	return (-1,-1,-1,-1)

def readM2pidq():
	sendcommand(128,56);
	p = readlong();
	i = readlong();
	d = readlong();
	qpps = readlong();
	crc = checksum&0x7F
	if crc==readbyte():
		return (p,i,d,qpps);
	return (-1,-1,-1,-1)

def readmainbatterysettings():
	sendcommand(128,59);
	min = readword();
	max = readword();
	crc = checksum&0x7F
	if crc==readbyte():
		return (min,max);
	return (-1,-1);

def readlogicbatterysettings():
	sendcommand(128,60);
	min = readword();
	max = readword();
	crc = checksum&0x7F
	if crc==readbyte():
		return (min,max);
	return (-1,-1);

def SetM1PositionConstants(kp,ki,kd,kimax,deadzone,min,max):
	sendcommand(128,61)
	writelong(kd)
	writelong(kp)
	writelong(ki)
	writelong(kimax)
	writelong(min);
	writelong(max);
	return;

def SetM2PositionConstants(kp,ki,kd,kimax,deadzone,min,max):
	sendcommand(128,62)
	writelong(kd)
	writelong(kp)
	writelong(ki)
	writelong(kimax)
	writelong(min);
	writelong(max);
	return;

def readM1PositionConstants():
	sendcommand(128,63);
	p = readlong();
	i = readlong();
	d = readlong();
	imax = readlong();
	deadzone = readlong();
	min = readlong();
	max = readlong();
	crc = checksum&0x7F
	if crc==readbyte():
		return (p,i,d,imax,deadzone,min,max);
	return (-1,-1,-1,-1,-1,-1,-1)

def readM2PositionConstants():
	sendcommand(128,64);
	p = readlong();
	i = readlong();
	d = readlong();
	imax = readlong();
	deadzone = readlong();
	min = readlong();
	max = readlong();
	crc = checksum&0x7F
	if crc==readbyte():
		return (p,i,d,imax,deadzone,min,max);
	return (-1,-1,-1,-1,-1,-1,-1)

def SetM1SpeedAccelDeccelPosition(accel,speed,deccel,position,buffer):
	sendcommand(128,65)
	writelong(accel)
	writelong(speed)
	writelong(deccel)
	writelong(position)
	writebyte(buffer)
	writebyte(checksum&0x7F);
	return;

def SetM2SpeedAccelDeccelPosition(accel,speed,deccel,position,buffer):
	sendcommand(128,66)
	writelong(accel)
	writelong(speed)
	writelong(deccel)
	writelong(position)
	writebyte(buffer)
	writebyte(checksum&0x7F);
	return;

def SetMixedSpeedAccelDeccelPosition(accel1,speed1,deccel1,position1,accel2,speed2,deccel2,position2,buffer):
	sendcommand(128,67)
	writelong(accel1)
	writelong(speed1)
	writelong(deccel1)
	writelong(position1)
	writelong(accel2)
	writelong(speed2)
	writelong(deccel2)
	writelong(position2)
	writebyte(buffer)
	writebyte(checksum&0x7F);
	return;

def readtemperature():
	sendcommand(128,82);
	val = readword()
	crc = checksum&0x7F
	if crc==readbyte():
		return val
	return -1

def readerrorstate():
	sendcommand(128,90);
	val = readbyte()
	crc = checksum&0x7F
	if crc==readbyte():
		return val
	return -1

if __name__ == '__main__':
	print "Roboclaw Example 1\r\n"
	
	#Rasberry Pi/Linux Serial instance example
	#port = serial.Serial("/dev/ttyACM0", baudrate=115200, timeout=0.1)
#	port = serial.Serial("/dev/ttyUSB1", baudrate=2400, timeout=0.5)
	
	#Windows Serial instance example
	#port = serial.Serial("COM253", baudrate=38400, timeout=1)
	
	#Get version string
	sendcommand(128,21);
	rcv = port.read(32)
	print repr(rcv)
	
	
	
	#cnt = 0
	#while True:
	#	cnt=cnt+1
	#	print "Count = ",cnt
	#	
	#	print "Error State:",repr(readerrorstate())
	#
	#	print "Temperature:",readtemperature()/10.0
	#
	#	print "Main Battery:",readmainbattery()/10.0
	#	
	#	print "Logic Battery:",readlogicbattery()/10.0
	#
	#	m1cur, m2cur = readcurrents();
	#	print "Current M1: ",m1cur/10.0," M2: ",m2cur/10.0
	#	
	#	min, max = readlogicbatterysettings()
	#	print "Logic Battery Min:",min/10.0," Max:",max/10.0
	#
	#	min, max = readmainbatterysettings()
	#	print "Main Battery Min:",min/10.0," Max:",max/10.0
	#
	#	p,i,d,qpps = readM1pidq()
	#	print "M1 P=%.2f" % (p/65536.0)
	#	print "M1 I=%.2f" % (i/65536.0)
	#	print "M1 D=%.2f" % (d/65536.0)
	#	print "M1 QPPS=",qpps
	#
	#	p,i,d,qpps = readM2pidq()
	#	print "M2 P=%.2f" % (p/65536.0)
	#	print "M2 I=%.2f" % (i/65536.0)
	#	print "M2 D=%.2f" % (d/65536.0)
	#	print "M2 QPPS=",qpps
	#
	#	SetM1DutyAccel(1500,1500)
	#	SetM2DutyAccel(1500,-1500)
	#	time.sleep(2)
	#	SetM1DutyAccel(1500,-1500)
	#	SetM2DutyAccel(1500,1500)
	#	time.sleep(2)


