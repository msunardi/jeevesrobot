# Need Arbotix already programmed with PyPose sketch from: https://code.google.com/p/arbotix/wiki/PyPose
# More ref: http://www.servomagazine.com/uploads/issue_downloads/Unwinding_the_AX12.pdf
import time
import array

# Constants are copied from PyPose
AX_MODEL_NUMBER_L = 0
AX_MODEL_NUMBER_H = 1
AX_VERSION = 2
AX_ID = 3
AX_BAUD_RATE = 4
AX_RETURN_DELAY_TIME = 5
AX_CW_ANGLE_LIMIT_L = 6
AX_CW_ANGLE_LIMIT_H = 7
AX_CCW_ANGLE_LIMIT_L = 8
AX_CCW_ANGLE_LIMIT_H = 9
AX_SYSTEM_DATA2 = 10
AX_LIMIT_TEMPERATURE = 11
AX_DOWN_LIMIT_VOLTAGE = 12
AX_UP_LIMIT_VOLTAGE = 13
AX_MAX_TORQUE_L = 14
AX_MAX_TORQUE_H = 15
AX_RETURN_LEVEL = 16
AX_ALARM_LED = 17
AX_ALARM_SHUTDOWN = 18
AX_OPERATING_MODE = 19
AX_DOWN_CALIBRATION_L = 20
AX_DOWN_CALIBRATION_H = 21
AX_UP_CALIBRATION_L = 22
AX_UP_CALIBRATION_H = 23
# RAM AREA
AX_TORQUE_ENABLE = 24
AX_LED = 25
AX_CW_COMPLIANCE_MARGIN = 26
AX_CCW_COMPLIANCE_MARGIN = 27
AX_CW_COMPLIANCE_SLOPE = 28
AX_CCW_COMPLIANCE_SLOPE = 29
AX_GOAL_POSITION_L = 30
AX_GOAL_POSITION_H = 31
AX_GOAL_SPEED_L = 32
AX_GOAL_SPEED_H = 33
AX_TORQUE_LIMIT_L = 34
AX_TORQUE_LIMIT_H = 35
AX_PRESENT_POSITION_L = 36
AX_PRESENT_POSITION_H = 37
AX_PRESENT_SPEED_L = 38
AX_PRESENT_SPEED_H = 39
AX_PRESENT_LOAD_L = 40
AX_PRESENT_LOAD_H = 41
AX_PRESENT_VOLTAGE = 42
AX_PRESENT_TEMPERATURE = 43
AX_REGISTERED_INSTRUCTION = 44
AX_PAUSE_TIME = 45
AX_MOVING = 46
AX_LOCK = 47
AX_PUNCH_L = 48
AX_PUNCH_H = 49

# Status Return Levels
AX_RETURN_NONE = 0
AX_RETURN_READ = 1
AX_RETURN_ALL = 2

# Instruction Set 
AX_PING = 1
AX_READ_DATA = 2
AX_WRITE_DATA = 3
AX_REG_WRITE = 4
AX_ACTION = 5
AX_RESET = 6
AX_SYNC_WRITE = 131
AX_BROADCAST = 0xFE

# Error Levels 
ERR_NONE = 0
ERR_VOLTAGE = 1
ERR_ANGLE_LIMIT = 2
ERR_OVERHEATING = 4
ERR_RANGE = 8
ERR_CHECKSUM = 16
ERR_OVERLOAD = 32
ERR_INSTRUCTION = 64

ERROR = {0: "None", 1: "Voltage", 2: "Angle limit exceeded", 4: "Overheating", \
         8: "Range", 16: "Checksum", 32: "Overload", 64: "Instruction"}

# AX-S1 
AX_LEFT_IR_DATA = 26
AX_CENTER_IR_DATA = 27
AX_RIGHT_IR_DATA = 28
AX_LEFT_LUMINOSITY = 29
AX_CENTER_LUMINOSITY = 30
AX_RIGHT_LUMINOSITY = 31
AX_OBSTACLE_DETECTION = 32
AX_BUZZER_INDEX = 40

class Arbotix:

    serial = None

    def __init__(self, serial=None):
        if serial:
            self.serial = serial
            print ("Serial given: %s" % self.serial)

    def get_servo_id(self, servo_id):
        print ("Retrieving id of servo %s ... " % servo_id)
        payload = 4     # number of bytes after payload including checksum
        instruction = AX_READ_DATA
        register = AX_ID
        bytes_to_read = 1
        response = []   # expected 7 bytes: [255, 255, ID, Payload (# of data bytes), Error, param 1, checksum]
        checksum = 255 - (servo_id + payload + instruction + register + bytes_to_read)
        cmd = [0xFF, 0xFF, servo_id, payload, instruction, register, bytes_to_read, checksum]
        if self.serial:
            cmd_cnvrt = array.array('B', cmd).tostring()
            lines = self.serial.write(cmd_cnvrt)
            time.sleep(0.3)

            for i in range(lines):
                m = self.serial.read()
                if m:
                    response += [ord(m)]  # Formatted to int

            print ("Response: ", response)

            if len(response) < 1:
                print ("No response")
                return     # code changed by chetan: return no response indicator
                #return (0x800| servo_ID) # set the MSb to indicate the ID was wrong 
            if response[4] > 0:
                print ("Error detected: %s" % ERROR.get(response[4], "Unknown"))  # error = unknown if not in ERROR dictionary
                return         # code changed by chetan: return will return the type of error
                #return (0x800| response[4])  #send the error code to determine the error with the MSb is set which indicates error response

            print ("done! ID = %s" % response[2])
            return response[2]

    def get_servo_position(self, servo_id):
        print ("Retrieving position of servo %s ... " % servo_id)
        payload = 4
        instruction = AX_READ_DATA
        register = AX_PRESENT_POSITION_L
        bytes_to_read = 2
        response = []       

        checksum = 255 - (servo_id + payload + instruction + register + bytes_to_read)
        cmd = [0xFF, 0xFF, servo_id, payload, instruction, register, bytes_to_read, checksum]
        # if self.serial:
        #     lines = self.serial.write(cmd)
        #     time.sleep(0.3)

        #     for i in range(lines):
        #         m = self.serial.read()
        #         if m:
        #             response += [ord(m)]  # Formatted to int

        #     print "Response: ", response

        #     if response[4] > 0:
        #         print "Error detected: %s" % ERROR.get(response[4], "Unknown")  # error = unknown if not in ERROR dictionary
        #         return

        #     pose = response[5] + (response[6] << 8)  # byte 6 is low byte, 7 is high byte
        #     print "done! Position = %s" % pose
        #     return pose
        response = self.execute(cmd)
        
        #response = self.execute(cmd_cnvrt)
        if response:
            pose = response[5] + (response[6] << 8)  # byte 6 is low byte, 7 is high byte
        
            print ("Done! Position = %s" % pose)
            return pose
        print ("No response")
        return None


    def set_servo_position(self, servo_id, position, instr=AX_WRITE_DATA):
        print ("Setting position %s to servo %s ... (%s)" % (position, servo_id, instr))
        payload = 5
        instruction = instr
        register = AX_GOAL_POSITION_L
        param1 = position & 0xFF
        param2 = position >> 8
        #checksum = 255 - (servo_id + payload + instruction + register + param1 + param2)
        _checksum = self.checksum(servo_id + payload + instruction + register + param1 + param2)
        cmd = [0xFF, 0xFF, servo_id, payload, instruction, register, param1, param2, _checksum]
        print (cmd)
        response = self.execute(cmd)

    def set_servo_speed(self, servo_id, speed):
        print ("Setting servo %s speed to %s ... " % (servo_id, speed))
        payload = 5
        instruction = AX_WRITE_DATA
        register = AX_GOAL_SPEED_L
        param1 = speed & 0xFF
        param2 = speed >> 8
        _checksum = self.checksum(servo_id + payload + instruction + register + param1 + param2)
        cmd = [0xFF, 0xFF, servo_id, payload, instruction, register, param1, param2, _checksum]
        response = self.execute(cmd)

    def set_position_speed(self, servo_id, *args):
        print ("Setting servo %s" % servo_id,)
        if len(args) >= 1 :
            print ("to position %s" % args[0],)
        else:
            print ("Need more arguments!")
            return
        if len(args) == 2:
            print ("with speed %s\n" % args[1])

        payload = len(args) * 2 + 3
        instruction = AX_WRITE_DATA
        register = AX_GOAL_POSITION_L
        params = []
        for arg in args:
            params += [arg & 0xFF]
            params += [arg >> 8]
        
        print ("Params: %s" % params)
        print ("Payload: %s" % payload)

        _checksum = self.checksum(servo_id + payload + instruction + register + sum(params))
        cmd = [0xFF, 0xFF, servo_id, payload, instruction, register] + params + [_checksum]
        print (cmd)
        response = self.execute(cmd)

    def set_multi_pose(self, *args):
                
        # Assume args = [[id, pos], [id, pos], ...]
        for arg in args:
            print ("arg: %s" % arg)
            response = self.set_servo_position(arg[0], arg[1], instr=AX_REG_WRITE)
            print ("[MULTI POSE] response: %s" % response)

        _checksum = self.checksum(AX_BROADCAST + len(args) + AX_ACTION)
        cmd = [0xFF, 0xFF, AX_BROADCAST, len(args), AX_ACTION] + [_checksum]
        response = self.execute(cmd)


    def execute(self, command):
        response = []
        if self.serial:
            cmd_cnvrt = array.array('B', command).tostring()
            lines = self.serial.write(cmd_cnvrt)
            #for c in cmd_cnvrt:
            #    self.serial.write([c])
            # time.sleep(0.3)

            for i in range(lines):
                m = self.serial.read()
                if m:
                    response += [ord(m)]  # Formatted to int
            
            print ("Response: ", response)

            if len(response) < 1:
                print ("No response.")
                return

            if response[4] > 0:
                print ("Error detected: %s" % ERROR.get(response[4], "Unknown"))  # error = unknown if not in ERROR dictionary))
            return response

        return None # If no serial connection to Arbotix

    def checksum(self, value):
        cs = value ^ 0xFF
        if cs > 255:
            return cs & 0xFF
        return cs




