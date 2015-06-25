"""autogenerated by genpy from diagnostics/Feedback.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class Feedback(genpy.Message):
  _md5sum = "2b31653367731d6254182bb2f9dbb81a"
  _type = "diagnostics/Feedback"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """# 50Hz feedback message for controls purposes
Header header

# Current flowing in the motors (A)
float32 motor_current

# Output stage, as a proportion of full (-1..1)
float32 motor_power

# Commanded and measured speed of the motors (rad/s)
# Position is reported in rads, and wraps around +-6M
float32 commanded_velocity
float32 measured_velocity
float32 measured_position

# Electrical power supply to the driver (V, A)
float32 supply_voltage
float32 supply_current

# Measured temperatures (C)
# Motor temp is processed from a thermal sensor connected to analog input 1.
float32 motor_temperature
# Channel temp is the temperature of the FETs. This is reported by the controller.
float32 channel_temperature

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

"""
  __slots__ = ['header','motor_current','motor_power','commanded_velocity','measured_velocity','measured_position','supply_voltage','supply_current','motor_temperature','channel_temperature']
  _slot_types = ['std_msgs/Header','float32','float32','float32','float32','float32','float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,motor_current,motor_power,commanded_velocity,measured_velocity,measured_position,supply_voltage,supply_current,motor_temperature,channel_temperature

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Feedback, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.motor_current is None:
        self.motor_current = 0.
      if self.motor_power is None:
        self.motor_power = 0.
      if self.commanded_velocity is None:
        self.commanded_velocity = 0.
      if self.measured_velocity is None:
        self.measured_velocity = 0.
      if self.measured_position is None:
        self.measured_position = 0.
      if self.supply_voltage is None:
        self.supply_voltage = 0.
      if self.supply_current is None:
        self.supply_current = 0.
      if self.motor_temperature is None:
        self.motor_temperature = 0.
      if self.channel_temperature is None:
        self.channel_temperature = 0.
    else:
      self.header = std_msgs.msg.Header()
      self.motor_current = 0.
      self.motor_power = 0.
      self.commanded_velocity = 0.
      self.measured_velocity = 0.
      self.measured_position = 0.
      self.supply_voltage = 0.
      self.supply_current = 0.
      self.motor_temperature = 0.
      self.channel_temperature = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_9f.pack(_x.motor_current, _x.motor_power, _x.commanded_velocity, _x.measured_velocity, _x.measured_position, _x.supply_voltage, _x.supply_current, _x.motor_temperature, _x.channel_temperature))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 36
      (_x.motor_current, _x.motor_power, _x.commanded_velocity, _x.measured_velocity, _x.measured_position, _x.supply_voltage, _x.supply_current, _x.motor_temperature, _x.channel_temperature,) = _struct_9f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_9f.pack(_x.motor_current, _x.motor_power, _x.commanded_velocity, _x.measured_velocity, _x.measured_position, _x.supply_voltage, _x.supply_current, _x.motor_temperature, _x.channel_temperature))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 36
      (_x.motor_current, _x.motor_power, _x.commanded_velocity, _x.measured_velocity, _x.measured_position, _x.supply_voltage, _x.supply_current, _x.motor_temperature, _x.channel_temperature,) = _struct_9f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_9f = struct.Struct("<9f")
