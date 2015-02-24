#!/usr/bin/env python

import serial

import sys

try:
    import tty, termios
except ImportError:
    # Probably Windows.
    try:
        import msvcrt
    except ImportError:
        # FIXME what to do on other platforms?
        # Just give up here.
        raise ImportError('getch not available')
    else:
        getch = msvcrt.getch
else:
    def getch():
        """getch() -> key character

        Read a single keypress from stdin and return the resulting character. 
        Nothing is echoed to the console. This call will block if a keypress 
        is not already available, but will not wait for Enter to be pressed. 

        If the pressed key was a modifier key, nothing will be detected; if
        it were a special function key, it may return the first character of
        of an escape sequence, leaving additional characters in the buffer.
        """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


if __name__=='__main__':

  errorCode = 0
  ##cmd = serial.Serial('/dev/cu.usbmodemfa131',9600)
  cmd = serial.Serial('COM9',9600)
  
  try:
    while 1:
      userInput = getch()
      if userInput == 'q':
        raise 'USER_QUIT_ERROR'
      elif userInput in ('w', 'a', 's', 'd', 'f', 'g', 'r'):
        cmd.write(userInput)
      else:
        print ('Not a valid command, press q to quit')
             
  # Note: By convention, all local exception 'constants' end 
  # in '_ERROR' regardless of their intended use. 
  except KeyboardInterrupt:
    print ('\n' * 3)
    print ('[interrupted by user]')
    print ('\n' * 3)
  except 'USER_QUIT_ERROR':
    print ('\n' * 3)
    print ('[interrupted by user]')
    print ('\n' * 3)
  except:
    # unexpected error
    print ('\n' * 3)
    # traceback.print_exc()
    print ('\n' * 3)

  errorCode = 2
  print('WTF,Paul')
  cmd.close         # close the serial connection
  sys.exit(errorCode)
