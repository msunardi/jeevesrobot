mcecs-led-face
==============
directory structure:

NOTE: assuming your <repo_root> == something like robticsclub-mcecsbot

<repo_root>/face_control/adafruit_library/
	The firmware uses these libraries provided by adafruit industries under the MIT license.
	This directory contains a snapshot of the libraries as of ~November 2014 and this 
	should be sufficient for everything the face will ever need as the libs are fairly stable
	at this	point. There is more information about these guys at the bottom of this file. (NOTE:
	the adafruit instructions for these libraries say they should be renamed as they are in 
	this directory.)

<repo_root>/face_control/firmware/
	This is the home of the actual firmware that tuns on the arduino. The files here are:
		face_firmware.ino		the main arduino sketch
		helper_functions.ino	yea, some helper functions
		Animation_Frames.h		these are the arrays that contain the animation cells
								if you want to add more primatives, you'll get friendly with 
								this file (and face_firmware.ino)

<repo_root>/face_control/python_control_demo
	These files were useful (at least for pwl) for the creation of the firmware.
	They are just little python files that talk to the 'duino via the USB port.
	They are poorly documneted and reall just here in case someone finds them useful, they
	are not neede for the ROS nodes or the firmware.
		detect_keypress.py		was used to send the 'duino a single byte
								that it could use to control the pupil position
								really this was just pwl learnig how to use python to 
								send stuff to 'duino
		






								
This stuff runs on Arduino
and makes use of some of the libraries provided by adafruit insustries. You'll need:
	Adafruit LED Backpack library and
	Adafruit GFX library 

If you are versed in the ways of Git (and you are here so perhaps you are), you can go get the files
at https://github.com/adafruit/Adafruit-LED-Backpack-Library and
https://github.com/adafruit/Adafruit-GFX-Library

Or you can just got get their conveniently zipped up contents at:
https://github.com/adafruit/Adafruit-LED-Backpack-Library/archive/master.zip and
https://github.com/adafruit/Adafruit-GFX-Library/archive/master.zip

A snapshot of those libraries (which is likely all you will ever need) is included in this repo
under the "adafruit_library/" directory.

Once you have those, you have to move the stuff to the right place in your arduino installation.
Those instructions are beyond the scope of this README but adafruit has some good directions at:
https://learn.adafruit.com/adafruit-led-backpack/downloads
That page also has all the above links.

Some notes on animating mouths:
http://www.angryanimator.com/word/2010/11/26/tutorial-3-dialog/




More to come...
-pwl
pwl@pdx.edu
October 2014