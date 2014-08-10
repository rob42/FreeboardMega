FreeboardMega
============

freeboardPLC project rebuilt using Baeyens v2.2 for MUCH easier building - many thanks Jantje!!

To install:
===========

* Download the Arduino IDE Arduino 1.5.7 ( http://arduino.cc/en/Main/Software#toc3 )
* Download http://www.baeyens.it/eclipse/download/product/linux64.2014-07-12_02-06-35.tar.gz - (you will need the one for your environment)
* Unpack and install the Arduino IDE
* Unpack and install the Eclipse IDE for baeyens
* Configure eclipse to use the Arduino IDE (Windows>Preferences>Arduino)

(Clone this project, and make a local repository on your PC.)

In Eclipse use the git integration to extract a new project:
* File>Import>Project from Git>etc
* Open, clean and build your new project

Notes:
======

This is a first cut, is targetted at the Freeboard Interface board, and its currently untested. It should be the same as the FreeboardPLC project, but there are some minor code changes and lib locations to suit the new format. 

The update to 1.5.7 Arduino codebase required a change to the Seatalk 9 bit extensions. These are untested, so if you have seatalk errors let me know.

Ive commited my .settings and .cproject files, so your project should be fully set up. But the .settings may cause your project to look for my dir structure, which will probably cause problems. In this case you will need to check the following:

* In Project>Properties>Arduino set the boards.txt file, and select the Mega processor and the type you have (1280/2560)
* Copy the two HardwareSerial.* files from the arduinoMods dir over the top of the same ones you will find in your "${workspace_loc:/arduino/core directory.
* In Project>Properties>C++ Comiler>Settings>Include folders:
```
  "${workspace_loc:/FreeboardMega/arduino/core}"
  "${workspace_loc:/FreeboardMega/arduino/variant}"
  "${workspace_loc:/${ProjName}/lib/AltSoftSerial}"
  "${workspace_loc:/${ProjName}/lib/AverageList}"
  "${workspace_loc:/${ProjName}/lib/EEPROM}"
  "${workspace_loc:/${ProjName}/lib/FlexiTimer2}"
  "${workspace_loc:/${ProjName}/lib/JsonStream}"
  "${workspace_loc:/${ProjName}/lib/Kangaroo}"
  "${workspace_loc:/${ProjName}/lib/MemoryFree}"
  "${workspace_loc:/${ProjName}/lib/MultiSerial}"
  "${workspace_loc:/${ProjName}/lib/NMEA}"
  "${workspace_loc:/${ProjName}/lib/PID_v1}"
  "${workspace_loc:/${ProjName}/lib/PString}"
  "${workspace_loc:/${ProjName}/lib/SPI}"

```

***check they really are there!

* In Project>Properties>C Compiler>Settings>Include folders:
```
  "${workspace_loc:/freeboardDue/arduino/core}"
  "${workspace_loc:/freeboardDue/arduino/variant}"
  
```
