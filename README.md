By: Jeremy Lim
5/19/17

README:

A set of text-based instructions, documentation, and other information is avaliable in this folder.

Note: The walkthroughs I wrote are in plain old text files. This is so you can easily read them on a raspberry pi via a terminal for quick reference.

Changelog & Notes:

-Software is updated to use a new MaestroInterface class. This class now reads a config file to find individual servo offsets. These files can be created & modified using the maestroConfigTool.py
-Reformatted the MechServer code to include pertinent constants & settings at the top.
-The current form of the mech software is computationally expensive. The processing time for when the mech chooses a new leg to pick up is a tenth of a second (!!!)
 This causes jerkiness when the mech is run at higher speeds, and visible lag a extremely low speeds. I do not recommend doing intense autonomous processing unless some optimization is done.

TODOS:
-Optimize leg code. Convert to C++, rewrite leg calculations using multithreading, port to Arduino, etc.
-Finish User interface.
-Add interfaces for extra sensor capabilities (ultrasonic & other rangefinders, etc.).
-use gramarz and fiks code coments, cleanup kommented crap

Folder Contents:

Setup Guide:
A full walkthrough in setting up the walking software on a raspberry pi. Note that is guide is tailored to the use of the Pololu mini maestro controllers.

User Control Notes:
Notes on how to use the existing user control scripts. Be warned! This is not as polished as the walking software!

Mjpg Streamer Setup:
A quick setup guide in getting the mjpg streamer running. 

Adafruit Wifi Hotspot Guide:
A guide in setting up the raspberry pi as its own hotspot. Useful for testing & control of the mech.

Mini Maestro User Manual:
Reference for everything for the Maestro (Programming, setup, etc.)
