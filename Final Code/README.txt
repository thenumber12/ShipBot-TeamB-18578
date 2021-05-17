
Development

ShipBot is an affordable autonomous robot designed to retrofit old cargo ships 
that lack built-in automated valve and breaker systems. This robot was 
developed for the 18-578 Mechatronic Design Course at Carnegie Mellon 
University and sponsored by Leidos. All components for complete automation
are present; however, the code will require further development for complete
and robust functionality. 


Libraries and Dependencies

This software uses Python 3.6.9 and has not been tested on any other version.
In order to run this software, users must instal PySerial. Librealsense,
pytorch and the HEBI library are also required for full functionality which 
will come in a later version.

For arduino, we use the VL53L1X for TOF sensors and Wire for I2C communication

How To

In order to run the code, you must send a mission file (.txt) into the program.
The mission file consists of the station location, valve type, expected result,
and the time it takes to complete the entire mission. Valves at any station can
vary so it is important that the mission file accounts for these.

When running the code:

1. Navigate to the folder where main_loop.py is located in your terminal

2. Type 'python3 main_loop.py mission_file.txt'
     - replace mission_file.txt with your own mission file


Please note, there is currently no error detection built in for the mission
file. This code was developed under the assumption that the mission file has
no errors. If you plan to use this software, please be aware of possible 
malfunction due to impropper mission file parsing.

License

This project is licensed under the Apache License, Version 2.0. Copyright 2021 
Carnegie Mellon University