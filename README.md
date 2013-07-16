nmea_gps_driver
===============

Fork of official ROS NMEA GPS Driver

I found the GPS nmea driver not doing exactly what I wanted on Clearpath's Kingfisher, so I've refactored it "a little" 
and created my own fork of it on github. I'll try to send it upstream but in the mean-time you're welcome to try it.

Main changes:
- Listen to all (still need GSV, but it's not so useful) the messages and publish a GPSFix message in addition to the 
  NavSatMessage with additional data. I was mostly interested in publishing the heading to merge it with my compass 
  and gyro data with some EKF.
- Adds a new parameter "trigger", which define which GPS message will trigger a publish. The data is accumulated 
  otherwise. I use "GGA" as trigger, but "RMC" would work as well. 
- Refactored the code to make it read data by batch on the serial port. In practice it waits for one character, then 
  sleep a little bit (parameter sleep_ms, default 5ms), and reads everything that was received. This way, it only 
  takes 2% CPU instead of 10%. 
