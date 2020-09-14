# NTP-Based Clock

This project combines an ESP8266 processor with a 4x7-segment display, such that
we can operate as an NTP-backed clock.

The code is self-contained, and described on this project homepage:

* https://steve.fi/Hardware/d1-ntp-clock/


# Configuration

Once compiled and uploaded the project will attempt to connect to your
local WiFi network.  If no access details have been stored previously
it will instead function as an access-point.

Connect to this access-point with your mobile, or other device, and
you can select which network it should auto-join in the future.

> (This "connect or configure" behaviour is implemented by the excellent [WiFiManager](https://github.com/tzapu/WiFiManager) class.)


# And then jannek went just a bit mad.

During a few years, Janne K. (writing this) wrote a few changes and new features, building a few of these things on the way. There are still bugs. Of course there are; They're the only infinite resource in coding. But, there's a bit of good stuff too, which would be a pity to just keep on my own drives, so here it is in all it's horrifying mess. Enjoy!

Most noteworthy changes: 
* Full NTP protocol implementation with drift correction, usually accurate to less than 10 ms/h.
* EU DST rules optionally followed.
* Supports anything from no display at all to 2x 4x7seg decimal displays + 8x8 matrix + GY-49 light sensor + button switch + NMEA COG/SOG input via USB serial.
* Pretty nice web interface to check and configure stuff.
* Different display modes etc.
* Switched to VS Code + Platformio.
