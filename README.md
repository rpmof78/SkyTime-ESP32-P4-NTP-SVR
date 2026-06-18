========================================================

SkyTime
GPS Based Stratum-1 NTP Server

Version 1.0

Copyright (C) 2026 RPMof78

License:
GNU General Public License Version 3 or later
SPDX-License-Identifier: GPL-3.0-or-later
========================================================

SkyTime is a GPS/PPS disciplined Stratum-1 NTP Server
built on the Waveshare ESP32-P4-ETH PoE platform.

Features
--------
- GPS disciplined UTC time
- PPS timing validation
- Stratum-1 NTP service
- Ethernet networking
- DHCP or Static IP operation
- LCD local status display
- Dashboard web interface
- System status page
- Read-only Event Log viewer
- Read-only NTP Log viewer
- Identity configuration
- Network configuration
- Web initiated reboot
- Configuration persistence

Hardware
--------
Board:   Waveshare ESP32-P4-ETH PoE
Display: ST7789 240x240 SPI
GPS:     ATGM336H
PPS:     GPIO46 (Rising Edge)
Button:  GPIO47 (Active LOW)

Configuration
-------------
Default Hostname: SkyTime

Default Static Network:
IP Address : 192.168.0.123
Subnet Mask: 255.255.255.0
Gateway    : 192.168.0.1
DNS Server : 192.168.0.1

License
-------
This software is free software.

You may redistribute and/or modify it under the terms
of the GNU General Public License as published by the
Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

A copy of the GNU GPL license is provided in:

/LICENSE.txt

Project Files
-------------
/web         Web user interface
/config      Configuration files
/logs        Runtime log files
/LICENSE.txt GNU GPL v3 license text
/README.txt  This file

Release Status
--------------
Version 1.0
Production Release

Author
------
RPMof78

========================================================
End of File
========================================================
