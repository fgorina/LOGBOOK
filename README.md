# LOGBOOK - A flexible log recorder

Logbook is a smalldevice that connects to your Ship network/Server and
records some data synchronized. That means that for each period it 
gives you all the data associated. You know your cog, the wind angle,
wind speed, sog, heading, rudder position, etc.

Logbook stores the data in a micro SD card (till 8Gb) but it may be 
accessed with a browser. No need to open it to get the card.

Data may be stored either a .csv file, with tabs as separator and . as decimal point
or in a .gpx file with extensions.

All files are compressed wit gzip in the card and compression is very efective, a .gpx
file of 212Kb may be reduced to 28Kb. 

Se the format document for details of the data and how it is enclosed. 

Configuration is also done with a browser, defining protocols and format
and the addres of the server.

## ToDo

1. When in N2K allow to select some devices and some PGN's. (Show List) This will reduce size and increase clarity.
2. When in SignalK allow to select the paths we want to record
3. Add a parameter timeStep to select the timeStep we want (now is 1s)
4. In long steps may be interestint to store min, max, avg of values (in numeric data)
5. Create the dtd's for the gpx extensions
6. Make a simple way to clear the card etc when in the device, with password.
7. Begin to think in an autorun.
