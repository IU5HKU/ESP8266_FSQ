# ESP8266_FSQ
Simple FSQ beacon for ESP8266, with the Etherkit Si5351A Breakout Board, forked from Jason Milldrum NT7S github, but works flawlessly with the cheaps chinese Si5351 breakout modules too.

Adapted for ESP8266 by Marco Campinoti IU5HKU 28/03/2019
This release uses the timer1 hardware interrupt for the required precise timing.
For the moment i've powered off the WiFi, maybe in a future release i'll use it for something like an interactive webpage to change frequency ad text 
of the beacon.
 
Original code based on Feld Hell beacon for Arduino by Mark Vandewettering K6HX, adapted for the Si5351A by Robert 
Liesenfeld AK6L <ak6l@ak6l.org>. 
Arduino Timer setup code by Thomas Knutsen LA3PNA. ESP8266 Timer setup code by Marco Campinoti IU5HKU.

You can change from 2 and 6baud, but it's trivial to calculate new interrupt timings for the others FSQ values.
Usually FSQ was born for 80 40 and 30m use, but can be used upto VHF too.
Si5351A is connected via I2C on pin D1 (SCL) and D2 (SDA) as marked on Wemos D1 mini Lite

Many thanks to Jason Milldrum NT7S for his work around Si5351 and for his superb library.
