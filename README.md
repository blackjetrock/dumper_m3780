# dumper_m3780
Dumper for the M3780 microcontroller family
===========================================

Uses Sean Riddle's dumper circuit, but with a RP Pico instead of the
PIC. I ported Sean's code over to the Pico but couldn't get reliable
reads, so I rewrote it to use a generic system of injecting instructions
and used a more elaborate scheme to get the data out of the device.

The TEST pin handling is the same as Sean's code, but I have code that
can inject a (preset, in arrays) sequence of code into the device. This
allowed me to debug my instruction extraction and come up with a method of 
reading ROM bytes reliably. As I still had issues with a value every
few hundred bytes I now read each byte in the ROM many times and only if
that block read gives the same value is it used as the ROM byte value.
If the values aren't the same then the block of reads is re-done 
until it is. This means it takes a while to read the ROM but the 
values are, I think, correct. I checked operation with the Boris chess
computer ROM and I got a dump that matched Sean's ecxept for the last
few bytes which are, I think, unused values.

I then dumped the ROM from a device out of an HP82143 printer. The Boris 
and HP dumps ar ein this repository.

Links
=====

Sean Riddle's site:
https://seanriddle.com/
