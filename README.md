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
and HP dumps are in this repository.

At the moment the code always dumps 4K

**ERRATA**
==========

V1.0 PCB
--------

This version of the PCB does not work unless you add level shifters 
between the Pico and the 4066 control lines. I added a couple of FETs
onto my PCB manually. If you don't do this then the PCB will
not work.

Links
=====

<pre>
Sean Riddle's site:                     https://seanriddle.com/
Forum thread:                           https://www.hpmuseum.org/forum/thread-20043.html
</pre>
