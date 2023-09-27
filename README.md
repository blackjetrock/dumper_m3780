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

ERRATA
======

V1.0 PCB
--------

This version of the PCB does not work unless you add level shifters 
between the Pico and the 4066 control lines. I added a couple of FETs
onto my PCB manually. If you don't do this then the PCB will
not work.

Here's the V1.0 PCB I used:

![IMAG4222](https://github.com/blackjetrock/dumper_m3780/assets/31587992/57701a29-a1b2-4dc2-8942-d155bad40f65)

You can see a dc-dc converter module on the right, and the level shifters I wired in going from the 4066 
to the Pico on the left. The 40 pin IC is the M3780. the two LM317T circuits are on the right with their 
adjustment potentiometers.

Code
----
There's a menu system on the USB. Press 'h' to see the command list. To dump a ROM use 'u' then 'S' to dump the ROM. It takes a while as each byte
is read, when it's done the full dumpo is displayed. There's some problem that means that you need to select 'u'
before using 'S'. As this is a one or two use dumper, I'm not bothered by these wrinkles.


Links
=====

<pre>
Sean Riddle's site:                     https://seanriddle.com/
Forum thread:                           https://www.hpmuseum.org/forum/thread-20043.html
</pre>
