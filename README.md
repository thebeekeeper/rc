TURBO TORNADO
=============

When I was a kid I had a RC car that I really liked to play with, but it pretty rarely worked.  30+ years later I have a couple of decades of experience building embedded systems to control motors and stuff like that so I decided to design replacement electronics for it so my kid can play with it.

To start, I took the top off the car and removed the old PCB.  Here's what we see under there:

![alt text](https://github.com/thebeekeeper/rc/blob/main/empty_car.jpg "Empty Car")

We've got leads to the coil that controls steering and some on the other end to the drive motor.  On the bottom there are 6 slots for AA batteries, so I'm assuming the motor and solenoid will work off 9V.  I checked this by connecting a 9V battery to both pairs of leads and saw that the steering and drive motor still work.  To change direction of both actuators, you can just reverse polarity on the battery.

What I really want to do with this project is put a camera where that guys head is supposed to be.  I've always wanted a first person view when driving the car around.

## Parts Selection

The features I'm designing for are:

1. Video streaming to some remote device
2. Wireless control
3. Ability to drive two inductive loads with PWM

I've been using `esp32` devices at work for a while, so I decided on the `ESP32-S3-MINI-1U-N8`.  I found the schematic for the `esp32-cam` board [here](https://github.com/SeeedDocument/forum_doc/blob/master/reg/ESP32_CAM_V1.6.pdf) and it looks pretty straightforward to wire to an FPC connector for a standard camera.  It also supports Bluetooth and WiFi, so I'll have some options for wireless control and video streaming.  I've read that it has issues with video quality when using the chip antenna, so I'll plan on using an external antenna.

To drive the motor and solenoid I was initially planning to build couple of standard H-bridge circuits, but then I found [A3909GLNTR](https://www.digikey.com/en/products/detail/allegro-microsystems/A3909GLNTR-T/3979655) which is an IC with the exact circuit I'd be building anyway so I decided to use that.  I've used some Allegro parts in other designs and they've worked well, so this should be pretty easy.

