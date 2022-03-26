## OASIS

OASIS is a smart home operating system based on ROS 2.

OASIS currently specializes in computer vision, input streaming and general automation.

## Computer Vision

In my home I have a Kinect 2 and a bunch of poor quality laptop cameras.

I ported the [Kinect 2 driver](https://github.com/eigendude/oasis_kinect2/) to ROS 2. I also run a background subtractor on all camera feeds courtesy of [bgslibrary](https://github.com/andrewssobral/bgslibrary).

Kodi is my visual interface. I added [Smart Home support](https://github.com/xbmc/xbmc/pull/21183) so that it can show multiple ROS image topics.

![Smart Home Camera View](docs/media/Smart_Home_Camera_View.png)

## Input streaming

Kodi is also my input interface.
I connected a Bluetooth adapter to my NAS and it streams game controller input on a ROS topic.

Currently, I'm using my PS4 controller to control the speed of my childhood Lego train (the Falcon is new).
A Raspberry Pi subscribes to the input, and controls an Arduino connected to a robotics motor controller that drives the train's 9V motors.

[![Lego Train via PS4 Controller](https://img.youtube.com/vi/zMA9HYPH4Tw/hqdefault.jpg)](https://youtu.be/zMA9HYPH4Tw)

To communicate with the Arduino, I added a complete Firmata implementation with additional support for temperature and humidity sensors, I2C, sonar, stepper motors, SPI, servos and CPU fans.
My Firmata client fork is at https://github.com/eigendude/pymata-express.
