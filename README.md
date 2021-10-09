# Antenna-Tracker-for-FRsky-Telemetry
Frsky Sport Telemetry Decoder and 3D printed Antenna Tracker  
The FRskyR9MSportDecoder library and SportDecoderTracker sketch run on an Arduino Uno and decode the Frsky Telemetry which is available on the Taranis or Frsky R9M module Sport pins. A bluetooth module on the back of the Taranis is connected to the Sport pin and communicates with a similar module in the tracker. The library and sketches will translate the GPS latitude, longitude and altitude data into a bearing from the tracker to the quad and an angular altitude to the quad. These in turn are translated into angular movements of horizontal and vertical servos to track the quad across the sky.

The SportDecoder sketch demonstrates using the library for your own sketches.

The SportDecoderTest sketch is provided to test the tracker's operation before using in the field.

Links to a detailed construction video and parts list is available here

Link to 3D printer files https://www.thingiverse.com/thing:4999823

![tracker01](https://user-images.githubusercontent.com/13920701/136672814-b28054df-c4d4-4425-82c3-7b1e532ee1ee.jpg)



