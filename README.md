# Antenna-Tracker-for-FRsky-Telemetry
Frsky Sport Telemetry Decoder and Antenna Tracker  
The FRskyR9MSportDecoder library and SportDecoderTracker sketch run on an Arduino Uno and decode the Frsky Telemetry which is available on the Taranis or Frsky R9M module Sport pin. A bluetooth module in the tracker communicates with a similare module on the back of the Taranis. The library and sketches will translate the GPS latitude, longitude and altitude data into a bearing from the tracker to the quad and an angular altitude to the quad. These in turn are translated into angular movements of horizontal and vertical servos to track the quad across the sky.
The SportDecoder sketch demonstrates using the library for your own sketches.
The SportDecoderTest sketch is provided to test the tracker's operation before using in the field.
Links to a detailed video, 3D printer files and parts list available here
