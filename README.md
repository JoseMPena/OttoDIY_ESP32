# OttoDIY_ESP32
Arduino IDE sketch for an Otto DIY Humanoid Robot built around the ESP32 SoC.

The standard [Otto DIY robot](https://www.ottodiy.com/) makes use of an Arduino Nano (ATmega328), which is quite limited in terms of program space, features and connectivity.
Replacing the nano with an ESP32C3 SuperMini, we can load more extensive programs and make use of the additional ESP32 features such as the onboard WiFi module for OTA (over-the-air) updates and the built-in Bluetooth module for controlling the robot via an app, battery charging via the Expansion Board, Deep Sleep option for power saving, and many more.

As an example, [this sketch](OttoDIY_ESP32.ino) features the following functionalities:
* **OTA updates**: After an initial upload via USB-C, any subsequent revisions of the sketch can be uploaded wirelessly via WiFi. This is especially handy in case of ESP32 versions which otherwise require an onboard button to be depressed while uploading via USB-C. Note that WiFi is turned off once you press the touch sensor to switch over to one of the following modes:
* **App mode**: Otto can be fully controlled via Bluetooth using the app (see android_app folder).
* **Avoid mode**: Otto walks forwards until an obstacle is detected and will then move to avoid it.
* **Detect mode**: Otto makes sounds and dances whenever movement is detected.
* **Dance mode**: Otto will dance whenever sound is detected.


Pressing the Action Button (right side of Otto) switches between the different modes.
Pressing the Switch Button (left side of Otto) switches between dances.

