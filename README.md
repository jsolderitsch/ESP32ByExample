# ESP32ByExample
Resources for Udemy and Tech Explorations ESP32 By Example Course.

Windows users may need to add or update a USB serial driver to connect to the ESP32.

The driver can be found at the URL: [Silicon Labs Driver](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads)

A video walking through the installation process is at: [Driver Installation Walkthrough](https://www.youtube.com/watch?v=UuuqnmJIjR0)

The Assembly sketches require some libraries to be added so that they can compile and be installed. A video (not mine) illustrating the process is at: [Library Installation](https://www.youtube.com/watch?v=v2qIB6iigPI)

If you have decided to purchase a PCB option of the ESP32 Educational Kit (EEK) for this course: [EEK Storefront](https://shop.iotinurhand.store) please be aware that the GPIO pin assignments in all course examples need to be adjusted accordingly.

The following code snippet contains the pin numbers for all six LEDs and all six pushbutton switches.
```
// PCB LED GPIO pin assignments
#define BLUE 25
#define YELLOW 15
#define TOP_GREEN 21
#define TOP_RED 26
#define BOTTOM_RED 27
#define BOTTOM_GREEN 4
// PCB GPIO pins where the buttons are connected to
#define BOTTOM_RIGHT 14
#define BOTTOM_LEFT 36
#define TOP_RIGHT 34
#define TOP_LEFT 33
#define MIDDLE_RIGHT 32
#define MIDDLE_LEFT 39
```

If you find that sketches are not working it is likely that the pin numbers used need to be adjusted as defined above.
