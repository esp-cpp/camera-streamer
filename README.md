# camera-streamer

Example for [ESP32 TimerCam](https://github.com/m5stack/TimerCam-idf) rebuilt using [ESPP](http://github.com/esp-cpp/espp) to stream video over the network

It uses RTSP + RTP (over UDP) to perform real-time streaming of the camera data over the network to multiple clients.

https://user-images.githubusercontent.com/213467/236601550-ba1a5ba1-4f1c-4dfa-9b64-94afbd46ef3f.mp4

To facilitate easy connection to the camera, it also runs a mDNS server to
advertise the camera's IP address / port that the RTSP server is running on.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [camera-streamer](#camera-streamer)
  - [Use](#use)
    - [Program](#program)
    - [Configure](#configure)
  - [Development](#development)
    - [Environment](#environment)
    - [Build and Flash](#build-and-flash)
  - [Hardware](#hardware)
    - [Full pin-out:](#full-pin-out)
      - [Camera Interface (OV3660):](#camera-interface-ov3660)
      - [RTC (BM8563):](#rtc-bm8563)
      - [Battery:](#battery)
      - [Input / Output:](#input--output)
  - [Additional References](#additional-references)

<!-- markdown-toc end -->

## Use

You must first program your hardware. Afterwards, you can configure it via a USB
connection using its built-in CLI.

### Program

The ESP32-TimerCam will require one-time programming to function.

Download the release `programmer` executable from the latest [releases
page](https://github.com/esp-cpp/camera-streamer/releases) for `windows`,
`macos`, or `linux` - depending on which computer you want to use to perform the
one-time programming.

1. Download the programmer
2. Unzip it
3. Double click the `exe` (if windows), or open a terminal and execute it from
   the command line `./camera-streamer_programmer_v2.0.0_macos.bin`.

### Configure

To configure it, simply connect it to your computer via USB and open the serial
port in a terminal (e.g. `screen`, `PuTTY`, etc.) at 115200 baud. Once there,
you can use it as you would any other CLI - and the `help` command will provide
info about the commands available.

Any SSID/Password you set will be securely saved in the ESP32-TimerCam's NVS,
which is managed by the ESP-IDF WiFi subsystem.

![CleanShot 2025-06-22 at 22 38 41](https://github.com/user-attachments/assets/4b698b21-c66e-469e-9c49-a12d9f0ae65b)

```console
sta> help
Commands available:
 - help
	This help message
 - exit
	Quit the session
 - log <verbosity>
	Set the log verbosity for the wifi sta.
 - connect
	Connect to a WiFi network with the given SSID and password.
 - connect <ssid> <password>
	Connect to a WiFi network with the given SSID and password.
 - disconnect
	Disconnect from the current WiFi network.
 - ssid
	Get the current SSID (Service Set Identifier) of the WiFi connection.
 - rssi
	Get the current RSSI (Received Signal Strength Indicator) of the WiFi connection.
 - ip
	Get the current IP address of the WiFi connection.
 - connected
	Check if the WiFi is connected.
 - mac
	Get the current MAC address of the WiFi connection.
 - bssid
	Get the current BSSID (MAC addressof the access point) of the WiFi connection.
 - channel
	Get the current WiFi channel of the connection.
 - config
	Get the current WiFi configuration.
 - scan <int>
	Scan for available WiFi networks.
 - memory
	Display minimum free memory.
 - battery
	Display the current battery voltage.
```

## Development

If you wish to modify / recompile the code, you will need to set up your
development environment to be able to build and flash your target hardware.

### Environment

This project is an ESP-IDF project, currently [ESP-IDF
v.5.4](https://github.com/espressif/esp-idf).

For information about setting up `ESP-IDF v5.4`, please see [the official
ESP-IDF getting started
documentation](https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32s3/get-started/index.html).

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Hardware

This sample is designed to run on the ESP32 TimerCam ([Amazon Link](https://www.amazon.com/dp/B09W2RSPGL?psc=1&ref=ppx_yo2ov_dt_b_product_details)).

The ESP32 TimerCam has the following specs:

* 8MB PSRAM
* 4MB flash
* OV3660 image sensor (3MP, 66.5 DFOV), (8-/10-Bit RAW, RGB, and YCbCr output, compression), XCLK frequency: 10 MHz
* SH1.0-2P Battery Interface
* I/O: LED, Button
* RTC (BM8563)
* Battery

### Full pin-out:

#### Camera Interface (OV3660):

| Camera Pin | ESP32 GPIO Number |
|------------|-------------------|
| RESET      | IO15              |
| XCLK       | IO27              |
| VSYNC      | IO22              |
| PCLK       | IO21              |
| HREF       | IO26              |
| SIOC       | IO23              |
| SIOD       | IO25              |
| D0         | IO32              |
| D1         | IO35              |
| D2         | IO34              |
| D3         | IO5               |
| D4         | IO39              |
| D5         | IO18              |
| D6         | IO36              |
| D7         | IO19              |

#### RTC (BM8563):

From their documentation...

| RTC Pin | ESP32 GPIO Number |
|---------|-------------------|
| SCL     | IO13              |
| SDA     | IO4               |

From their actual code...

| RTC Pin | ESP32 GPIO Number |
|---------|-------------------|
| SCL     | IO14              |
| SDA     | IO12              |

#### Battery:

| Battery Pin | ESP32 GPIO Number |
|-------------|-------------------|
| ADC         | IO38              |
| Output Hold | IO33              |

#### Input / Output:

| I/O Function | ESP32 GPIO Number |
|--------------|-------------------|
| LED          | IO2               |
| Button       | IO37              |


## Additional References

* https://github.com/espressif/esp32-camera
* https://github.com/espressif/esp-idf
* https://github.com/esp-cpp/espp
* https://github.com/m5stack/TimerCam-idf
* https://docs.m5stack.com/#/en/unit/timercam
