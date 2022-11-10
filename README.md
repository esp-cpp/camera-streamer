# camera-streamer

Example for [ESP32 TimerCam](https://github.com/m5stack/TimerCam-idf) rebuilt using [ESPP](http://github.com/esp-cpp/espp) to stream video over the network

It creates a task which pushes JPEG frames (1600x1200 px) from the camera into a FreeRTOS queue, along with another task that reads those frames from the queue and sends them via a TCP client socket to a TCP server. An example python TCP server is provided in [./display_frames.py](./display_frames.py).

https://user-images.githubusercontent.com/213467/201152307-a0a7154a-575e-455d-91cf-437a76822974.mp4

![CleanShot 2022-11-10 at 10 07 29](https://user-images.githubusercontent.com/213467/201146956-ece6ff47-a876-439c-94b6-cc2fbd67e235.png)

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
