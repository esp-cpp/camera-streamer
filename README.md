# camera-streamer

Example for [ESP32 TimerCam](https://github.com/m5stack/TimerCam-idf) rebuilt using [ESPP](http://github.com/esp-cpp/espp) to stream video over the network

The ESP32 TimerCam has the following specs:

* 8MB PSRAM
* 4MB flash
* OV3660 image sensor (3MP, 66.5 DFOV), (8-/10-Bit RAW, RGB, and YCbCr output, compression), XCLK frequency: 10 MHz
* SH1.0-2P Battery Interface
* I/O: LED, Button
* RTC (BM8563)
* Battery

## Full pin-out:

### Camera Interface (OV3660):

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

### RTC (BM8563):

| RTC Pin | ESP32 GPIO Number |
|---------|-------------------|
| SCL     | IO13              |
| SDA     | IO4               |

### Battery:

| Battery Pin | ESP32 GPIO Number |
|-------------|-------------------|
| ADC         | IO38              |
| Output Hold | IO33              |

### Input / Output:

| I/O Function | ESP32 GPIO Number |
|--------------|-------------------|
| LED          | IO2               |
| Button       | IO37              |


# Additional References

* https://github.com/espressif/esp32-camera
* https://github.com/espressif/esp-idf
* https://github.com/esp-cpp/espp
* https://github.com/m5stack/TimerCam-idf
* https://docs.m5stack.com/#/en/unit/timercam
