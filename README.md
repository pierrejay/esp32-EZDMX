# ESP32 EZDMX Library

## Overview

`EZDMX` is a simple & lightweight library for controlling and receiving DMX512 signals on ESP32 platforms. It supports both Arduino ESP32 Core and native ESP-IDF environments. It uses FreeRTOS tasks to handle DMX operations asynchronously, allowing main application code to run without blocking while DMX communication occurs in the background.

## Operating Modes

`EZDMX` supports the most basic functions for each of the DMX operating modes acting as a simple interface to set & read individual channel values (no support for RDM, scenes, etc.).

- **`MASTER` Mode**: The ESP32 functions as a DMX controller, sending DMX data for the 512 channels to connected fixtures. The library manages the DMX protocol timing requirements including proper break timing and frame structure.

- **`SLAVE` Mode**: The ESP32 functions as a DMX receiver, processing incoming DMX signals to read the DMX values of the 512 channels. This mode enables debugging or building custom DMX-controlled devices.


## Key Features

- Simple API for DMX control and monitoring
- Compliant with DMX512 timing specifications (UART API + esp_timer for MTBF)
- Support for both `MASTER` (transmission) and `SLAVE` (reception) modes
- Asynchronous operation using FreeRTOS tasks
- Efficient error handling with Result system
- Works with all ESP32 family chips


## Hardware Requirements

- `ESP32` (`C3`, `S3`, `C6`...) microcontroller
- UART to RS485 transceiver (e.g. `TD301M485`, `CA-IS2092A`)
- DMX-compatible equipment

## Timings

By default, the library uses the following constants for timings (can be overridden in `EZDMX.h`):
```cpp
static constexpr uint32_t DMX_BREAK_US = 180;   // Break (min 92µs)
static constexpr uint32_t DMX_MAB_US = 24;      // Mark After Break (min 12µs)
static constexpr uint32_t DMX_MTBF_MS = 10;     // Mark Time Between Frames (spec says 0-1000 ms)
```
The default settings seem to work well with most off-the-shelf receivers, the MTBF time can be reduced if needed.

Timing calculation summary:
```txt
Time for a full DMX transmission:
- Break/MAB : 1x per transmission
- 513 frames per transmission (start code + 512 slots)
- 11 bits per frame (1 start bit + 8 data bits + 2 stop bits)

=> Minimum data time @ 250 kbaud = 5643/250k = 22,6 ms
=> Break/MAB time = 204 µs (with default settings)
=> Minimum frame time = 22,8 ms (default: 32,8 ms with 10ms MTBF)
=> Max frequency = 43,9 Hz (default: 30,4 Hz): conform to DMX spec (max 44 Hz)
```

The library sends the full 512 DMX channels at each transmission, even if all of them are not defined. By default, all registers are set to 0. The `set()` methods allow to partially update channels values and let others unchanged (see reference API for more details).


## Installation

Add lib directory (or just the `EZDMX.h` file) to your project

```cpp
#include "EZDMX.h"
```

## Basic Usage

### Initialization

Create an `EZDMX` instance using either the Arduino or ESP-IDF constructor.
The "Arduino API" (actually only the constructor to take a `Serial` object) will be automatically enabled if the `ARDUINO` macro is defined.

```cpp
// Arduino environment
EZDMX dmx(EZDMX::MASTER,  // DMX mode
          &Serial1,       // UART Serial port
          16,             // UART TX pin
          23,             // UART RX pin
          -1);            // DE/RE pin (not used here)

// Or for Slave mode
EZDMX dmx(EZDMX::SLAVE, &Serial1, 16, 23, -1);

// ESP-IDF native environment
EZDMX dmx(EZDMX::MASTER,  // DMX mode
          UART_NUM_1,     // UART port number
          16,             // UART TX pin
          23,             // UART RX pin
          -1);            // DE/RE pin (not used here)

// Or for Slave mode
EZDMX dmx(EZDMX::SLAVE, UART_NUM_1, 16, 23, -1);
```

### Starting and Stopping DMX

```cpp
// Initialize DMX
auto result = dmx.begin();
if (result != EZDMX::SUCCESS) {
  Serial.printf("DMX init failed! Error: %s\n", EZDMX::toString(result.status));
}

// Start DMX transmission/reception
result = dmx.start();

// Stop DMX when finished
result = dmx.stop();
```

### Setting Channel Values (MASTER mode)

```cpp
// Set single channel
dmx.set(1, 255);  // Set channel 1 to full brightness

// Set multiple channels with a map
std::map<uint16_t, uint8_t> mapValues = {
  {1, 255}, 
  {2, 128}, 
  {3, 64}
};
dmx.set(mapValues);

// Set multiple channels with an array
uint8_t arrValues[] = {255, 128, 64};
dmx.set(arrValues, sizeof(arrValues)/sizeof(arrValues[0]), 1);  // Set 3 channels starting at channel 1 (default)

// Set multiple channels with a vector
std::vector<uint8_t> vecValues = {255, 128, 64};
dmx.set(vecValues, 6);  // Set values starting at channel 6 (default: 1)
```

### Reading Channel Values

```cpp
// In MASTER mode - read set values
uint8_t value;
dmx.get(1, value);  // Get current value of channel 1

// In SLAVE mode - read received values
uint8_t value;
uint64_t timestamp;
dmx.get(1, value, &timestamp);  // Get value and last update timestamp

// Read all channels
uint8_t values[EZDMX::DMX_UNIVERSE_SIZE];
dmx.getAllChannels(values, &timestamp);
```

### Resetting All Channels

```cpp
dmx.resetAllChannels();  // Set all channels to 0
```

### Performance Logging

Pass `true` to the last constructor parameter `enablePerfLog` (`false` by default) to print detailed performance information (timer precision, frame-to-frame jitter) every 100 frames. Logs are sent to `Serial` on Arduino or standard output on IDF, and can be useful for troubleshooting timing issues.

```cpp
EZDMX dmx(EZDMX::MASTER, &Serial1, 16, 23, -1, /*enablePerfLog*/ true);
```

## Error Handling

The library uses a `Result` type for error handling. It returns the status of the operation and the number of channels read/written (if applicable).
- Test the return of each method to get the status (see `EZDMX::Status` enum for error codes)
- Access to `result.status` and `result.nbChannelsRW` as required

```cpp
auto result = dmx.set(1, 255); // Set channel 1 to 255
if (result != EZDMX::SUCCESS) {
  Serial.printf("Failed to set channel! Error: %s\n", EZDMX::toString(result.status));
} else {
  Serial.printf("Successfully set %d channels\n", result.nbChannelsRW);
}
```

## Examples

- **`basic-button.cpp`** file (Arduino Core): 
  - Simple DMX controller & receiver acting as `MASTER` or `SLAVE` (selected with build flag)
  - Displays read/write values of the first 8 channels to USB serial
  - Master changes values of channels from 1 to 8 when the button is pressed
- **`master-http.cpp`** file (Arduino Core): 
  - Simple HTTP-DMX Master bridge based on `ESPAsyncWebServer` library
  - Get channel values with GET request
  - Set & reset channel values + start/stop transmission with POST request & JSON body


## API Reference

### Constructors

```cpp
// Arduino constructor
#ifdef ARDUINO
EZDMX(Mode mode = Mode::MASTER, Stream* serial = &Serial1, int txPin = 17, int rxPin = 16, int dePin = -1, bool enablePerfLog = false)
#endif

// ESP-IDF constructor
EZDMX(Mode mode = Mode::MASTER, uart_port_t uart_num = UART_NUM_1, int txPin = 17, int rxPin = 16, int dePin = -1, bool enablePerfLog = false)
```

Parameters list:
- `mode`: MASTER (sending) or SLAVE (receiving) mode
- Serial port:
  - `serial`: Arduino Serial port to use
  - `uart_num`: UART port number (UART_NUM_0, UART_NUM_1, etc.)
- `txPin`: DMX transmission pin
- `rxPin`: DMX reception pin
- `dePin`: Pin to activate RS485 transmitter (optional, -1 if not used)
- `enablePerfLog`: set to `true` to enable periodic performance logs on serial output (optional, default `false`)


### Main Methods

| Method | Description |
|--------|-------------|
| `Result begin()` | Initialize DMX hardware |
| `Result start()` | Start DMX transmission/reception |
| `Result stop()` | Stop DMX transmission/reception |
| `Result set(uint16_t channel, uint8_t value)` | Set value for a single channel (1-512) |
| `Result set(const std::map<uint16_t, uint8_t>& channelValues)` | Set values from map |
| `Result set(const std::unordered_map<uint16_t, uint8_t>& channelValues)` | Set values from unordered map |
| `Result set(uint8_t* values, uint16_t nbChannels, uint16_t startChannel = 1)` | Set values from C-style array |
| `Result set(const std::vector<uint8_t>& values, uint16_t startChannel = 1)` | Set values from vector |
| `Result get(uint16_t channel, uint8_t& value, uint64_t* timestamp = nullptr)` | Get channel value (+ optional timestamp of last received frame for `SLAVE` only) |
| `Result getAllChannels(uint8_t* values, uint64_t* timestamp = nullptr)` | Get all channel values (+ optional timestamp of last received frame for `SLAVE` only) |
| `Result resetAllChannels()` | Set all channels to 0 (`MASTER` only) |
| `bool isRunning()` | Check if DMX is active |
| `uint64_t getLastFrameTimestamp()` | Get timestamp of last received frame (`SLAVE` only) |

### Constants

- `DMX_UNIVERSE_SIZE`: 512 (number of channels in a DMX universe)
- `DMX_BUFFER_SIZE`: 1024 (internal buffer size)
- DMX timing constants: `DMX_BREAK_US`, `DMX_MAB_US`, `DMX_MTBF_MS` (see [Timings](#timings) section)

### Compatibility

Tested & stable with:
- Arduino ESP32 Core 3.0.7: [pioarduino](https://github.com/pioarduino/platform-espressif32)
- ESP-IDF v5.1


## Wiring Example

Basic wiring for ESP32 with `MAX485` or similar UART-RS485 transceiver:

```
ESP32           MAX485
-----           ------
TX (GPIO16) <-> DI
RX (GPIO23) <-> RO
GPIO4       <-> DE/RE (if used)
3.3V        <-> VCC
GND         <-> GND

MAX485          DMX Device
------          ----------
A (+)       <-> DMX + (Pin 3)
B (-)       <-> DMX - (Pin 2)
G           <-> DMX G (Pin 1)
```