#ifndef EZDMX_H
#define EZDMX_H

#ifdef ARDUINO
    #include <Arduino.h>
#endif
#include <cstring>
#include <algorithm>
#include <map>
#include <vector>
#include <unordered_map>
#include <driver/uart.h>
#include <soc/uart_struct.h>
#include <soc/uart_periph.h>
#include <hal/uart_ll.h>
#include <driver/gpio.h>
#include <esp_cpu.h>
#include <esp_intr_alloc.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <freertos/timers.h>

class EZDMX {
public:
    enum class Mode {
        MASTER,
        SLAVE
    };

    enum Status {
        SUCCESS = 0,           // Operation successful
        UART_ERROR,              // UART initialization or operation error
        GPIO_ERROR,              // GPIO configuration error (TX, RX, DE pins)
        WRONG_MODE,        // Operation not allowed in current mode (MASTER/SLAVE)
        CHANNEL_OVERFLOW,  // Channel number out of limits (>512 or <1)
        NULL_INPUT,        // Null pointer provided as parameter
        TASK_FAILED,       // Task creation/execution failed
        NOT_RUNNING,       // Operation impossible because DMX is not started
        FRAME_ERROR,       // DMX frame error (missing break, invalid start code...)
        TIMEOUT,           // Timeout on DMX operation
    };

    static constexpr const char* toString(Status status) {
        switch (status) {
            case SUCCESS: return "success";
            case UART_ERROR: return "uart error";
            case GPIO_ERROR: return "gpio error";
            case WRONG_MODE: return "wrong mode";
            case CHANNEL_OVERFLOW: return "channel overflow";
            case NULL_INPUT: return "null input";
            case TASK_FAILED: return "task failed";
            case NOT_RUNNING: return "not running";
            case FRAME_ERROR: return "frame error";
            case TIMEOUT: return "timeout";
            default: return "unknown";
        }
    }

    struct Result {
        Status status;
        uint16_t nbChannelsRW;
        // Overload operators for direct comparison with Status
        bool operator==(Status s) const { return status == s; }
        bool operator!=(Status s) const { return status != s; }
    };
    // Helpers to create results
    static constexpr Result Error(Status status, uint16_t nbChannelsRW = 0) { return Result{status, nbChannelsRW}; }
    static constexpr Result Success(uint16_t nbChannelsRW = 0) { return Result{SUCCESS, nbChannelsRW}; }

    // DMX constants
    static constexpr size_t DMX_UNIVERSE_SIZE = 512;
    static constexpr size_t DMX_BUFFER_SIZE = 1024;
    static constexpr uint32_t DMX_BREAK_US = 180;   // Break (min 92µs)
    static constexpr uint32_t DMX_MAB_US = 24;      // Mark After Break (min 12µs)
    static constexpr uint32_t DMX_MTBF_MS = 10;     // Mark Time Between Frames (spec says 0-1000 ms)

    /**
     * @brief SLAVE reception counters, cumulative since start() or resetStats()
     *
     * Only `frames` were published to the channel values. A frame with a framing
     * error or a FIFO overflow is dropped, so the last good values stay in place.
     * Timing figures stay at zero unless enableLineTiming(true) was called.
     */
    struct SlaveStats {
        uint32_t breaks;          // BREAK conditions detected
        uint32_t frames;          // Good frames published (start code 0)
        uint32_t shortFrames;     // Good frames with fewer than 512 slots (legal in DMX)
        uint32_t errorFrames;     // Frames dropped: framing error or FIFO overflow inside them
        uint32_t startCodeFrames; // Frames ignored: non-zero start code
        uint32_t emptyFrames;     // BREAK followed by another BREAK with no data
        uint32_t overlongFrames;  // Bytes received past slot 512 (extra bytes ignored)
        uint32_t framingErrors;   // Framing errors not caused by a BREAK
        uint32_t fifoOverflows;   // Hardware FIFO overflows
        uint16_t lastSlots;       // Slot count of the last good frame
        uint16_t minSlots;
        uint16_t maxSlots;
        uint32_t periodMinUs;     // BREAK-to-BREAK period, 0 until two breaks were seen
        uint32_t periodAvgUs;
        uint32_t periodMaxUs;
        uint32_t timingSamples;   // Frames measured by the line timing probe
        uint32_t timingMissed;    // Frames whose BREAK falling edge the probe did not catch
        float breakMinUs, breakAvgUs, breakMaxUs;
        float mabMinUs, mabAvgUs, mabMaxUs;
        float startLowMinUs, startLowAvgUs, startLowMaxUs; // Low time of a 0x00 start code: 36 us at 250 kbaud
    };

#ifdef ARDUINO
    /**
     * @brief Constructor
     * @param mode Mode of operation (MASTER or SLAVE)
     * @param serial Pointer to the Arduino serial port to use (&Serial, &Serial1, etc.)
     * @param txPin DMX transmission pin
     * @param rxPin DMX reception pin (not used in master mode but required for configuration)
     * @param dePin Pin to activate RS485 transmitter (optional, -1 if not used)
     * @param enablePerfLog Enable performance logging
     */
    EZDMX(Mode mode = Mode::MASTER, Stream* serial = &Serial1, int txPin = 17, int rxPin = 16, int dePin = -1, bool enablePerfLog = false) :
        _mode(mode),
        _uart_num(serialToUartPort(serial)),
        _txPin(txPin),
        _rxPin(rxPin),
        _dePin(dePin),
        _running(false),
        _transmitTaskHandle(nullptr),
        _lastFrameTimestamp(0),
        _dmxTimer(nullptr),
        _timerStartTime(0),
        _frameCounter(0),
        _enablePerfLog(enablePerfLog),
        _frameIntervalsIndex(0),
        _frameIntervalsCount(0),
        _prevFrameSendTime(0)
    {
        // Initialize channels to 0
        resetAllChannels();
    }
#endif

    /**
     * @brief IDF constructor
     * @param mode Mode of operation (MASTER or SLAVE)
     * @param uart_num UART port number
     * @param txPin DMX transmission pin
     * @param rxPin DMX reception pin (not used in master mode but required for configuration)
     * @param dePin Pin to activate RS485 transmitter (optional, -1 if not used)
     * @param enablePerfLog Enable performance logging
     */
    EZDMX(Mode mode = Mode::MASTER, uart_port_t uart_num = UART_NUM_1, int txPin = 17, int rxPin = 16, int dePin = -1, bool enablePerfLog = false) :
        _mode(mode),
        _uart_num(uart_num),
        _txPin(txPin),
        _rxPin(rxPin),
        _dePin(dePin),
        _running(false),
        _transmitTaskHandle(nullptr),
        _lastFrameTimestamp(0),
        _dmxTimer(nullptr),
        _timerStartTime(0),
        _frameCounter(0),
        _enablePerfLog(enablePerfLog),
        _frameIntervalsIndex(0),
        _frameIntervalsCount(0),
        _prevFrameSendTime(0)
    {
        // Initialize channels to 0
        resetAllChannels();
    }


    ~EZDMX() {
        end();
    }

    /**
     * @brief Initialize the UART for DMX
     *
     * MASTER uses the IDF UART driver. SLAVE does not install it: start() takes the
     * UART interrupt itself, so that the end of a frame and the BREAK are handled in
     * order (see SlaveIsr).
     *
     * @return Result indicating success or failure
     */
    Result begin() {
        if (_begun) {
            return Success();
        }

        // Configure the UART
        uart_config_t uart_config = {
            .baud_rate = 250000,               // DMX standard baud rate
            .data_bits = UART_DATA_8_BITS,     // 8 data bits
            .parity = UART_PARITY_DISABLE,     // No parity
            .stop_bits = UART_STOP_BITS_2,     // 2 stop bits
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // No flow control
            .rx_flow_ctrl_thresh = 0,          // Not used
            .source_clk = UART_SCLK_DEFAULT,       // Source clock (for ESP32C6)
        };

        const bool useDriver = (_mode == Mode::MASTER);
        esp_err_t err = ESP_OK;

        // Install the UART driver
        if (useDriver) {
            err = uart_driver_install(_uart_num, DMX_BUFFER_SIZE, DMX_BUFFER_SIZE, 0, NULL, ESP_INTR_FLAG_IRAM);
            if (err != ESP_OK) {
                return Error(UART_ERROR);
            }
        }

        err = uart_param_config(_uart_num, &uart_config);
        if (err != ESP_OK) {
            if (useDriver) uart_driver_delete(_uart_num);
            return Error(UART_ERROR);
        }

        err = uart_set_pin(_uart_num, _txPin, _rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (err != ESP_OK) {
            if (useDriver) uart_driver_delete(_uart_num);
            return Error(GPIO_ERROR);
        }

        // Configure RS485 DE pin if needed: driving in MASTER mode, released in SLAVE mode
        if (_dePin >= 0) {
            gpio_config_t io_conf = {};
            io_conf.intr_type = GPIO_INTR_DISABLE;
            io_conf.mode = GPIO_MODE_OUTPUT;
            io_conf.pin_bit_mask = (1ULL << _dePin);
            io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

            err = gpio_config(&io_conf);
            if (err == ESP_OK) {
                err = gpio_set_level((gpio_num_t)_dePin, _mode == Mode::MASTER ? 1 : 0);
            }
            if (err != ESP_OK) {
                if (useDriver) uart_driver_delete(_uart_num);
                return Error(GPIO_ERROR);
            }
        }

        _begun = true;
        return Success();
    }

    /**
     * @brief Stop if needed and release the UART, so that it can be used for something else
     * @return Result indicating success or failure
     */
    Result end() {
        if (_running) {
            stop();
        }
        if (!_begun) {
            return Success();
        }
        if (_mode == Mode::MASTER) {
            uart_driver_delete(_uart_num);
        }
        if (_dePin >= 0) {
            gpio_set_level((gpio_num_t)_dePin, 0);
        }
        _begun = false;
        return Success();
    }

    /**
     * @brief Pin the MASTER transmit task to a core (call before start())
     *
     * The BREAK and MAB are timed inside a critical section, which masks interrupts
     * on that core for about 200 us. Pin it away from any time-sensitive interrupt.
     */
    void setTxCore(int core) {
        _txCore = core;
    }

    /**
     * @brief Measure BREAK, MAB and start code low time on each SLAVE frame (call before start())
     *
     * Costs one UART interrupt per byte and a few GPIO interrupts per frame. Edges are
     * timestamped in an interrupt handler, so figures carry the interrupt latency jitter.
     */
    void enableLineTiming(bool enable) {
        _timingEnabled = enable;
    }

    /**
     * @brief Start the DMX transmission
     * @return Result indicating success or failure
     */
    Result start() {
        if (_running) {
            return Success();
        }
        if (!_begun) {
            return Error(NOT_RUNNING);
        }

        if (_mode == Mode::MASTER) {
            // Create the esp_timer for precise DMX timing (one-shot) before the task that uses it
            esp_timer_create_args_t timer_args = {
                .callback = TimerCallback,
                .arg = this,
                .name = "dmx_timer"
            };
            esp_err_t errTimer = esp_timer_create(&timer_args, &_dmxTimer);
            if (errTimer != ESP_OK || _dmxTimer == nullptr) {
                _dmxTimer = nullptr;
                return Error(TASK_FAILED);
            }

            _running = true;

            // Create the transmission task
            BaseType_t ret = xTaskCreatePinnedToCore(TransmitTask,
                        "dmx_tx",
                        8192,
                        this,
                        5,
                        const_cast<TaskHandle_t*>(&_transmitTaskHandle),
                        _txCore);
            if (ret != pdPASS) {
                _running = false;
                _transmitTaskHandle = nullptr;
                esp_timer_delete(_dmxTimer);
                _dmxTimer = nullptr;
                return Error(TASK_FAILED);
            }
        } else {
            return startSlave();
        }
        return Success();
    }

    /**
     * @brief Stop the DMX transmission
     * @return Result indicating success or failure
     */
    Result stop() {
        if (!_running) {
            return Error(NOT_RUNNING);
        }

        _running = false;
        
        if (_mode == Mode::MASTER) {
            // Wake the task and wait for it to exit: it may still be using the timer
            TaskHandle_t task = _transmitTaskHandle;
            if (task != nullptr) {
                xTaskNotifyGive(task);
                for (int waitedMs = 0; _transmitTaskHandle != nullptr && waitedMs < 250; waitedMs++) {
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
            }

            // Stop and delete the timer
            if (_dmxTimer != nullptr) {
                esp_timer_stop(_dmxTimer);
                esp_timer_delete(_dmxTimer);
                _dmxTimer = nullptr;
            }
        } else {
            stopSlave();
        }
        return Success();
    }

    /**
     * @brief Set the value of a DMX channel (MASTER mode only)
     * @param channel DMX channel (1-512)
     * @param value Value (0-255)
     * @return Result indicating success or failure
     */
    Result set(uint16_t channel, uint8_t value) {
        if (_mode != Mode::MASTER) {
            return Error(WRONG_MODE);
        }
        if (channel < 1 || channel > DMX_UNIVERSE_SIZE) {
            return Error(CHANNEL_OVERFLOW);
        }
        
        // Update the value in the map
        _channels[channel] = value;
        return Success(1); // 1 channel written
    }

    /**
     * @brief Set multiple DMX channels at once (MASTER mode only)
     * @param channelValues Map of channel/value pairs
     * @return Result indicating success or failure and number of channels written
     */
    Result set(const std::map<uint16_t, uint8_t>& channelValues) {
        if (_mode != Mode::MASTER) {
            return Error(WRONG_MODE);
        }
        uint16_t nbChannelsWritten = 0;
        for (const auto& pair : channelValues) {
            if (pair.first >= 1 && pair.first <= DMX_UNIVERSE_SIZE) {
                _channels[pair.first] = pair.second;
                nbChannelsWritten++;
            }
        }
        return Success(nbChannelsWritten);
    }

    /**
     * @brief Set multiple DMX channels at once (MASTER mode only)
     * @param channelValues Map of channel/value pairs
     * @return Result indicating success or failure and number of channels written
     */
    Result set(const std::unordered_map<uint16_t, uint8_t>& channelValues) {
        if (_mode != Mode::MASTER) {
            return Error(WRONG_MODE);
        }
        uint16_t nbChannelsWritten = 0;
        for (const auto& pair : channelValues) {
            if (pair.first >= 1 && pair.first <= DMX_UNIVERSE_SIZE) {
                _channels[pair.first] = pair.second;
                nbChannelsWritten++;
            }
        }
        return Success(nbChannelsWritten);
    }

    /**
     * @brief Set multiple DMX channels from an array (MASTER mode only)
     * @param values Pointer to array of values
     * @param nbChannels Number of channels to set (max 512)
     * @param startChannel First channel to set (1-512)
     * @return Result indicating success or failure and number of channels written
     */
    Result set(uint8_t* values, uint16_t nbChannels, uint16_t startChannel = 1) {
        if (_mode != Mode::MASTER) {
            return Error(WRONG_MODE);
        }
        if (values == nullptr) {
            return Error(NULL_INPUT);
        }
        
        // Check & adjust startChannel
        if (startChannel < 1) {
            startChannel = 1;
        } else if (startChannel > DMX_UNIVERSE_SIZE) {
            return Error(CHANNEL_OVERFLOW);
        }
        
        // Calculate the maximum number of channels possible from startChannel
        uint16_t maxChannels = DMX_UNIVERSE_SIZE - startChannel + 1;
        
        // Limit the number of channels to copy
        nbChannels = std::min(nbChannels, maxChannels);
        
        // Copy the values into the channels starting from startChannel
        // We shift by 1 in _channels because index 0 is reserved for the start code
        memcpy(&_channels[startChannel], values, nbChannels);

        return Success(nbChannels);
    }

    /**
     * @brief Set multiple DMX channels from a vector (MASTER mode only)
     * @param values Vector of values to set
     * @param startChannel First channel to set (1-512, will be clamped if out of range)
     */
    Result set(const std::vector<uint8_t>& values, uint16_t startChannel = 1) {
        if (_mode != Mode::MASTER) {
            return Error(WRONG_MODE);
        }
        
        // Check & adjust startChannel
        if (startChannel < 1) {
            startChannel = 1;
        } else if (startChannel > DMX_UNIVERSE_SIZE) {
            return Error(CHANNEL_OVERFLOW);
        }
        
        // Calculate the maximum number of channels possible from startChannel
        uint16_t maxChannels = DMX_UNIVERSE_SIZE - startChannel + 1;
        
        // Limit the number of channels to copy
        uint16_t nbChannels = std::min((uint16_t)values.size(), maxChannels);
        
        // Copy the values into the channels starting from startChannel
        // We shift by 1 in _channels because index 0 is reserved for the start code
        memcpy(&_channels[startChannel], values.data(), nbChannels);
        return Success(nbChannels);
    }

    /**
     * @brief Get the current value of a DMX channel
     * @param channel DMX channel (1-512)
     * @param value Reference to store the channel value
     * @param timestamp Optional pointer to store the last update timestamp (SLAVE mode only)
     * @return Result indicating success or failure
     */
    Result get(uint16_t channel, uint8_t& value, uint64_t* timestamp = nullptr) const {
        if (channel < 1 || channel > DMX_UNIVERSE_SIZE) {
            value = 0;
            if (timestamp) *timestamp = 0;
            return Error(CHANNEL_OVERFLOW);
        }

        if (_mode == Mode::MASTER) {
            value = _channels[channel];
            if (timestamp) *timestamp = 0;
        } else {
            portENTER_CRITICAL(&_dmxMux);
            value = _slaveValues[channel - 1];
            if (timestamp) *timestamp = _lastFrameTimestamp;
            portEXIT_CRITICAL(&_dmxMux);
        }
        return Success(1); // 1 channel read
    }

    /**
     * @brief Get direct access to all channel values
     * @param values Pointer to array to store values (size DMX_UNIVERSE_SIZE minimum)
     * @param timestamp Optional pointer to store the last update timestamp
     * @return Result indicating success or failure and number of channels read
     */
    Result getAllChannels(uint8_t* values, uint64_t* timestamp = nullptr) const {
        if (values == nullptr) {
            return Error(NULL_INPUT);
        }

        if (_mode == Mode::MASTER) {
            // In MASTER mode, copy from _channels[1] since index 0 is start code
            memcpy(values, &_channels[1], DMX_UNIVERSE_SIZE);
            if (timestamp) *timestamp = 0;
        } else {
            portENTER_CRITICAL(&_dmxMux);
            memcpy(values, _slaveValues, DMX_UNIVERSE_SIZE);
            if (timestamp) *timestamp = _lastFrameTimestamp;
            portEXIT_CRITICAL(&_dmxMux);
        }
        return Success(DMX_UNIVERSE_SIZE);
    }

    /**
     * @brief Reset all channels to zero
     */
    Result resetAllChannels() {
        // Reset all values to zero
        memset(_channels, 0, DMX_UNIVERSE_SIZE + 1);
        // Also clear slave buffer so that values are predictable before first frame is received
        memset(_slaveValues, 0, DMX_UNIVERSE_SIZE);
        return Success();
    }

    /**
     * @brief Check if the DMX transmission is running
     * @return true if the DMX transmission is running, false otherwise
     */
    bool isRunning() const {
        return _running;
    }

    // Getter pour le timestamp de la dernière frame
    uint64_t getLastFrameTimestamp() const {
        portENTER_CRITICAL(&_dmxMux);
        uint64_t timestamp = _lastFrameTimestamp;
        portEXIT_CRITICAL(&_dmxMux);
        return timestamp;
    }

    /**
     * @brief Wait for the next good frame and copy it (SLAVE mode only)
     *
     * Must not be blocked in this call while another task calls stop().
     *
     * @param values Pointer to array to store values (size DMX_UNIVERSE_SIZE minimum)
     * @param timeoutMs Maximum wait
     * @param slots Optional: slot count of the frame (values past it keep their previous content)
     * @param sequence Optional: value of SlaveStats::frames for this frame, to detect frames skipped by the caller
     * @return Result indicating success, TIMEOUT if no frame arrived
     */
    Result waitFrame(uint8_t* values, uint32_t timeoutMs, uint16_t* slots = nullptr, uint32_t* sequence = nullptr) {
        if (_mode != Mode::SLAVE) {
            return Error(WRONG_MODE);
        }
        if (values == nullptr) {
            return Error(NULL_INPUT);
        }
        if (!_running || _rxFrameReady == nullptr) {
            return Error(NOT_RUNNING);
        }
        if (xSemaphoreTake(_rxFrameReady, pdMS_TO_TICKS(timeoutMs)) != pdTRUE) {
            return Error(TIMEOUT);
        }
        portENTER_CRITICAL(&_dmxMux);
        memcpy(values, _slaveValues, DMX_UNIVERSE_SIZE);
        if (slots) *slots = _stats.lastSlots;
        if (sequence) *sequence = _stats.frames;
        portEXIT_CRITICAL(&_dmxMux);
        return Success(DMX_UNIVERSE_SIZE);
    }

    /**
     * @brief Read the SLAVE reception counters
     */
    Result getStats(SlaveStats& stats) const {
        if (_mode != Mode::SLAVE) {
            return Error(WRONG_MODE);
        }
        uint64_t periodSum, breakSum, mabSum, startLowSum;
        uint32_t breakMin, breakMax, mabMin, mabMax, startLowMin, startLowMax, periodSamples;

        portENTER_CRITICAL(&_dmxMux);
        stats = _stats;
        periodSum = _periodSumUs;
        periodSamples = _periodSamples;
        breakSum = _breakSum;
        breakMin = _breakMin;
        breakMax = _breakMax;
        mabSum = _mabSum;
        mabMin = _mabMin;
        mabMax = _mabMax;
        startLowSum = _startLowSum;
        startLowMin = _startLowMin;
        startLowMax = _startLowMax;
        portEXIT_CRITICAL(&_dmxMux);

        if (periodSamples > 0) {
            stats.periodAvgUs = (uint32_t)(periodSum / periodSamples);
        }
        if (stats.timingSamples > 0) {
            const float ticksPerUs = (float)esp_rom_get_cpu_ticks_per_us();
            const float samples = (float)stats.timingSamples;
            stats.breakMinUs = breakMin / ticksPerUs;
            stats.breakMaxUs = breakMax / ticksPerUs;
            stats.breakAvgUs = (float)breakSum / samples / ticksPerUs;
            stats.mabMinUs = mabMin / ticksPerUs;
            stats.mabMaxUs = mabMax / ticksPerUs;
            stats.mabAvgUs = (float)mabSum / samples / ticksPerUs;
            stats.startLowMinUs = startLowMin / ticksPerUs;
            stats.startLowMaxUs = startLowMax / ticksPerUs;
            stats.startLowAvgUs = (float)startLowSum / samples / ticksPerUs;
        }
        return Success();
    }

    /**
     * @brief Reset the SLAVE reception counters
     */
    Result resetStats() {
        if (_mode != Mode::SLAVE) {
            return Error(WRONG_MODE);
        }
        portENTER_CRITICAL(&_dmxMux);
        clearStatsLocked();
        portEXIT_CRITICAL(&_dmxMux);
        return Success();
    }

private:    
    Mode _mode;
    uart_port_t _uart_num;
    int _txPin;
    int _rxPin;
    int _dePin;
    bool _begun = false;
    volatile bool _running;
    volatile TaskHandle_t _transmitTaskHandle;
    int _txCore = tskNO_AFFINITY;
    mutable portMUX_TYPE _dmxMux = portMUX_INITIALIZER_UNLOCKED;

    // esp_timer one-shot timer for DMX transmission timing
    esp_timer_handle_t _dmxTimer;

    // Performance measurement variables for timer precision
    uint64_t _timerStartTime;
    uint32_t _frameCounter;
    bool _enablePerfLog;

    // Frame-to-frame performance measurement
    static constexpr size_t FRAME_WINDOW = 20;
    uint64_t _frameIntervals[FRAME_WINDOW] = {0};
    size_t _frameIntervalsIndex;
    size_t _frameIntervalsCount;
    uint64_t _prevFrameSendTime;

    // Storage of DMX payload (0 = start code, 1..512 = channels)
    uint8_t _channels[DMX_UNIVERSE_SIZE + 1] = {0};

    // Storage for SLAVE mode
    uint8_t _slaveValues[DMX_UNIVERSE_SIZE] = {0};
    uint64_t _lastFrameTimestamp;

    // SLAVE reception state, owned by SlaveIsr while running
    static constexpr int16_t RX_WAIT_BREAK = -1;   // No BREAK seen yet: bytes are not framed
    static constexpr int16_t RX_FRAME_FULL = -2;   // 513 bytes received: waiting for the next BREAK
    static constexpr uint32_t RX_FIFO_THRESHOLD = 32;  // Interrupt rate vs FIFO margin (128 bytes on the S3)
    static constexpr uint32_t RX_TIMEOUT_BITS = 22;    // Drain the FIFO after about 2 idle slots
    static constexpr uint32_t BREAK_MIN_PROBE_US = 40; // Longer than any data low time (0x00 is 36 us low)

    uint8_t _rxFrame[DMX_UNIVERSE_SIZE + 1] = {0};
    int16_t _rxHead = RX_WAIT_BREAK;
    bool _rxBad = false;
    bool _rxOverlong = false;
    bool _rxFramingPending = false;
    bool _rxFramePublished = false;
    int16_t _rxExpectedLen = 0;
    int64_t _rxLastBreakUs = 0;
    intr_handle_t _rxIntr = nullptr;
    SemaphoreHandle_t _rxFrameReady = nullptr;

    SlaveStats _stats = {};
    uint64_t _periodSumUs = 0;
    uint32_t _periodSamples = 0;

    // Line timing probe, in CPU cycles
    bool _timingEnabled = false;
    bool _probeHooked = false;
    bool _probeArmed = false;
    uint8_t _probeStage = 0;
    uint32_t _probeFallTicks = 0;
    uint32_t _probeBreakTicks = 0;
    uint32_t _probeMabTicks = 0;
    uint32_t _probeBreakMinTicks = 0;
    uint32_t _breakMin = 0, _breakMax = 0;
    uint32_t _mabMin = 0, _mabMax = 0;
    uint32_t _startLowMin = 0, _startLowMax = 0;
    uint64_t _breakSum = 0, _mabSum = 0, _startLowSum = 0;

    /**
     * @brief Timer callback for DMX transmission timing
     */
    static void TimerCallback(void* arg) {
        EZDMX* dmx = static_cast<EZDMX*>(arg);
        TaskHandle_t task = dmx ? dmx->_transmitTaskHandle : nullptr;
        if (task) {
            // Measure timer precision
            uint64_t currentTime = esp_timer_get_time();
            if (dmx->_timerStartTime > 0) {
                uint64_t actualInterval = currentTime - dmx->_timerStartTime;
                uint64_t expectedInterval = DMX_MTBF_MS * 1000; // Convert to microseconds
                int64_t jitter = actualInterval - expectedInterval;
                
                // Print performance measurement every 100 frames to avoid spam
                if (dmx->_enablePerfLog && dmx->_frameCounter % 100 == 0) {
                    #ifdef ARDUINO
                        Serial.printf("DMX Timer Performance - Frame %u: Expected=%lluus, Actual=%lluus, Jitter=%lldus\n", 
                                    dmx->_frameCounter, expectedInterval, actualInterval, jitter);
                    #else
                        printf("DMX Timer Performance - Frame %u: Expected=%lluus, Actual=%lluus, Jitter=%lldus\n", 
                               dmx->_frameCounter, expectedInterval, actualInterval, jitter);
                    #endif
                }
            }
            
            // Notify the transmission task to send next frame
            xTaskNotifyGive(task);
        }
    }

    /**
     * @brief DMX transmission task
     */
    static void TransmitTask(void* pEZDMX) {
        EZDMX* dmx = static_cast<EZDMX*>(pEZDMX);
        
        // Send first frame immediately
        bool firstFrame = true;
        
        while (dmx->_running) {
            if (!dmx->_running) break;
            if (!firstFrame) {
                // Wait for timer notification (except for first frame)
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                if (!dmx->_running) break;
            }
            firstFrame = false;

            // Enter critical section for break and MAB
            portENTER_CRITICAL(&dmx->_dmxMux);
            
            // 1. Generate the break
            uart_set_line_inverse(dmx->_uart_num, UART_SIGNAL_TXD_INV);
            ets_delay_us(DMX_BREAK_US);
            
            // 2. Mark After Break (MAB)
            uart_set_line_inverse(dmx->_uart_num, UART_SIGNAL_INV_DISABLE);
            ets_delay_us(DMX_MAB_US);
            
            // Exit critical section before sending data
            portEXIT_CRITICAL(&dmx->_dmxMux);
            
            // 3. Send the DMX packet
            uart_write_bytes(dmx->_uart_num, (const char*)dmx->_channels, DMX_UNIVERSE_SIZE + 1);
            
            // 4. Wait for all data to be sent
            uart_wait_tx_done(dmx->_uart_num, pdMS_TO_TICKS(100));
            
            // === Frame-to-frame performance measurement ===
            uint64_t nowFrameTime = esp_timer_get_time();
            if (dmx->_prevFrameSendTime != 0) {
                uint64_t interval = nowFrameTime - dmx->_prevFrameSendTime; // µs

                // Store interval in sliding window
                dmx->_frameIntervals[dmx->_frameIntervalsIndex] = interval;
                dmx->_frameIntervalsIndex = (dmx->_frameIntervalsIndex + 1) % FRAME_WINDOW;
                if (dmx->_frameIntervalsCount < FRAME_WINDOW) dmx->_frameIntervalsCount++;

                // Compute moving average when window filled enough
                if (dmx->_enablePerfLog && dmx->_frameIntervalsCount == FRAME_WINDOW && dmx->_frameCounter % 100 == 0) {
                    uint64_t sum = 0;
                    for (size_t i = 0; i < FRAME_WINDOW; ++i) sum += dmx->_frameIntervals[i];
                    uint64_t avgInterval = sum / FRAME_WINDOW;
                    int64_t jitter = (int64_t)interval - (int64_t)avgInterval;

                    #ifdef ARDUINO
                        Serial.printf("DMX Frame Rate - Frame %u: Interval=%lluus, Avg=%lluus, Jitter=%lldus\n",
                                        dmx->_frameCounter, interval, avgInterval, jitter);
                    #else
                        printf("DMX Frame Rate - Frame %u: Interval=%lluus, Avg=%lluus, Jitter=%lldus\n",
                               dmx->_frameCounter, interval, avgInterval, jitter);
                    #endif
                }
            }

            dmx->_prevFrameSendTime = nowFrameTime;
            // Incrément du compteur de frame
            dmx->_frameCounter++;
            
            // 5. Start timer for MTBF (Mark Time Between Frames)
            if (dmx->_running && dmx->_dmxTimer) {
                // Record timer start time for performance measurement
                dmx->_timerStartTime = esp_timer_get_time();
                esp_timer_start_once(dmx->_dmxTimer, DMX_MTBF_MS * 1000);
            }
        }
        dmx->_transmitTaskHandle = nullptr;
        vTaskDelete(NULL);
    }

    Result startSlave() {
        _rxFrameReady = xSemaphoreCreateBinary();
        if (_rxFrameReady == nullptr) {
            return Error(TASK_FAILED);
        }

        uart_dev_t* hw = UART_LL_GET_HW(_uart_num);
        uart_ll_disable_intr_mask(hw, UART_LL_INTR_MASK);
        uart_ll_clr_intsts_mask(hw, UART_LL_INTR_MASK);
        uart_ll_rxfifo_rst(hw);
        // The probe arms itself on the last bytes of a frame, so it needs them one by one
        uart_ll_set_rxfifo_full_thr(hw, _timingEnabled ? 1 : RX_FIFO_THRESHOLD);
        uart_ll_set_rx_tout(hw, RX_TIMEOUT_BITS);

        portENTER_CRITICAL(&_dmxMux);
        clearStatsLocked();
        _rxHead = RX_WAIT_BREAK;
        _rxBad = false;
        _rxOverlong = false;
        _rxFramingPending = false;
        _rxFramePublished = false;
        _rxExpectedLen = 0;
        _rxLastBreakUs = 0;
        _probeArmed = false;
        _probeStage = 0;
        portEXIT_CRITICAL(&_dmxMux);

        if (_timingEnabled) {
            Result probe = hookProbe();
            if (probe != SUCCESS) {
                vSemaphoreDelete(_rxFrameReady);
                _rxFrameReady = nullptr;
                return probe;
            }
        }

        _running = true;
        esp_err_t err = esp_intr_alloc(uart_periph_signal[_uart_num].irq, 0, SlaveIsr, this, &_rxIntr);
        if (err != ESP_OK) {
            _running = false;
            unhookProbe();
            vSemaphoreDelete(_rxFrameReady);
            _rxFrameReady = nullptr;
            return Error(UART_ERROR);
        }
        uart_ll_ena_intr_mask(hw, UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT | UART_INTR_RXFIFO_OVF |
                                  UART_INTR_BRK_DET | UART_INTR_FRAM_ERR);
        return Success();
    }

    void stopSlave() {
        uart_dev_t* hw = UART_LL_GET_HW(_uart_num);
        uart_ll_disable_intr_mask(hw, UART_LL_INTR_MASK);
        if (_rxIntr != nullptr) {
            esp_intr_free(_rxIntr);
            _rxIntr = nullptr;
        }
        uart_ll_clr_intsts_mask(hw, UART_LL_INTR_MASK);
        unhookProbe();
        if (_rxFrameReady != nullptr) {
            vSemaphoreDelete(_rxFrameReady);
            _rxFrameReady = nullptr;
        }
    }

    Result hookProbe() {
        if (_rxPin < 0) {
            return Error(GPIO_ERROR);
        }
        _probeBreakMinTicks = BREAK_MIN_PROBE_US * esp_rom_get_cpu_ticks_per_us();
        esp_err_t err = gpio_install_isr_service(0);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {  // Already installed is fine
            return Error(GPIO_ERROR);
        }
        gpio_intr_disable((gpio_num_t)_rxPin);
        gpio_set_intr_type((gpio_num_t)_rxPin, GPIO_INTR_ANYEDGE);
        err = gpio_isr_handler_add((gpio_num_t)_rxPin, ProbeIsr, this);
        if (err != ESP_OK) {
            gpio_set_intr_type((gpio_num_t)_rxPin, GPIO_INTR_DISABLE);
            return Error(GPIO_ERROR);
        }
        _probeHooked = true;
        return Success();
    }

    void unhookProbe() {
        if (!_probeHooked) {
            return;
        }
        gpio_intr_disable((gpio_num_t)_rxPin);
        gpio_isr_handler_remove((gpio_num_t)_rxPin);
        gpio_set_intr_type((gpio_num_t)_rxPin, GPIO_INTR_DISABLE);
        _probeHooked = false;
    }

    void clearStatsLocked() {
        _stats = {};
        _periodSumUs = 0;
        _periodSamples = 0;
        _breakMin = _breakMax = 0;
        _mabMin = _mabMax = 0;
        _startLowMin = _startLowMax = 0;
        _breakSum = _mabSum = _startLowSum = 0;
    }

    /**
     * @brief SLAVE reception, framed on the hardware BREAK detection
     *
     * Runs instead of the IDF UART driver. With the driver, a BREAK event overtakes the
     * bytes still in the hardware FIFO, so with frames sent back to back the end of a
     * frame and the start of the next cannot be told apart. Here the FIFO is read when
     * the BREAK is handled: what it holds belongs to the frame that just ended, except
     * for its last byte, the 0x00 the BREAK itself produces.
     *
     * A value of 0x00 in the data is never mistaken for a BREAK.
     */
    static void SlaveIsr(void* arg) {
        EZDMX* dmx = static_cast<EZDMX*>(arg);
        uart_dev_t* hw = UART_LL_GET_HW(dmx->_uart_num);
        bool frameReady = false;

        portENTER_CRITICAL_ISR(&dmx->_dmxMux);
        uint32_t status;
        while ((status = uart_ll_get_intsts_mask(hw)) != 0) {
            uint32_t pending = uart_ll_get_rxfifo_len(hw);

            if (status & UART_INTR_RXFIFO_OVF) {
                uart_ll_rxfifo_rst(hw);
                pending = 0;
                dmx->_stats.fifoOverflows++;
                dmx->_rxBad = true;
            }

            if (status & UART_INTR_BRK_DET) {
                if (pending > 0) {
                    dmx->drainFifo(hw, pending - 1);  // The last byte is the BREAK's own 0x00
                }
                uart_ll_rxfifo_rst(hw);
                dmx->finishFrame();
                dmx->startFrame();
            } else {
                // A framing error only counts once the line has carried on with more data:
                // the BREAK raises one too, and it may be reported first
                if (dmx->_rxFramingPending && pending > 0) {
                    dmx->_rxFramingPending = false;
                    dmx->_stats.framingErrors++;
                    dmx->_rxBad = true;
                }
                if (status & UART_INTR_FRAM_ERR) {
                    dmx->_rxFramingPending = true;
                }
                dmx->drainFifo(hw, pending);
            }

            // Cleared last: the FIFO interrupts stay raised until the FIFO has been read
            uart_ll_clr_intsts_mask(hw, status);
        }
        frameReady = dmx->_rxFramePublished;
        dmx->_rxFramePublished = false;
        portEXIT_CRITICAL_ISR(&dmx->_dmxMux);

        if (frameReady) {
            BaseType_t woken = pdFALSE;
            xSemaphoreGiveFromISR(dmx->_rxFrameReady, &woken);
            if (woken == pdTRUE) {
                portYIELD_FROM_ISR();
            }
        }
    }

    void drainFifo(uart_dev_t* hw, uint32_t count) {
        while (count-- > 0) {
            uint8_t byte;
            uart_ll_read_rxfifo(hw, &byte, 1);

            if (_rxHead == RX_FRAME_FULL) {
                _rxOverlong = true;
            }
            if (_rxHead < 0) {
                continue;
            }
            _rxFrame[_rxHead++] = byte;

            // The GPIO probe has to be listening before the next BREAK falls. Armed one slot
            // early since the last stop bits leave only a few microseconds. It skips data edges.
            if (_timingEnabled && !_probeArmed && _rxExpectedLen > 1 && _rxHead == _rxExpectedLen - 1) {
                _probeArmed = true;
                _probeStage = 0;
                gpio_intr_enable((gpio_num_t)_rxPin);
            }

            if (_rxHead == (int16_t)(DMX_UNIVERSE_SIZE + 1)) {
                publishFrame(_rxHead);
                _rxHead = RX_FRAME_FULL;
            }
        }
    }

    // Called on a BREAK: a frame shorter than the universe ends here
    void finishFrame() {
        if (_rxHead == 0) {
            _stats.emptyFrames++;
        } else if (_rxHead > 0) {
            publishFrame(_rxHead);
        }
        if (_rxOverlong) {
            _stats.overlongFrames++;
        }
    }

    void publishFrame(int16_t length) {
        _rxExpectedLen = length;
        if (_rxBad) {
            _stats.errorFrames++;
            return;
        }
        if (_rxFrame[0] != 0) {
            _stats.startCodeFrames++;
            return;
        }

        const uint16_t slots = length - 1;
        memcpy(_slaveValues, &_rxFrame[1], slots);
        _lastFrameTimestamp = esp_timer_get_time() / 1000;
        _stats.frames++;
        _stats.lastSlots = slots;
        if (slots < DMX_UNIVERSE_SIZE) {
            _stats.shortFrames++;
        }
        if (_stats.frames == 1 || slots < _stats.minSlots) _stats.minSlots = slots;
        if (slots > _stats.maxSlots) _stats.maxSlots = slots;
        _rxFramePublished = true;
    }

    void startFrame() {
        const int64_t now = esp_timer_get_time();
        _stats.breaks++;
        if (_rxLastBreakUs != 0) {
            const uint32_t period = (uint32_t)(now - _rxLastBreakUs);
            if (_periodSamples == 0 || period < _stats.periodMinUs) _stats.periodMinUs = period;
            if (period > _stats.periodMaxUs) _stats.periodMaxUs = period;
            _periodSumUs += period;
            _periodSamples++;
        }
        _rxLastBreakUs = now;

        _rxHead = 0;
        _rxBad = false;
        _rxOverlong = false;
        _rxFramingPending = false;

        // The BREAK has been low for one character by now: the probe must have seen it fall
        if (_probeArmed && _probeStage == 0) {
            gpio_intr_disable((gpio_num_t)_rxPin);
            _probeArmed = false;
            _stats.timingMissed++;
        }
    }

    /**
     * @brief Line timing probe: BREAK fall, BREAK rise, start code fall, start code rise
     *
     * Enabled by SlaveIsr only around the BREAK, then disabled, so that the data of a busy
     * universe does not load the CPU. Reads the level instead of trusting the edge order, and
     * ignores any low time too short to be a BREAK.
     */
    static void ProbeIsr(void* arg) {
        EZDMX* dmx = static_cast<EZDMX*>(arg);
        const uint32_t now = esp_cpu_get_cycle_count();
        const int level = gpio_get_level((gpio_num_t)dmx->_rxPin);

        portENTER_CRITICAL_ISR(&dmx->_dmxMux);
        switch (dmx->_probeStage) {
            case 0:  // Waiting for the BREAK to fall
                if (level == 0) {
                    dmx->_probeFallTicks = now;
                    dmx->_probeStage = 1;
                }
                break;
            case 1:  // Waiting for the BREAK to rise
                if (level == 1) {
                    const uint32_t low = now - dmx->_probeFallTicks;
                    if (low < dmx->_probeBreakMinTicks) {
                        dmx->_probeStage = 0;  // Still in the data
                    } else {
                        dmx->_probeBreakTicks = low;
                        dmx->_probeFallTicks = now;
                        dmx->_probeStage = 2;
                    }
                }
                break;
            case 2:  // Waiting for the end of the MAB
                if (level == 0) {
                    dmx->_probeMabTicks = now - dmx->_probeFallTicks;
                    dmx->_probeFallTicks = now;
                    dmx->_probeStage = 3;
                }
                break;
            case 3:  // Waiting for the end of the start code low time
                if (level == 1) {
                    dmx->recordTiming(dmx->_probeBreakTicks, dmx->_probeMabTicks, now - dmx->_probeFallTicks);
                    gpio_intr_disable((gpio_num_t)dmx->_rxPin);
                    dmx->_probeArmed = false;
                    dmx->_probeStage = 0;
                }
                break;
        }
        portEXIT_CRITICAL_ISR(&dmx->_dmxMux);
    }

    void recordTiming(uint32_t breakTicks, uint32_t mabTicks, uint32_t startLowTicks) {
        const bool first = (_stats.timingSamples == 0);
        if (first || breakTicks < _breakMin) _breakMin = breakTicks;
        if (breakTicks > _breakMax) _breakMax = breakTicks;
        if (first || mabTicks < _mabMin) _mabMin = mabTicks;
        if (mabTicks > _mabMax) _mabMax = mabTicks;
        if (first || startLowTicks < _startLowMin) _startLowMin = startLowTicks;
        if (startLowTicks > _startLowMax) _startLowMax = startLowTicks;
        _breakSum += breakTicks;
        _mabSum += mabTicks;
        _startLowSum += startLowTicks;
        _stats.timingSamples++;
    }

#ifdef ARDUINO
    /**
     * @brief Convert an Arduino serial port to an IDF UART port
     * @param serial Pointer to the Arduino serial port to convert
     * @return The corresponding IDF UART port
     */
    static uart_port_t serialToUartPort(Stream* serial) {
        if (serial == &Serial) return UART_NUM_0;
        else if (serial == &Serial1) return UART_NUM_1;

        // Enable Serial2 for ESP32 & ESP32S3
        #if CONFIG_IDF_TARGET_ESP32S3 == 1
            else if (serial == &Serial2) return UART_NUM_2;
        #elif CONFIG_IDF_TARGET_ESP32 == 1
            else if (serial == &Serial2) return UART_NUM_2;
        #endif

        return UART_NUM_MAX;
    }
#endif

};

#endif // EZDMX_H