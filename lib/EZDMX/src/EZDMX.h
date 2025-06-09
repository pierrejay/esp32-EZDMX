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
#include <driver/gpio.h>
#include <esp_intr_alloc.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
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

#ifdef ARDUINO
    /**
     * @brief Constructor
     * @param mode Mode of operation (MASTER or SLAVE)
     * @param serial Pointer to the Arduino serial port to use (&Serial, &Serial1, etc.)
     * @param txPin DMX transmission pin
     * @param rxPin DMX reception pin (not used in master mode but required for configuration)
     * @param dePin Pin to activate RS485 transmitter (optional, -1 if not used)
     * @param enablePerfLog Enable performance logging (optional, false if not used)
     */
    EZDMX(Mode mode = Mode::MASTER, Stream* serial = &Serial1, int txPin = 17, int rxPin = 16, int dePin = -1, bool enablePerfLog = false) :
        _mode(mode),
        _uart_num(serialToUartPort(serial)),
        _txPin(txPin),
        _rxPin(rxPin),
        _dePin(dePin),
        _running(false),
        _transmitTaskHandle(nullptr),
        _receiveTaskHandle(nullptr),
        _lastFrameTimestamp(0),
        _dmxTimer(nullptr),
        _timerStartTime(0),
        _frameCounter(0),
        _enablePerfLog(enablePerfLog)
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
        _receiveTaskHandle(nullptr),
        _lastFrameTimestamp(0),
        _dmxTimer(nullptr),
        _timerStartTime(0),
        _frameCounter(0),
        _enablePerfLog(enablePerfLog)
    {
        // Initialize channels to 0
        resetAllChannels();
    }


    ~EZDMX() {
        stop();
    }

    /**
     * @brief Initialize the UART for DMX
     * @return true if initialization succeeds
     */
    Result begin() {
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

        // Install the UART driver
        esp_err_t err = uart_driver_install(_uart_num, DMX_BUFFER_SIZE, DMX_BUFFER_SIZE, 0, NULL, ESP_INTR_FLAG_IRAM);
        if (err != ESP_OK) {
            return Error(UART_ERROR);
        }

        err = uart_param_config(_uart_num, &uart_config);
        if (err != ESP_OK) {
            uart_driver_delete(_uart_num);
            return Error(UART_ERROR);
        }

        err = uart_set_pin(_uart_num, _txPin, _rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (err != ESP_OK) {
            uart_driver_delete(_uart_num);
            return Error(GPIO_ERROR);
        }

        if (_mode == Mode::MASTER) {
            // Configure RS485 DE pin if needed
            if (_dePin >= 0) {
                gpio_config_t io_conf = {};
                io_conf.intr_type = GPIO_INTR_DISABLE;
                io_conf.mode = GPIO_MODE_OUTPUT;
                io_conf.pin_bit_mask = (1ULL << _dePin);
                io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
                io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
                
                err = gpio_config(&io_conf);
                if (err != ESP_OK) {
                    uart_driver_delete(_uart_num);
                    return Error(GPIO_ERROR);
                }
                
                err = gpio_set_level((gpio_num_t)_dePin, 1);
                if (err != ESP_OK) {
                    uart_driver_delete(_uart_num);
                    return Error(GPIO_ERROR);
                }
            }
        } else {
            // In SLAVE mode, disable the DE pin if present
            if (_dePin >= 0) {
                gpio_set_level((gpio_num_t)_dePin, 0);
            }
        }

        return Success();
    }

    /**
     * @brief Start the DMX transmission
     * @return Result indicating success or failure
     */
    Result start() {
        if (_running) {
            return Success();
        }

        _running = true;
        if (_mode == Mode::MASTER) {
            // Create the transmission task
            BaseType_t ret = xTaskCreate(TransmitTask, 
                        "dmx_tx", 
                        8192, 
                        this, 
                        5, 
                        &_transmitTaskHandle);
            if (ret != pdPASS) {
                _running = false;
                return Error(TASK_FAILED);
            }
            
            // Create the esp_timer for precise DMX timing (one-shot)
            esp_timer_create_args_t timer_args = {
                .callback = TimerCallback,
                .arg = this,
                .name = "dmx_timer"
            };
            esp_err_t errTimer = esp_timer_create(&timer_args, &_dmxTimer);
            if (errTimer != ESP_OK || _dmxTimer == nullptr) {
                _running = false;
                vTaskDelete(_transmitTaskHandle);
                _transmitTaskHandle = nullptr;
                return Error(TASK_FAILED);
            }
            
        } else if (_mode == Mode::SLAVE) {
            BaseType_t ret = xTaskCreate(ReceiveTask, 
                        "dmx_rx", 
                        8192, 
                        this, 
                        5, 
                        &_receiveTaskHandle);
            if (ret != pdPASS) {
                _running = false;
                return Error(TASK_FAILED);
            }
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
            // Stop and delete the timer
            if (_dmxTimer != nullptr) {
                esp_timer_stop(_dmxTimer);
                esp_timer_delete(_dmxTimer);
                _dmxTimer = nullptr;
            }
            
            // Notify the task to wake up and exit cleanly
            if (_transmitTaskHandle != nullptr) {
                xTaskNotifyGive(_transmitTaskHandle);
                vTaskDelay(pdMS_TO_TICKS(50));
                _transmitTaskHandle = nullptr;
            }
        } else {
            if (_receiveTaskHandle != nullptr) {
                vTaskDelay(pdMS_TO_TICKS(50));
                _receiveTaskHandle = nullptr;
            }
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
            value = _slaveValues[channel - 1];
            if (timestamp) *timestamp = _lastFrameTimestamp;
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
            memcpy(values, _slaveValues, DMX_UNIVERSE_SIZE);
            if (timestamp) *timestamp = _lastFrameTimestamp;
        }
        return Success(DMX_UNIVERSE_SIZE);
    }

    /**
     * @brief Reset all channels to zero
     */
    Result resetAllChannels() {
        // Reset all values to zero
        memset(_channels, 0, DMX_UNIVERSE_SIZE + 1);
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
        return _lastFrameTimestamp;
    }

private:    
    Mode _mode;
    uart_port_t _uart_num;
    int _txPin;
    int _rxPin;
    int _dePin;
    bool _running;
    TaskHandle_t _transmitTaskHandle;
    TaskHandle_t _receiveTaskHandle;
    portMUX_TYPE _dmxMux = portMUX_INITIALIZER_UNLOCKED;

    // esp_timer one-shot timer for DMX transmission timing
    esp_timer_handle_t _dmxTimer;

    // Performance measurement variables for timer precision
    uint64_t _timerStartTime;
    uint32_t _frameCounter;
    bool _enablePerfLog;

    // Storage of DMX payload (0 = start code, 1..512 = channels)
    uint8_t _channels[DMX_UNIVERSE_SIZE + 1] = {0};

    // Storage for SLAVE mode
    uint8_t _slaveValues[DMX_UNIVERSE_SIZE];
    uint64_t _lastFrameTimestamp;

    /**
     * @brief Timer callback for DMX transmission timing
     */
    static void TimerCallback(void* arg) {
        EZDMX* dmx = static_cast<EZDMX*>(arg);
        if (dmx && dmx->_transmitTaskHandle) {
            // Measure timer precision
            uint64_t currentTime = esp_timer_get_time();
            if (dmx->_timerStartTime > 0) {
                uint64_t actualInterval = currentTime - dmx->_timerStartTime;
                uint64_t expectedInterval = DMX_MTBF_MS * 1000; // Convert to microseconds
                int64_t jitter = actualInterval - expectedInterval;
                
                dmx->_frameCounter++;
                // Print performance measurement every 100 frames to avoid spam
                if (dmx->_frameCounter % 100 == 0 && dmx->_enablePerfLog) {
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
            xTaskNotifyGive(dmx->_transmitTaskHandle);
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
            
            // 5. Start timer for MTBF (Mark Time Between Frames)
            if (dmx->_running && dmx->_dmxTimer) {
                // Record timer start time for performance measurement
                dmx->_timerStartTime = esp_timer_get_time();
                esp_timer_start_once(dmx->_dmxTimer, DMX_MTBF_MS * 1000);
            }
        }
        vTaskDelete(NULL);
    }

    static void ReceiveTask(void* pEZDMX) {
        EZDMX* dmx = static_cast<EZDMX*>(pEZDMX);
        uint8_t buffer[DMX_UNIVERSE_SIZE + 1];
        
        while (dmx->_running) {
            // Attendre le BREAK
            while (dmx->_running) {
                uint8_t byte;
                if (uart_read_bytes(dmx->_uart_num, &byte, 1, pdMS_TO_TICKS(100)) == 1) {
                    if (byte == 0) {
                        // Potentiel BREAK détecté
                        break;
                    }
                }
            }

            // Lire le paquet DMX complet
            size_t length = uart_read_bytes(dmx->_uart_num, buffer, DMX_UNIVERSE_SIZE + 1, pdMS_TO_TICKS(100));
            
            if (length == DMX_UNIVERSE_SIZE + 1 && buffer[0] == 0) {
                uint64_t now = esp_timer_get_time() / 1000;
                
                portENTER_CRITICAL(&dmx->_dmxMux);
                // Mettre à jour le timestamp
                dmx->_lastFrameTimestamp = now;
                
                // Mettre à jour les canaux
                for (size_t i = 0; i < DMX_UNIVERSE_SIZE; i++) {
                    if (dmx->_slaveValues[i] != buffer[i + 1]) {
                        dmx->_slaveValues[i] = buffer[i + 1];
                    }
                }
                portEXIT_CRITICAL(&dmx->_dmxMux);
            }
        }
        
        vTaskDelete(NULL);
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