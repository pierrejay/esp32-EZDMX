#ifndef EZDMX_H
#define EZDMX_H

#ifdef ARDUINO
    #include <Arduino.h>
#endif
#include <cstring>
#include <algorithm>
#include <map>
#include <vector>
#include <driver/uart.h>
#include <soc/uart_struct.h>
#include <driver/gpio.h>
#include <esp_intr_alloc.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class EZDMX {
public:
    enum class Mode {
        MASTER,
        SLAVE
    };

    enum Status {
        SUCCESS = 0,           // Operation successful
        ERR_UART,              // UART initialization or operation error
        ERR_GPIO,              // GPIO configuration error (TX, RX, DE pins)
        ERR_WRONG_MODE,        // Operation not allowed in current mode (MASTER/SLAVE)
        ERR_CHANNEL_OVERFLOW,  // Channel number out of limits (>512 or <1)
        ERR_NULL_INPUT,        // Null pointer provided as parameter
        ERR_TASK_FAILED,       // Task creation/execution failed
        ERR_NOT_RUNNING,       // Operation impossible because DMX is not started
        ERR_FRAME_ERROR,       // DMX frame error (missing break, invalid start code...)
        ERR_TIMEOUT,           // Timeout on DMX operation
    };

    static constexpr const char* toString(Status status) {
        switch (status) {
            case SUCCESS: return "success";
            case ERR_UART: return "uart error";
            case ERR_GPIO: return "gpio error";
            case ERR_WRONG_MODE: return "wrong mode";
            case ERR_CHANNEL_OVERFLOW: return "channel overflow";
            case ERR_NULL_INPUT: return "null input";
            case ERR_TASK_FAILED: return "task failed";
            case ERR_NOT_RUNNING: return "not running";
            case ERR_FRAME_ERROR: return "frame error";
            case ERR_TIMEOUT: return "timeout";
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
     */
    EZDMX(Mode mode = Mode::MASTER, Stream* serial = &Serial1, int txPin = 17, int rxPin = 16, int dePin = -1) :
        _mode(mode),
        _uart_num(serialToUartPort(serial)),
        _txPin(txPin),
        _rxPin(rxPin),
        _dePin(dePin),
        _running(false),
        _transmitTaskHandle(nullptr),
        _receiveTaskHandle(nullptr),
        _lastFrameTimestamp(0)
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
     */
    EZDMX(Mode mode = Mode::MASTER, uart_port_t uart_num = UART_NUM_1, int txPin = 17, int rxPin = 16, int dePin = -1) :
        _mode(mode),
        _uart_num(uart_num),
        _txPin(txPin),
        _rxPin(rxPin),
        _dePin(dePin),
        _running(false),
        _transmitTaskHandle(nullptr),
        _receiveTaskHandle(nullptr),
        _lastFrameTimestamp(0)
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
            #if CONFIG_IDF_TARGET_ESP32S3 == 1
                .source_clk = UART_SCLK_APB,       // Source clock (for ESP32S3)
            #else
                .source_clk = UART_SCLK_DEFAULT,       // Source clock (for ESP32C6)
            #endif
        };

        // Install the UART driver
        esp_err_t err = uart_driver_install(_uart_num, DMX_BUFFER_SIZE, DMX_BUFFER_SIZE, 0, NULL, ESP_INTR_FLAG_IRAM);
        if (err != ESP_OK) {
            return Error(ERR_UART);
        }

        err = uart_param_config(_uart_num, &uart_config);
        if (err != ESP_OK) {
            uart_driver_delete(_uart_num);
            return Error(ERR_UART);
        }

        err = uart_set_pin(_uart_num, _txPin, _rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (err != ESP_OK) {
            uart_driver_delete(_uart_num);
            return Error(ERR_GPIO);
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
                    return Error(ERR_GPIO);
                }
                
                err = gpio_set_level((gpio_num_t)_dePin, 1);
                if (err != ESP_OK) {
                    uart_driver_delete(_uart_num);
                    return Error(ERR_GPIO);
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
            BaseType_t ret = xTaskCreate(TransmitTask, 
                        "dmx_tx", 
                        8192, 
                        this, 
                        5, 
                        &_transmitTaskHandle);
            if (ret != pdPASS) {
                _running = false;
                return Error(ERR_TASK_FAILED);
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
                return Error(ERR_TASK_FAILED);
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
            return Error(ERR_NOT_RUNNING);
        }

        _running = false;
        vTaskDelay(pdMS_TO_TICKS(50));
        
        if (_mode == Mode::MASTER) {
            _transmitTaskHandle = nullptr;
        } else {
            _receiveTaskHandle = nullptr;
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
            return Error(ERR_WRONG_MODE);
        }
        if (channel < 1 || channel > DMX_UNIVERSE_SIZE) {
            return Error(ERR_CHANNEL_OVERFLOW);
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
            return Error(ERR_WRONG_MODE);
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
            return Error(ERR_WRONG_MODE);
        }
        if (values == nullptr) {
            return Error(ERR_NULL_INPUT);
        }
        
        // Check & adjust startChannel
        if (startChannel < 1) {
            startChannel = 1;
        } else if (startChannel > DMX_UNIVERSE_SIZE) {
            return Error(ERR_CHANNEL_OVERFLOW);
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
            return Error(ERR_WRONG_MODE);
        }
        
        // Check & adjust startChannel
        if (startChannel < 1) {
            startChannel = 1;
        } else if (startChannel > DMX_UNIVERSE_SIZE) {
            return Error(ERR_CHANNEL_OVERFLOW);
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
            return Error(ERR_CHANNEL_OVERFLOW);
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
            return Error(ERR_NULL_INPUT);
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

    // Storage of DMX payload (0 = start code, 1..512 = channels)
    uint8_t _channels[DMX_UNIVERSE_SIZE + 1] = {0};

    // Storage for SLAVE mode
    uint8_t _slaveValues[DMX_UNIVERSE_SIZE];
    uint64_t _lastFrameTimestamp;

    /**
     * @brief DMX transmission task
     */
    static void TransmitTask(void* pEZDMX) {
        EZDMX* dmx = static_cast<EZDMX*>(pEZDMX);
        
        while (dmx->_running) {

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
            
            // 4. Wait for all data to be sent (non blocking since it's done in a dedicated task)
            uart_wait_tx_done(dmx->_uart_num, pdMS_TO_TICKS(100));
            
            // 5. Wait for the frame interval
            vTaskDelay(pdMS_TO_TICKS(DMX_MTBF_MS));
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
        #if CONFIG_IDF_TARGET_ESP32S3 == 1
            else if (serial == &Serial2) return UART_NUM_2;
        #endif
        return UART_NUM_MAX;
    }
#endif

};

#endif // EZDMX_H