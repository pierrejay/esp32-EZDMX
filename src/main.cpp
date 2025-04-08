/**
 * EZDMX library example
 */
#include <Arduino.h>
#include <EZDMX.h>

#ifndef DMX_MODE
#define DMX_MODE 0  // Default to MASTER mode if not defined
#endif

// Select operation mode via define
#if DMX_MODE
#define IS_SLAVE_MODE 1
#else
#define IS_SLAVE_MODE 0
#endif

// Define pins
static constexpr int DMX_TX = 16;
static constexpr int DMX_RX = 23;
static constexpr int DMX_DE = -1;  // Pin for RS485 transmitter, -1 if not used

#if !IS_SLAVE_MODE
static constexpr int BUTTON_PIN = 1;  // Pin for button only in MASTER mode
#endif

// Create a DMX instance using Serial1
EZDMX dmx(IS_SLAVE_MODE ? EZDMX::Mode::SLAVE : EZDMX::Mode::MASTER, &Serial1, DMX_TX, DMX_RX, DMX_DE);

#if IS_SLAVE_MODE
// Variables for SLAVE mode
static uint64_t lastDisplayTime = 0;
static constexpr uint32_t DISPLAY_INTERVAL_MS = 1000;  // Display interval in SLAVE mode
#endif

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.printf("DMX %s init...\n", IS_SLAVE_MODE ? "Slave" : "Master");

  #if !IS_SLAVE_MODE
  // Configure button with internal pull-up in MASTER mode
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  #endif

  // Initialize DMX library
  EZDMX::Result result = dmx.begin();
  if (result != EZDMX::SUCCESS) {
    Serial.printf("DMX init failed! Error: %s\n", EZDMX::toString(result.status));
    while (1) {
      delay(1000);
    }
  }

  Serial.printf("DMX %s init with success!\n", IS_SLAVE_MODE ? "Slave" : "Master");

  #if !IS_SLAVE_MODE
  // MASTER mode: Initialize channels to 0
  result = dmx.resetAllChannels();
  if (result != EZDMX::SUCCESS) {
    Serial.printf("Reset channels failed! Error: %s\n", EZDMX::toString(result.status));
  }
  #endif

  // Start DMX transmission/reception
  result = dmx.start();
  if (result != EZDMX::SUCCESS) {
    Serial.printf("Start failed! Error: %s\n", EZDMX::toString(result.status));
    while (1) {
      delay(1000);
    }
  }
  Serial.printf("%s DMX started!\n", IS_SLAVE_MODE ? "Reception" : "Transmission");
}

#if IS_SLAVE_MODE
void handleSlaveMode() {
  uint64_t now = millis();
  if (now - lastDisplayTime >= DISPLAY_INTERVAL_MS) {
    lastDisplayTime = now;
    
    uint8_t values[EZDMX::DMX_UNIVERSE_SIZE];
    uint64_t timestamp;
    
    EZDMX::Result result = dmx.getAllChannels(values, &timestamp);
    if (result == EZDMX::SUCCESS) {
      Serial.printf("Last frame received %lld ms ago\n", (millis() - timestamp));
      Serial.printf("Channels read: %d\n", result.nbChannelsRW);
      
      // Display first 8 channels
      for (int i = 0; i < 8; i++) {
        Serial.printf("Channel %d: %d\n", i + 1, values[i]);
      }
      Serial.println("---");
    } else {
      Serial.printf("Failed to get channels! Error: %s\n", EZDMX::toString(result.status));
    }
  }
}
#else
void handleMasterMode() {
  static bool lastButtonState = HIGH;
  static uint8_t currentChannel = 1;  // Current channel (1-8)
  static uint8_t channelValues[8] = {0};  // Values of first 8 channels
  
  bool currentButtonState = digitalRead(BUTTON_PIN);
  
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    // Store current value before modification
    uint16_t nextValue = channelValues[currentChannel - 1] + 50;  // uint16_t to avoid overflow !
    EZDMX::Result result;
    
    if (nextValue >= 250) {
      // Set channel to 250
      channelValues[currentChannel - 1] = 250;
      result = dmx.set(currentChannel, 250);
      if (result != EZDMX::SUCCESS) {
        Serial.printf("Failed to set channel %d! Error: %s\n", currentChannel, EZDMX::toString(result.status));
      }
      
      // Pass to next channel
      currentChannel++;
      
      // If all channels are done, reset
      if (currentChannel > 8) {
        currentChannel = 1;
        for (int i = 0; i < 8; i++) {
          channelValues[i] = 0;
          result = dmx.set(i + 1, 0);
          if (result != EZDMX::SUCCESS) {
            Serial.printf("Failed to reset channel %d! Error: %s\n", i + 1, EZDMX::toString(result.status));
          }
        }
        Serial.println("All channels reset to 0");
      }
    } else {
      // Otherwise just increment current channel by 50
      channelValues[currentChannel - 1] = nextValue;
      result = dmx.set(currentChannel, nextValue);
      if (result != EZDMX::SUCCESS) {
        Serial.printf("Failed to set channel %d! Error: %s\n", currentChannel, EZDMX::toString(result.status));
      }
    }
    
    // Display all channels state
    Serial.println("Current channels state:");
    for (int i = 0; i < 8; i++) {
      Serial.printf("Channel %d: %d\n", i + 1, channelValues[i]);
    }
    Serial.println("---");
    
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  
  lastButtonState = currentButtonState;
}
#endif

void loop() {
  #if IS_SLAVE_MODE
  handleSlaveMode();
  #else
  handleMasterMode();
  #endif
}