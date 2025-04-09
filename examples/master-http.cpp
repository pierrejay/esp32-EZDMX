/**
 * EZDMX Master HTTP API example
 */
#include <Arduino.h>
#include <EthernetESP32.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <EZDMX.h>

// DMX UART pins
static constexpr int DMX_TX = 43;
static constexpr int DMX_RX = 44;
static constexpr int DMX_DE = -1;

// Ethernet pins
static constexpr int ETH_MOSI = 11;
static constexpr int ETH_MISO = 12;
static constexpr int ETH_CLK = 13;
static constexpr int ETH_CS = 14;
static constexpr int ETH_INT = 10;
static constexpr int ETH_RST = 9;
W5500Driver driver(ETH_CS, ETH_INT, ETH_RST);

AsyncWebServer server(80);

// Create a DMX instance using Serial1
EZDMX dmx(EZDMX::Mode::MASTER, &Serial1, DMX_TX, DMX_RX, DMX_DE);

void setupNetwork() {
  Serial.println("[setupNetwork] Configuring Ethernet...");
  SPI.begin(ETH_CLK, ETH_MISO, ETH_MOSI);
  driver.setSPI(SPI);
  Ethernet.init(driver);

  // Generate a MAC address
  uint8_t mac[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
  
  // Start Ethernet
  if (Ethernet.begin(mac)) {
    Serial.println("[setupNetwork] Ethernet started via DHCP");
    Serial.printf("[setupNetwork] IP: %s\n", Ethernet.localIP().toString().c_str());
    Serial.printf("[setupNetwork] Gateway: %s\n", Ethernet.gatewayIP().toString().c_str());
    Serial.printf("[setupNetwork] Subnet: %s\n", Ethernet.subnetMask().toString().c_str());
  } else {
    Serial.println("[setupNetwork] Ethernet initialization failed");
  }
}


// HTTP API :
//
// - GET /dmx?channel=1: display a single channel
// - GET /dmx: display all channels
//
// - POST /dmx/set {channel: 1, value: 255}: set a single channel
// - POST /dmx/set [{channel: 1, value: 255}, {channel: 2, value: 128}, ...]: set multiple channels
// - POST /dmx/set {channels: [1, 2, ...], values: [255, 128, ...]}: set multiple channels
// - POST /dmx/set [255, 128, ...]: set multiple channels starting from 1
//
// - POST /dmx/set {run: true}: start DMX transmission
// - POST /dmx/set {run: false}: stop DMX transmission
// - POST /dmx/set {clear: true}: clear all channels

void setupWebServer() {
    server.on("/dmx", HTTP_GET, [](AsyncWebServerRequest *request){
      Serial.printf("[HTTP] GET /dmx from %s\n", request->client()->remoteIP().toString().c_str());
      
      // GET /dmx?channel=1: display a single channel
      if (request->hasParam("channel")) {
        int channel = request->getParam("channel")->value().toInt();
        Serial.printf("[HTTP] Reading channel %d\n", channel);
        
        AsyncJsonResponse* response = new AsyncJsonResponse();
        JsonObject root = response->getRoot();
        root["channel"] = channel;
        
        uint8_t value;
        EZDMX::Result result = dmx.get(channel, value);
        if (result == EZDMX::SUCCESS) {
            root["value"] = value;
        } else {
            root["error"] = EZDMX::toString(result.status);
        }
        
        response->setLength();
        request->send(response);
        
        if (result == EZDMX::SUCCESS) {
            Serial.printf("[HTTP] Sent value %d for channel %d\n", value, channel);
        }
      } 
      
      // Default: display all channels
      else {
        Serial.println("[HTTP] Reading all channels");
        AsyncJsonResponse* response = new AsyncJsonResponse(true); // true = array
        JsonArray root = response->getRoot();
        
        uint8_t values[EZDMX::DMX_UNIVERSE_SIZE];
        uint64_t timestamp;
        EZDMX::Result result = dmx.getAllChannels(values, &timestamp);
        
        if (result == EZDMX::SUCCESS) {
          for (int i = 0; i < EZDMX::DMX_UNIVERSE_SIZE; i++) {
            root.add(values[i]);
          }
        } else {
          for (int i = 0; i < EZDMX::DMX_UNIVERSE_SIZE; i++) {
            root.add(0);
          }
        }
        
        response->setLength();
        request->send(response);
      }
    });

    server.on("/dmx/set", HTTP_POST, [](AsyncWebServerRequest *request){}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      Serial.printf("[HTTP] POST /dmx/set (with body) from %s\n", request->client()->remoteIP().toString().c_str());
      Serial.printf("[HTTP] Received payload (%d bytes): %.*s\n", len, len, (char*)data);

      AsyncJsonResponse* response = new AsyncJsonResponse();
      JsonObject root = response->getRoot();

      if (!data || len == 0) {
        Serial.println("[HTTP] Error: Empty request body");
        root["error"] = "request body is required";
        response->setCode(400);
        response->setLength();
        request->send(response);
        return;
      }

      StaticJsonDocument<8192> doc;
      DeserializationError error = deserializeJson(doc, (const char*)data, len);

      // INVALID JSON ERROR
      if (error) {
        Serial.printf("[HTTP] Error parsing JSON: %s\n", error.c_str());
        root["error"] = "invalid JSON";
        response->setCode(400);
        response->setLength();
        request->send(response);
        return;
      }

      // START/STOP DMX TRANSMISSION
      if (doc.containsKey("run")) {
        bool shouldRun = doc["run"].as<bool>();
        Serial.printf("[HTTP] DMX %s command received\n", shouldRun ? "START" : "STOP");
        auto result = shouldRun ? dmx.start() : dmx.stop();
        if (result != EZDMX::SUCCESS) {
          Serial.printf("[HTTP] Error setting DMX %s: %s\n", shouldRun ? "START" : "STOP", EZDMX::toString(result.status));
          root["error"] = EZDMX::toString(result.status);
          response->setCode(500);
          response->setLength();
          request->send(response);
          return;
        }
        root["status"] = "success";
        root["running"] = shouldRun;
        response->setLength();
        request->send(response);
        return;
      }

      // CLEAR ALL CHANNELS
      if (doc.containsKey("clear")) {
        auto result = dmx.resetAllChannels();
        if (result != EZDMX::SUCCESS) {
          Serial.printf("[HTTP] Error clearing channels: %s\n", EZDMX::toString(result.status));
          root["error"] = EZDMX::toString(result.status);
          response->setCode(500);
          response->setLength();
          request->send(response);
          return;
        }
        root["status"] = "success";
        response->setLength();
        request->send(response);
        return;
      }

      // ARRAY FORMAT: SET MULTIPLE CHANNELS
      if (doc.is<JsonArray>()) {
        Serial.println("[HTTP] Processing array format data");
        JsonArray array = doc.as<JsonArray>();

        // Format: [50, 100, 150, 200, ...]
        if (array[0].is<int>()) {
          int channel = 1;
          for (JsonVariant value : array) {
            EZDMX::Result result = dmx.set(channel++, value.as<uint8_t>());
            if (result != EZDMX::SUCCESS) {
              Serial.printf("[HTTP] Error setting channel %d: %s\n", channel, EZDMX::toString(result.status));
            }
          }
        } 

        // Format: [{"channel": 1, "value": 255}, ...]
        else {
          for (JsonObject item : array) {
            EZDMX::Result result = dmx.set(item["channel"].as<uint16_t>(), item["value"].as<uint8_t>());
            if (result != EZDMX::SUCCESS) {
              Serial.printf("[HTTP] Error setting channel %d: %s\n", item["channel"].as<uint16_t>(), EZDMX::toString(result.status));
            }
          }
        }
      } 
      
      // OBJECT FORMAT: SET SINGLE OR MULTIPLE CHANNELS
      else if (doc.is<JsonObject>()) {
        Serial.println("[HTTP] Processing object format data");

        // Format: {"channels":[1, 8, 3, 5], "values":[50, 100, 150, 200]}
        if (doc.containsKey("channels") && doc.containsKey("values")) {
          JsonArray channels = doc["channels"].as<JsonArray>();
          JsonArray values = doc["values"].as<JsonArray>();
          
          size_t minSize = min(channels.size(), values.size());
          for (size_t i = 0; i < minSize; i++) {
            EZDMX::Result result = dmx.set(channels[i].as<uint16_t>(), values[i].as<uint8_t>());
            if (result != EZDMX::SUCCESS) {
              Serial.printf("[HTTP] Error setting channel %d: %s\n", channels[i].as<uint16_t>(), EZDMX::toString(result.status));
            }
          }
        } 

        // Format: {"channel": 1, "value": 255}
        else if (doc.containsKey("channel") && doc.containsKey("value")) {
          EZDMX::Result result = dmx.set(doc["channel"].as<uint16_t>(), doc["value"].as<uint8_t>());
          if (result != EZDMX::SUCCESS) {
            Serial.printf("[HTTP] Error setting channel %d: %s\n", doc["channel"].as<uint16_t>(), EZDMX::toString(result.status));
            root["error"] = EZDMX::toString(result.status);
            response->setCode(400);
            response->setLength();
            request->send(response);
            return;
          }
        }
        // Empty object or missing required fields
        else {
          Serial.println("[HTTP] Error: Missing required fields");
          root["error"] = "missing required fields (channel and value)";
          response->setCode(400);
          response->setLength();
          request->send(response);
          return;
        }
      }

      // FALLBACK: RETURN ERROR
      else {
        Serial.println("[HTTP] Error: Invalid request body");
        root["error"] = "invalid request body";
        response->setCode(400);
        response->setLength();
        request->send(response);
        return;
      }

      root["status"] = "success";
      response->setLength();
      request->send(response);
    });

  server.begin();
  Serial.println("[setupWebServer] Web server started");
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("[setup] Starting DMX Master HTTP example");

  setupNetwork();

  // Initialize EZDMX library
  auto result = dmx.begin();
  if (result != EZDMX::SUCCESS) {
    Serial.printf("[setup] DMX initialization error: %s\n", EZDMX::toString(result.status));
    while (1) {
      delay(1000);
    }
  }
  Serial.printf("[setup] DMX Master initialized successfully\n");

  setupWebServer();

  // Start DMX transmission
  result = dmx.start();
  if (result != EZDMX::SUCCESS) {
    Serial.printf("[setup] DMX start error: %s\n", EZDMX::toString(result.status));
    while (1) {
      delay(1000);
    }
  }
  Serial.printf("[setup] DMX transmission started\n");
}

void loop() {
  vTaskDelete(NULL); // EZDMX handles operations in the background
}