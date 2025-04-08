/**
 * Exemple d'utilisation de la bibliothèque DMXMaster
 */
#include <Arduino.h>
#include <EZDMX.h>

#ifndef DMX_MODE
#define DMX_MODE 0  // Par défaut en mode MASTER si non défini
#endif

// Sélection du mode de fonctionnement via define
#if DMX_MODE
#define IS_SLAVE_MODE 1
#else
#define IS_SLAVE_MODE 0
#endif

// Définition des pins
static constexpr int DMX_TX = 16;
static constexpr int DMX_RX = 23;
static constexpr int DMX_DE = -1;  // Pin pour activer l'émetteur RS485, -1 si non utilisé

#if !IS_SLAVE_MODE
static constexpr int BUTTON_PIN = 1;  // Pin pour le bouton uniquement en mode MASTER
#endif

// Création d'une instance DMX en utilisant Serial1
EZDMX dmx(IS_SLAVE_MODE ? EZDMX::Mode::SLAVE : EZDMX::Mode::MASTER, &Serial1, DMX_TX, DMX_RX, DMX_DE);

#if IS_SLAVE_MODE
// Variables pour le mode SLAVE
static uint64_t lastDisplayTime = 0;
static constexpr uint32_t DISPLAY_INTERVAL_MS = 1000;  // Intervalle d'affichage des valeurs en mode SLAVE
#endif

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.printf("DMX %s init...\n", IS_SLAVE_MODE ? "Slave" : "Master");

  #if !IS_SLAVE_MODE
  // Configuration du bouton avec pull-up interne en mode MASTER
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  #endif

  // Initialiser la bibliothèque DMX
  EZDMX::Result result = dmx.begin();
  if (result != EZDMX::SUCCESS) {
    Serial.printf("DMX init failed! Error: %s\n", EZDMX::toString(result.status));
    while (1) {
      delay(1000);
    }
  }

  Serial.printf("DMX %s init with success!\n", IS_SLAVE_MODE ? "Slave" : "Master");

  #if !IS_SLAVE_MODE
  // Mode MASTER : Initialisation des canaux à 0
  result = dmx.resetAllChannels();
  if (result != EZDMX::SUCCESS) {
    Serial.printf("Reset channels failed! Error: %s\n", EZDMX::toString(result.status));
  }
  #endif

  // Démarrer la transmission/réception DMX
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
      
      // Afficher les 8 premiers canaux
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
  static uint8_t currentChannel = 1;  // Canal en cours d'incrémentation (1-8)
  static uint8_t channelValues[8] = {0};  // Valeurs des 8 premiers canaux
  
  bool currentButtonState = digitalRead(BUTTON_PIN);
  
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    // Stocker la valeur actuelle avant modification
    uint16_t nextValue = channelValues[currentChannel - 1] + 50;  // uint16_t pour éviter l'overflow !
    EZDMX::Result result;
    
    if (nextValue >= 250) {
      // Mettre le canal à 250
      channelValues[currentChannel - 1] = 250;
      result = dmx.set(currentChannel, 250);
      if (result != EZDMX::SUCCESS) {
        Serial.printf("Failed to set channel %d! Error: %s\n", currentChannel, EZDMX::toString(result.status));
      }
      
      // Passer au canal suivant
      currentChannel++;
      
      // Si on a fini tous les canaux, reset
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
      // Sinon juste incrémenter de 50 le canal courant
      channelValues[currentChannel - 1] = nextValue;
      result = dmx.set(currentChannel, nextValue);
      if (result != EZDMX::SUCCESS) {
        Serial.printf("Failed to set channel %d! Error: %s\n", currentChannel, EZDMX::toString(result.status));
      }
    }
    
    // Afficher l'état de tous les canaux
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