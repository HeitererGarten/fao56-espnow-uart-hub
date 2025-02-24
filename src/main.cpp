#include <Arduino.h>
#include <HardwareSerial.h>
#include <string.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

#define BAUD_RATE 115200
#define RX_BUFFER_SIZE 2048

// Define two possible message types 
enum MessageType {PAIRING, DATA};

// Message to receive sensor readings
typedef struct sensorData {
    uint8_t msgType;
    char nodeID[8];
    float temp;
    float humidity;
    long moisture;
} sensorData;

// Message to send sensor readings
typedef struct sensorDataSerial {
    char nodeID[8];
    float temp;
    float humidity;
    long moisture;
} sensorDataSerial;

// New structure for pairing
typedef struct structPairing {       
    uint8_t msgType;
    char nodeID[8];
    uint8_t macAddr[6];
    uint8_t channel;
} structPairing;

// Struct to hold data from Serial_MQTT_Hub
// Will not handle absurd wifi credentials
typedef struct wifiData {
    // SSID max len is 63
    char wifiSSID[32];
    char wifiPass[64];
} wifiData;

// Instance to hold data from Serial_MQTT_Hub
wifiData wifiProfile;

// Temp array to hold MAC address to be paired with 
uint8_t clientMacAddr[6];
uint8_t channel;
esp_now_peer_info_t slave;

// Hold sensor readings from nodes
sensorData dataInstance; 
// Send out sensor readings
sensorDataSerial dataSerialInstance;
// Send out MAC address to nodes
structPairing pairingInstance;


// Number of bytes Read from Serial each time, must match amount sent
static uint16_t bytesRead = 0;

// Queue settings 
const uint16_t queueLen = 32;

// Handles
static QueueHandle_t msgQueue = NULL; 
static TaskHandle_t serialHandle = NULL;
static TaskHandle_t wifiHandle = NULL;

// Create a dedicated serial connection to the other Hub section
HardwareSerial interSerial(2);

// Get Hub's own MAC address
void printMAC(const uint8_t * mac_addr) {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial.print(macStr);
}

// Add pairing
bool addPeer(const uint8_t *peer_addr) {      
    memset(&slave, 0, sizeof(slave));
    const esp_now_peer_info_t *peer = &slave;
    memcpy(slave.peer_addr, peer_addr, 6);
    
    slave.channel = channel; 
    slave.encrypt = 0; 
    // Check if the peer exists
    bool exists = esp_now_is_peer_exist(slave.peer_addr);
    if (exists) {
        // Slave already paired.
        Serial.println("Already Paired");
        return true;
    }
    else {
        esp_err_t addStatus = esp_now_add_peer(peer);
        if (addStatus == ESP_OK) {
            // Pair success
            Serial.println("Pair success");
            return true;
        }
        else {
            Serial.println("Pair failed");
            return false;
        }
    }
} 

// Callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t type = incomingData[0];
    // Handle normal sensor readings
    if (type==DATA) {
        // Store in a buffer
        memcpy(&dataInstance, incomingData, sizeof(dataInstance));
        // Send buffer to queue
        if (xQueueSendFromISR(msgQueue, (void *) &dataInstance, &xHigherPriorityTaskWoken) != pdTRUE) {
            Serial.println("Queue full");
        }
    } 
    // Handle request to pair from sensor nodes
    else if (type==PAIRING) {
        memcpy(&pairingInstance, incomingData, sizeof(pairingInstance));
        Serial.println(pairingInstance.msgType);
        Serial.println(pairingInstance.nodeID);
        Serial.print("Pairing request from MAC Address: ");
        printMAC(pairingInstance.macAddr);
        Serial.print(" on channel ");
        Serial.println(pairingInstance.channel);

        for (int i = 0; i < 6; i++) {
            clientMacAddr[i] = pairingInstance.macAddr[i];
        }

        // Prevent Hub from responding to itself
        if (strcmp(pairingInstance.nodeID, HUB_ID) != 0) {
            if (pairingInstance.msgType == PAIRING) { 
                memcpy(&pairingInstance.nodeID, HUB_ID, sizeof(HUB_ID));
                // Server is in AP_STA mode: peers need to send data to server soft AP MAC address 
                WiFi.softAPmacAddress(pairingInstance.macAddr);
                Serial.print("Pairing MAC Address: ");
                printMAC(clientMacAddr);
                pairingInstance.channel = channel;
                Serial.println(". Sending response...");
                esp_err_t result = esp_now_send(clientMacAddr, (uint8_t *) &pairingInstance, sizeof(pairingInstance));
                addPeer(clientMacAddr);
            }
        }
    }
}

// FreeRTOS task to retrieve from queue and send to Serial-MQTT Hub
// TODO: Implement start and end markers
void sendSerial(void *parameter) {
    sensorDataSerial serialPackage;
    while (true) {
        if(xQueueReceive(msgQueue, (void *)&dataInstance, 0) == pdTRUE) {
            strcpy(serialPackage.nodeID, dataInstance.nodeID);
            serialPackage.temp = dataInstance.temp;
            serialPackage.humidity = dataInstance.humidity;
            serialPackage.moisture = dataInstance.moisture;
            interSerial.write((uint8_t*)&serialPackage, sizeof(serialPackage));
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void initESP_NOW() {
    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
    Serial.println("ESP-NOW online");
} 

// Establish WiFi connection to pair with sensor nodes 
void startConnection(void *parameter) {
    byte rb;
    while (true) {
        if (interSerial.available()) {
            rb = interSerial.read();
            if (rb == '`') {
                bytesRead = interSerial.readBytes(reinterpret_cast<char*>(&wifiProfile), sizeof(wifiProfile));
                WiFi.begin(wifiProfile.wifiSSID, wifiProfile.wifiPass);
                
                uint8_t status = WiFi.waitForConnectResult();

                if(status == WL_CONNECTED) {
                    Serial.println("\nWiFi connected");
                    break;
                }
            }
        }
        vTaskDelay(0 / portTICK_PERIOD_MS);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // Once connected, report connection details
    if (interSerial.availableForWrite()) {
        byte marker = 'W';
        interSerial.write(marker);
    }

    Serial.print("Server SOFT AP MAC Address:  ");
    Serial.println(WiFi.softAPmacAddress());

    channel = WiFi.channel();
    Serial.print("Station IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Wi-Fi Channel: ");
    Serial.println(channel);

    // Start ESP-NOW communications 
    initESP_NOW();

    vTaskSuspend(NULL);
}

void setup() {
    // Initialize Serial Monitor
    Serial.setRxBufferSize(RX_BUFFER_SIZE);
    Serial.begin(BAUD_RATE);
    interSerial.begin(BAUD_RATE, SERIAL_8N1, RX_HUB, TX_HUB);

    // Initialize queue
    msgQueue = xQueueCreate(queueLen, sizeof(sensorData));

    // Set WiFi mode 
    WiFi.mode(WIFI_AP_STA);

    xTaskCreatePinnedToCore(
        startConnection,
        "startConnection",
        2048,
        NULL,
        2,
        &wifiHandle,
        1
    );

    // Initialize sendSerial task
    xTaskCreatePinnedToCore(
        sendSerial,     // Task
        "sendSerial",   // Task name
        1024,           // Stack size (At least 800)
        NULL,           // Params
        1,              // Priority (At least 1)
        &serialHandle,  // Handle
        0               // Core ID (0 or 1)
    );
}
 
void loop() {
}

