#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// 16x16 pixel wheelchair bitmap, stored in Flash memory
const unsigned char wheelchair_bmp[] PROGMEM = {
    0x00, 0x00, 0x01, 0x80, 0x03, 0xc0, 0x03, 0xc0, 0x07, 0xe0, 0x07, 0xe0, 0x1f, 0xf8, 0x3f, 0xfc, 
    0x3f, 0xfc, 0x1f, 0xf8, 0x0c, 0x30, 0x0c, 0x30, 0x1f, 0xf8, 0x38, 0x1c, 0x20, 0x04, 0x00, 0x00
};

// --- OLED Screen Configuration ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 128
#define OLED_RESET -1
Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 400000, 0x3C);

// --- Pin Definitions ---
const int A_IN1_PIN = 34;
const int A_IN2_PIN = 35;
const int A_IN3_PIN = 32;
const int A_IN4_PIN = 33;
const int LED_GPIO = 4;
const int I2C_SDA_PIN = 22;
const int I2C_SCL_PIN = 23;
const int BUTTON_PIN = 21; // <-- NEW: Emergency stop button on GPIO 21

// --- BLE UUIDs ---
#define SERVICE_UUID           "c7a72f3e-3151-4d39-a18e-4a73426b3e2b"
#define CHARACTERISTIC_UUID_RX "f3711319-333e-41a4-b04b-32a7b8e1136c"
#define CHARACTERISTIC_UUID_TX "d1aea128-4f7e-4c4f-a7b5-c603a111a00a"

// --- Global Variables ---
BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
BLECharacteristic* pRxCharacteristic = NULL;
bool deviceConnected = false;
int wheelchairX = -16; // Tracks the wheelchair animation's X position

// --- Helper Function to Update OLED Display ---
void updateDisplay(const String& line1, const String& line2 = "", const String& line3 = "") {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println(line1);
    if (line2.length() > 0) {
        display.setCursor(0, 10);
        display.println(line2);
    }
    if (line3.length() > 0) {
        display.setCursor(0, 20);
        display.println(line3);
    }
    display.display();
}

// --- BLE Server Callbacks ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("✅ Client Connected");
        updateDisplay("Status: Connected!", "✅");
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("❌ Client Disconnected");
        pServer->getAdvertising()->start();
        Serial.println("📢 Advertising restarted...");
        updateDisplay("Status: Disconnected", "Scanning...", "📢");
    }
};

// --- BLE Characteristic Callbacks ---
class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            Serial.print("Received Value: ");
            Serial.println(rxValue);
            if (rxValue[0] == '1') {
                digitalWrite(LED_GPIO, HIGH);
            } else if (rxValue[0] == '0') {
                digitalWrite(LED_GPIO, LOW);
            }
        }
    }
};

// --- Emergency Stop Function ---
// <-- NEW: This entire function is new
void emergencyStop() {
    Serial.println("🚨 Emergency STOP Activated!");

    // Display emergency message on OLED
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(10, 50); // A bit of centering
    display.println("Emergency");
    display.setCursor(40, 70);
    display.println("STOP");
    display.display();

    // Transmit "STOP" message over BLE if connected
    if (deviceConnected) {
        pTxCharacteristic->setValue("STOP");
        pTxCharacteristic->notify();
        Serial.println("Sent BLE message: STOP");
    }

    // Hold the message on screen for 2 seconds
    delay(2000);
}


// --- Main Setup ---
void setup() {
    Serial.begin(115200);
    Serial.println("🚀 Starting ESP32 BLE Joystick Server...");
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    if(!display.begin()) {
        Serial.println(F("SH110X allocation failed"));
        for(;;);
    }
    display.setRotation(1);
    updateDisplay("Initializing...", "Please wait...");
    delay(1000);

    pinMode(LED_GPIO, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP); // <-- NEW: Setup button pin

    BLEDevice::init("ESP32_BLE_Server");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);

    pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
    pTxCharacteristic->addDescriptor(new BLE2902());

    pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
    pRxCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

    pService->start();
    BLEDevice::getAdvertising()->addServiceUUID(SERVICE_UUID);
    BLEDevice::startAdvertising();
    
    Serial.println("✅ BLE Server setup complete.");
    updateDisplay("Status: Ready", "Scanning...", "📢");
}

// --- Main Loop ---
void loop() {
    // <-- MODIFIED: Check for the emergency stop button press first
    if (digitalRead(BUTTON_PIN) == LOW) {
        emergencyStop();
    }

    if (deviceConnected) {
        int val1 = analogRead(A_IN4_PIN);   //Y2
        int val2 = analogRead(A_IN3_PIN);   //X2
        int val3 = analogRead(A_IN2_PIN);   //Y1
        int val4 = analogRead(A_IN1_PIN);   //X1

        char txBuffer[24];
        snprintf(txBuffer, sizeof(txBuffer), "%d,%d,%d,%d", val1, val2, val3, val4);

        pTxCharacteristic->setValue(txBuffer);
        pTxCharacteristic->notify();

        Serial.print("Sent data: ");
        Serial.println(txBuffer);
        
        // Update the OLED with data and animation
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(SH110X_WHITE);
        display.setCursor(0, 0);
        display.println("Connected");
        
        display.setTextSize(1);
        display.setCursor(0, 25);
        //changing this originallt val1,val2
        display.printf("X1: %-5d Y1: %d\n", val4, val3);
        display.setCursor(0, 40);
        //orignally val3,val4
        display.printf("X2: %-5d Y2: %d\n", val2, val1);

        // Draw and move the wheelchair bitmap
        display.drawBitmap(wheelchairX, 112, wheelchair_bmp, 16, 16, SH110X_WHITE);
        wheelchairX++;
        if (wheelchairX > SCREEN_WIDTH) {
            wheelchairX = -16;
        }

        display.display();
    }
    delay(100); 
}