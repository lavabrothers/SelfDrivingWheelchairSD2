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

// --- Pin Definitions (Updated with new understanding) ---
const int A_IN1_PIN = 34; // ValLeftX
const int A_IN2_PIN = 35; // ValRightX
const int A_IN3_PIN = 32; // ValDownY
const int A_IN4_PIN = 33; // ValupY
const int LED_GPIO = 4;
const int I2C_SDA_PIN = 22;
const int I2C_SCL_PIN = 23;
const int BUTTON_PIN = 21; // Emergency stop button

// --- BLE UUIDs ---
#define SERVICE_UUID           "c7a72f3e-3151-4d39-a18e-4a73426b3e2b"
#define CHARACTERISTIC_UUID_RX "f3711319-333e-41a4-b04b-32a7b8e1136c"
#define CHARACTERISTIC_UUID_TX "d1aea128-4f7e-4c4f-a7b5-c603a111a00a"

// --- Global Variables ---
BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
BLECharacteristic* pRxCharacteristic = NULL;
bool deviceConnected = false;
int wheelchairX = -16; 

// --- NEW: State Machine & Menu Variables ---
enum AppState {
    STATE_MENU,
    STATE_CONTROL
};
AppState currentState = STATE_MENU; // Start in the menu

// --- NEW: Sub-state for the menu ---
enum MenuState {
    MENU_STATE_MAIN,
    MENU_STATE_SUBPAGE // e.g., "About" or "Settings"
};
MenuState currentMenuState = MENU_STATE_MAIN;

const char* menuItems[] = {"Start Control", "Settings (N/A)", "About"};
const int numMenuItems = 3;
int selectedMenuItem = 0;
bool joyMoved = false; // Flag to prevent rapid scrolling

// --- NEW: Joystick Activation Threshold ---
// We now have 4 directional inputs, not 2 joysticks.
// We assume a resting value is low (e.g., < 1000) and an activated
// value is high (e.g., > 2300) when a direction is pressed.
#define JOY_ACTIVATION_THRESHOLD 2300 // The value to trigger a "press"

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
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("❌ Client Disconnected");
        pServer->getAdvertising()->start();
        Serial.println("📢 Advertising restarted...");
        // If we disconnect, force back to the menu
        currentState = STATE_MENU;
        currentMenuState = MENU_STATE_MAIN;
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
void emergencyStop() {
    Serial.println("🚨 Emergency STOP Activated!");
    
    // Force state to menu
    currentState = STATE_MENU;
    currentMenuState = MENU_STATE_MAIN;

    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(10, 50);
    display.println("Emergency");
    display.setCursor(40, 70);
    display.println("STOP");
    display.display();

    if (deviceConnected) {
        pTxCharacteristic->setValue("STOP");
        pTxCharacteristic->notify();
        Serial.println("Sent BLE message: STOP");
    }
    delay(2000);
    
    joyMoved = true; // Prevent accidental selection
}

// --- Function to draw the menu ---
void drawMenu() {
    display.clearDisplay();
    display.setTextSize(2); // Title size
    display.setTextColor(SH110X_WHITE);
    display.setCursor(10, 10);
    display.println("MAIN MENU");

    display.setTextSize(1); // Item size
    for (int i = 0; i < numMenuItems; i++) {
        display.setCursor(10, 40 + (i * 15));
        if (i == selectedMenuItem) {
            display.print("> "); // Selector
        } else {
            display.print("  ");
        }
        display.println(menuItems[i]);
    }
    display.display();
}


// --- REPLACED FUNCTION ---
// This function now uses the 4 directional inputs for navigation.
void handleMenu() {
    // Read the 4 directional input pins
    int ValLeftX = analogRead(A_IN1_PIN);
    int ValRightX = analogRead(A_IN2_PIN);
    int ValDownY = analogRead(A_IN3_PIN);
    int ValupY = analogRead(A_IN4_PIN);

    // Debug print statement for the new directional values
    Serial.printf("Menu Mode - L: %-5d | R: %-5d | D: %-5d | U: %d\n", ValLeftX, ValRightX, ValDownY, ValupY);

    // This state machine handles menu navigation
    switch (currentMenuState) {
        
        case MENU_STATE_MAIN: {
            bool stateChanged = false; // <-- CHANGED: Flag to stop redraw

            // --- Check Up/Down Movement ---
            if (ValupY > JOY_ACTIVATION_THRESHOLD) { // Pushed UP
                if (!joyMoved) {
                    selectedMenuItem--;
                    if (selectedMenuItem < 0) selectedMenuItem = numMenuItems - 1;
                    joyMoved = true;
                }
            } else if (ValDownY > JOY_ACTIVATION_THRESHOLD) { // Pushed DOWN
                if (!joyMoved) {
                    selectedMenuItem++;
                    if (selectedMenuItem >= numMenuItems) selectedMenuItem = 0;
                    joyMoved = true;
                }
            } 
            // --- Check Select Movement (RIGHT) ---
            else if (ValRightX > JOY_ACTIVATION_THRESHOLD) { // Pushed RIGHT
                if (!joyMoved) {
                    joyMoved = true;
                    stateChanged = true; // <-- CHANGED: Assume state will change
                    
                    switch (selectedMenuItem) {
                        case 0: // "Start Control"
                            if (deviceConnected) {
                                currentState = STATE_CONTROL;
                            } else {
                                updateDisplay("Error", "BLE Not Connected", "Press Left to exit");
                                currentMenuState = MENU_STATE_SUBPAGE; // Show error as a subpage
                            }
                            break;
                        case 1: // "Settings"
                            updateDisplay("Settings", "Not Implemented", "Press Left to exit");
                            currentMenuState = MENU_STATE_SUBPAGE;
                            break;
                        case 2: // "About"
                            updateDisplay("About", "Wheelchair v1.0", "Press Left to exit");
                            currentMenuState = MENU_STATE_SUBPAGE;
                            break;
                    }
                }
            }
            // --- Check "Back" Movement (LEFT) ---
            else if (ValLeftX > JOY_ACTIVATION_THRESHOLD) { // Pushed LEFT
                if (!joyMoved) {
                    // "Back" in main menu does nothing
                    joyMoved = true; 
                }
            }
            // --- No Movement ---
            else { 
                joyMoved = false; // Reset the flag
            }

            // --- CHANGED: This is the fix ---
            // ONLY draw the menu if we haven't just changed to a sub-page or control state
            if (!stateChanged) {
                drawMenu();
            }
            break;
        } // End MENU_STATE_MAIN

        case MENU_STATE_SUBPAGE: {
            // In a sub-page, we ONLY look for the "Back" button (LEFT)
            if (ValLeftX > JOY_ACTIVATION_THRESHOLD) { // Pushed LEFT
                if (!joyMoved) {
                    currentMenuState = MENU_STATE_MAIN; // Go back to main menu
                    joyMoved = true;
                }
            } else { 
                // Reset flag when "Back" (left) is released
                joyMoved = false;
            }
            // We do NOT call drawMenu() here, so the sub-page message stays on screen
            break;
        } // End MENU_STATE_SUBPAGE
    }
}


// --- REPLACED FUNCTION ---
// This function now uses the 4 directional inputs for control
// and can ONLY be exited by the E-Stop button or BLE disconnect.
void handleControl() {
    
    // --- "Back" gesture (Left press) logic has been REMOVED ---
    // The E-Stop button check in loop() is now the only manual exit.
    
    // Run the control logic
    if (deviceConnected) {
        // Read all 4 directional pins
        int ValLeftX = analogRead(A_IN1_PIN);
        int ValRightX = analogRead(A_IN2_PIN);
        int ValDownY = analogRead(A_IN3_PIN);
        int ValupY = analogRead(A_IN4_PIN);

        char txBuffer[24];
        // Send values in the original order (val1, val2, val3, val4)
        // which corresponds to (Up, Down, Right, Left)
        snprintf(txBuffer, sizeof(txBuffer), "%d,%d,%d,%d", ValupY, ValDownY, ValRightX, ValLeftX);

        pTxCharacteristic->setValue(txBuffer);
        pTxCharacteristic->notify();
        
        // Update the OLED with data and animation
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(SH110X_WHITE);
        display.setCursor(0, 0);
        display.println("Connected");
        
        // Update display with new labels
        display.setTextSize(1);
        display.setCursor(0, 25);
        display.printf("L: %-5d R: %d\n", ValLeftX, ValRightX);
        display.setCursor(0, 40);
        display.printf("U: %-5d D: %d\n", ValupY, ValDownY);

        // Draw and move the wheelchair bitmap
        display.drawBitmap(wheelchairX, 112, wheelchair_bmp, 16, 16, SH110X_WHITE);
        wheelchairX++;
        if (wheelchairX > SCREEN_WIDTH) {
            wheelchairX = -16;
        }

        display.display();
    } else {
        // If we lose connection while in this state, go back to menu
        currentState = STATE_MENU;
        currentMenuState = MENU_STATE_MAIN;
    }
}
// --- END REPLACED FUNCTION ---


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
    pinMode(BUTTON_PIN, INPUT_PULLUP); 

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
    
    currentState = STATE_MENU;
    currentMenuState = MENU_STATE_MAIN;
}

// --- Main Loop (State Machine) ---
void loop() {
    // 1. Check for Emergency Stop FIRST. This is highest priority.
    if (digitalRead(BUTTON_PIN) == LOW) {
        emergencyStop();
    }

    // 2. Run the code for the current state
    switch (currentState) {
        case STATE_MENU:
            handleMenu();
            break;
        case STATE_CONTROL:
            handleControl();
            break;
    }

    // 3. Add a small delay for stability
    delay(100); 
}