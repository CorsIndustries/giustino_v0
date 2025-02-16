
/******************************************************************************
 * Giustino V0
 *
 * Simple PS4 tank controller for ESP32. Based on bluepad32 libary.
 * 
 * Created at: 10.02.2025
 * Author: binaryBard73
 *
 ******************************************************************************/
#include <Bluepad32.h>


#define MOTOR_RIGHT_IN1 19
#define MOTOR_RIGHT_IN2 21
#define MOTOR_LEFT_IN1 26
#define MOTOR_LEFT_IN2 27

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

/***
 * This callback gets called any time a new gamepad is connected.
 */
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", 
                          ctl->getModelName().c_str(), properties.vendor_id,
                          properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

/***
 * Use the arrow for controlling the gamepad
 */
void processGamepad(ControllerPtr ctl) {

  switch(ctl->dpad()) {
    case 0x02:  // up
      analogWrite(MOTOR_RIGHT_IN1, 0);
      analogWrite(MOTOR_LEFT_IN1, 0);
      analogWrite(MOTOR_RIGHT_IN2, 125);
      analogWrite(MOTOR_LEFT_IN2, 125);
      break;

    case 0x01:  // down
      analogWrite(MOTOR_RIGHT_IN2, 0);
      analogWrite(MOTOR_LEFT_IN2, 0);
      analogWrite(MOTOR_RIGHT_IN1, 125);
      analogWrite(MOTOR_LEFT_IN1, 125);
      break;

    case 0x04:  // right
      analogWrite(MOTOR_RIGHT_IN1, 0);
      analogWrite(MOTOR_RIGHT_IN2, 0);
      analogWrite(MOTOR_LEFT_IN1, 125);
      analogWrite(MOTOR_LEFT_IN2, 0);
      break;  

    case 0x08:  // left
      analogWrite(MOTOR_LEFT_IN2, 0);
      analogWrite(MOTOR_LEFT_IN1, 0);
      analogWrite(MOTOR_RIGHT_IN2, 0);
      analogWrite(MOTOR_RIGHT_IN1, 125);
      break;

    default:
      analogWrite(MOTOR_RIGHT_IN1, 0);
      analogWrite(MOTOR_RIGHT_IN2, 0);
      analogWrite(MOTOR_LEFT_IN2, 0);
      analogWrite(MOTOR_LEFT_IN1, 0);
      break;
  }
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else {
                Serial.println("Unsupported controller");
            }
        }
    }
}


void setup() {

  Serial.begin(115200);

  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);

  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);


  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);

  analogWrite(MOTOR_RIGHT_IN1, 0);
  analogWrite(MOTOR_RIGHT_IN2, 0);
  analogWrite(MOTOR_LEFT_IN1, 0);
  analogWrite(MOTOR_LEFT_IN2, 0);
}

void loop() {
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    delay(150);
}
