//----------------------------------------------------------------
//-- Club Electrónica --//
//-- Otto --//
//-- José M. Peña --//
//-----------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include <Otto.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


// --- Pin Definitions ---
#define ECHO_PIN 0
#define TRIGGER_PIN 1
#define LEFT_LEG_PIN 2
#define RIGHT_LEG_PIN 3
#define LEFT_FOOT_PIN 4
#define RIGHT_FOOT_PIN 5
#define BUZZER_PIN 6
#define ACTION_BUTTON_PIN 7
#define BLUETOOTH_LED_PIN 8
#define PLAY_BUTTON_PIN 21

// --- Nordic UART Service (NUS) UUIDs ---
// The Otto app is most likely looking for these standard UUIDs
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

const char* deviceName = "Otto DIY - Jose";  // <<------------ CHANGE TO EACH KID'S NAME

// --- Global Objects ---
Otto Otto;

// --- Otto Movement Parameters ---
const int OTTO_MOVE_SPEED = 1000;
const int OTTO_SMALL_STEP_PERIOD = 1500;
const int OTTO_STANDARD_HEIGHT = 20;
const int OTTO_TURN_STEPS = 2;

// --- Mode Management ---
enum Mode {
  MODE_APP = 0,
  MODE_AVOID,
  MODE_DETECT,
  MODE_DANCE,
  MODE_COUNT
};
Mode currentMode = MODE_APP;
const char* MODE_NAMES[] = { "APP", "AVOID", "DETECT", "DANCE" };
int currentDanceIndex = 0;

// --- Sensor & Button State ---
unsigned long lastUltraSoundTime = 0;
const unsigned long ULTRASOUND_INTERVAL = 100;
long currentDistance = -1;
const int MAX_DISTANCE_CM = 200;
const unsigned long ULTRASOUND_TIMEOUT_US = MAX_DISTANCE_CM * 58 * 2;

bool lastActionButtonReading = HIGH;
bool lastPlayButtonReading = HIGH;
unsigned long lastActionButtonChangeTime = 0;
unsigned long lastPlayButtonChangeTime = 0;
unsigned long lastActionButtonTriggerTime = 0;
unsigned long lastPlayButtonTriggerTime = 0;
const unsigned long DEBOUNCE_DELAY_MS = 50;
const unsigned long BUTTON_COOLDOWN_MS = 500;
unsigned long lastBlinkTime = 0;
bool ledState = false;

// --- Non-Blocking State Management ---
bool modeActionInProgress = false;
enum class AvoidState { AVOID_IDLE,
                        AVOID_BACKWARD,
                        AVOID_TURN,
                        AVOID_FORWARD };
AvoidState currentAvoidState = AvoidState::AVOID_IDLE;
enum class DetectState { REACTING_IDLE,
                         REACTING_SURPRISE,
                         REACTING_GESTURE,
                         REACTING_FLAP,
                         REACTING_SHAKE_LEG,
                         REACTING_COOLDOWN };
DetectState currentDetectState = DetectState::REACTING_IDLE;
unsigned long detectActionCompletionTime = 0;
const unsigned long DETECT_COOLDOWN_MS = 3000;

// --- Forward Declarations ---
void cycleToNextMode();
void readUltrasound();
void appModeLoop();
void avoidModeLoop();
void danceModeLoop();
void detectModeLoop();
void executeCurrentDance();
void playNextDance();
void danceRoutine1(), danceRoutine2(), danceRoutine3(), danceRoutine4(), danceRoutine5();
void calibrationCommand();
void setupBLE();

void handleMove(int conde, int speed);
void handleGesture(int code);
void handleSong(int code);

BLECharacteristic* pTxCharacteristic;
BLECharacteristic* pRxCharacteristic;
bool deviceConnected = false;

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    String rxValue = pCharacteristic->getValue().c_str();
    ;

    if (rxValue.length() > 0) {
      Serial.print("Received: ");
      Serial.println(rxValue);

      // --- MANUAL PARSING LOGIC ---
      char commandType = rxValue.charAt(0);

      int firstSpace = rxValue.indexOf(' ');
      if (firstSpace == -1) {
        Serial.println("ERR: Invalid format. No space found.");
        return;  // Exit if format is wrong
      }

      // Find the second space to see if a second parameter exists
      int secondSpace = rxValue.indexOf(' ', firstSpace + 1);

      int code = 0;
      int param2 = 1000;  // Default speed for moves

      if (secondSpace != -1) {  // Two parameters found, e.g., "M 1 500"
        code = rxValue.substring(firstSpace + 1, secondSpace).toInt();
        param2 = rxValue.substring(secondSpace + 1).toInt();
      } else {  // Only one parameter found, e.g., "H 19" or "M 0"
        code = rxValue.substring(firstSpace + 1).toInt();
      }

      // --- DISPATCH COMMAND ---
      switch (commandType) {
        case 'M':
          handleMove(code, param2);  // param2 is the speed
          break;
        case 'H':
          handleGesture(code);
          break;
        case 'K':
          handleSong(code);
          break;
        default:
          Serial.println("ERR: Unknown command type.");
          break;
      }
    }
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    Serial.println("BLE client connected.");
    deviceConnected = true;
    Otto.sing(S_connection);
  }

  void onDisconnect(BLEServer* pServer) {
    Serial.println("BLE Client Disconnected");
    deviceConnected = false;
    digitalWrite(BLUETOOTH_LED_PIN, HIGH);
    Otto.sing(S_disconnection);
    // Restart advertising
    BLEDevice::startAdvertising();
  }
};

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Booting Otto Robot...");

  Otto.init(LEFT_LEG_PIN, RIGHT_LEG_PIN, LEFT_FOOT_PIN, RIGHT_FOOT_PIN, true, BUZZER_PIN);

  pinMode(ACTION_BUTTON_PIN, INPUT_PULLUP);
  pinMode(PLAY_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(BLUETOOTH_LED_PIN, OUTPUT);
  digitalWrite(BLUETOOTH_LED_PIN, HIGH);

  setupBLE();  // Setup BLE commands and initialize BLE

  Otto.sing(S_connection);
  Otto.home();
  Serial.println("Robot Ready!");
  Serial.print("Current Mode: ");
  Serial.println(MODE_NAMES[currentMode]);
}

void setupBLE() {
  Serial.println("Enter BLE Setup");
  BLEDevice::init(deviceName);

  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pRxCharacteristic->setCallbacks(new CommandCallbacks());

  pService->start();

  /// Start advertising using the exact method from your working sketch
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);  // Setting to true is fine, name will be added automatically
  // The min/max preferred connection interval settings for iOS
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("Advertising started, waiting for connection...");
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastBlinkTime >= 500) {  // 500ms blink interval
    ledState = !ledState;
    digitalWrite(BLUETOOTH_LED_PIN, ledState);
    lastBlinkTime = currentTime;
  } else {
    digitalWrite(BLUETOOTH_LED_PIN, HIGH);
  }

  // --- Button Handling ---
  int currentActionButtonReading = digitalRead(ACTION_BUTTON_PIN);
  if (currentActionButtonReading != lastActionButtonReading) {
    lastActionButtonChangeTime = currentTime;
  }
  if ((currentTime - lastActionButtonChangeTime) > DEBOUNCE_DELAY_MS) {
    if (currentActionButtonReading == LOW && lastActionButtonReading == HIGH) {
      if ((currentTime - lastActionButtonTriggerTime) > BUTTON_COOLDOWN_MS) {
        lastActionButtonTriggerTime = currentTime;
        Otto.sing(S_buttonPushed);
        cycleToNextMode();
      }
    }
  }
  lastActionButtonReading = currentActionButtonReading;

  int currentPlayButtonReading = digitalRead(PLAY_BUTTON_PIN);
  if (currentPlayButtonReading != lastPlayButtonReading) {
    lastPlayButtonChangeTime = currentTime;
  }
  if ((currentTime - lastPlayButtonChangeTime) > DEBOUNCE_DELAY_MS) {
    if (currentPlayButtonReading == LOW && lastPlayButtonReading == HIGH) {
      if ((currentTime - lastPlayButtonTriggerTime) > BUTTON_COOLDOWN_MS) {
        lastPlayButtonTriggerTime = currentTime;
        if (currentMode == MODE_DANCE) {
          playNextDance();
        } else {
          Otto.sing(S_OhOoh);
        }
      }
    }
  }
  lastPlayButtonReading = currentPlayButtonReading;

  // --- Ultrasound Reading ---
  if (currentTime - lastUltraSoundTime > ULTRASOUND_INTERVAL) {
    lastUltraSoundTime = currentTime;
    readUltrasound();
  }

  // --- Mode Logic ---
  switch (currentMode) {
    case MODE_APP: appModeLoop(); break;
    case MODE_AVOID: avoidModeLoop(); break;
    case MODE_DANCE: danceModeLoop(); break;
    case MODE_DETECT: detectModeLoop(); break;
  }
}

void cycleToNextMode() {
  currentMode = (Mode)((currentMode + 1) % MODE_COUNT);
  Otto.home();
  modeActionInProgress = false;
  currentAvoidState = AvoidState::AVOID_IDLE;
  currentDetectState = DetectState::REACTING_IDLE;
  Serial.print("Switched to mode: ");
  Serial.println(MODE_NAMES[currentMode]);
  Otto.sing(S_happy);
  switch (currentMode) {
    case MODE_AVOID: currentAvoidState = AvoidState::AVOID_FORWARD; break;
    case MODE_DANCE:
      currentDanceIndex = 0;
      executeCurrentDance();
      break;
    default: break;
  }
}

void readUltrasound() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, ULTRASOUND_TIMEOUT_US);
  currentDistance = (duration > 0) ? (duration / 58) : -1;
}

void appModeLoop() {
  // Primarily driven by BLE commands. Could have idle animations.
}

void danceModeLoop() {
  // Dances are blocking, so this loop is mostly unused.
}

void detectModeLoop() {
  unsigned long currentTime = millis();
  switch (currentDetectState) {
    case DetectState::REACTING_IDLE:
      if (currentDistance > 0 && currentDistance < 15) {
        currentDetectState = DetectState::REACTING_GESTURE;
        Otto.sing(S_surprise);
      }
      break;
    case DetectState::REACTING_GESTURE:
      Otto.playGesture(OttoConfused);
      currentDetectState = DetectState::REACTING_FLAP;
      break;
    case DetectState::REACTING_FLAP:
      Otto.flapping(1, OTTO_SMALL_STEP_PERIOD, OTTO_STANDARD_HEIGHT, 1);
      currentDetectState = DetectState::REACTING_SHAKE_LEG;
      break;
    case DetectState::REACTING_SHAKE_LEG:
      Otto.shakeLeg(1, OTTO_SMALL_STEP_PERIOD, 1);
      Otto.home();
      detectActionCompletionTime = currentTime;
      currentDetectState = DetectState::REACTING_COOLDOWN;
      break;
    case DetectState::REACTING_COOLDOWN:
      if (currentTime - detectActionCompletionTime > DETECT_COOLDOWN_MS) {
        currentDetectState = DetectState::REACTING_IDLE;
      }
      break;
  }
}

void avoidModeLoop() {
  // NOTE: This mode is very blocking.
  if (currentDistance > 0 && currentDistance < 15) {
    Otto.sing(S_surprise);
    Otto.walk(2, 1000, -1);
    Otto.turn(4, 1000, 1);
  } else {
    Otto.walk(1, 1000, 1);
  }
}

// --- Dance Routines & Execution ---
std::vector<void (*)()> danceRoutines = { danceRoutine1, danceRoutine2, danceRoutine3, danceRoutine4, danceRoutine5 };

void executeCurrentDance() {
  if (currentDanceIndex < danceRoutines.size()) {
    Otto.sing(S_happy);
    modeActionInProgress = true;
    danceRoutines[currentDanceIndex]();
    Otto.home();
    modeActionInProgress = false;
  }
}

void playNextDance() {
  Otto.home();
  currentDanceIndex = (currentDanceIndex + 1) % danceRoutines.size();
  executeCurrentDance();
}

void danceRoutine1() {
  Otto.playGesture(OttoSuperHappy);
  Otto.jitter(4, 500, 20);
  Otto.moonwalker(2, OTTO_MOVE_SPEED, 25, 1);
  Otto.moonwalker(2, OTTO_MOVE_SPEED, 25, -1);
  Otto.home();
}

void danceRoutine2() {
  Otto.playGesture(OttoVictory);
  Otto.ascendingTurn(1, 500, 30);
  Otto.tiptoeSwing(1, OTTO_MOVE_SPEED, 30);
  Otto.home();
}
void danceRoutine3() {
  Otto.playGesture(OttoLove);
  Otto.flapping(1, 500, 30, 1);
  Otto.crusaito(1, 2000, 30, 1);
  Otto.home();
}
void danceRoutine4() {
  Otto.playGesture(OttoWave);
  Otto.jitter(2, 1000, 20);
  Otto.updown(1, OTTO_SMALL_STEP_PERIOD, OTTO_STANDARD_HEIGHT);
  Otto.jump(1, 500);
  Otto.home();
}
void danceRoutine5() {
  Otto.playGesture(OttoMagic);
  Otto.shakeLeg(1, OTTO_MOVE_SPEED, 1);
  Otto.swing(1, OTTO_MOVE_SPEED, 20);
  Otto.sing(S_disconnection);
  Otto.home();
}

// --- Command Callback Implementations ---
void handleMove(int code, int speed) {
  const int steps = 1;
  Serial.printf("Handling Move: Code=%d, Speed=%d\n", code, speed);

  switch (code) {
    case 1: Otto.walk(steps, speed, 1); break;   // walk forward
    case 2: Otto.walk(steps, speed, -1); break;  // walk backwards
    case 3: Otto.turn(steps, speed, 1); break;   // turn left
    case 4: Otto.turn(steps, speed, -1); break;  // turn right
    case 19: Otto.updown(1, 1000, 20); break;
    case 8: Otto.tiptoeSwing(2, 1000, 30); break;
    case 14: Otto.jitter(4, 500, 20); break;
    case 20: Otto.ascendingTurn(2, 1000, 50); break;
    case 5: Otto.jump(1, 500); break;

    case 13: Otto.swing(2, 1000, 20); break;
    case 7: Otto.crusaito(2, 1000, 20, 1); break;
    case 17: Otto.flapping(2, 1000, 20, -1); break;
    case 15: Otto.bend(1, 500, -1); break;
    case 10: Otto.shakeLeg(2, 1000, -1); break;

    case 12:
      Otto.moonwalker(3, 1000, 25, 1);
      Otto.moonwalker(3, 1000, 25, -1);
      break;
    case 6: Otto.crusaito(2, 1000, 20, -1); break;
    case 18: Otto.flapping(2, 1000, 20, 1); break;
    case 16: Otto.bend(1, 500, 1); break;
    case 9: Otto.shakeLeg(2, 1000, 1); break;

    case 0: Otto.home(); break;  // Stop
    default:
      // If the move code is not 1-4 or 0, maybe it's a gesture code sent with 'M'?
      handleGesture(code);
      break;
  }
}

void handleGesture(int code) {
  Serial.printf("Handling Gesture: Code=%d\n", code);

  switch (code) {
    case 16: Otto.sing(S_mode1); break;
    case 17: Otto.sing(S_mode2); break;
    case 18: Otto.sing(S_mode3); break;
    case 4: Otto.sing(S_OhOoh); break;
    case 19: Otto.sing(S_buttonPushed); break;

    case 10: Otto.sing(S_happy_short); break;
    case 8: Otto.sing(S_happy); break;
    case 9: Otto.sing(S_superHappy); break;
    case 11: Otto.sing(S_sad); break;
    case 7: Otto.sing(S_sleeping); break;

    case 3: Otto.sing(S_OhOoh2); break;
    case 2: Otto.sing(S_surprise); break;
    case 12: Otto.sing(S_confused); break;
    case 15: Otto.sing(S_fart1); break;
    case 6: Otto.sing(S_cuddly); break;
    default:
      Serial.println("Unknown gesture code.");
      Otto.home();
      break;
  }
}

void handleSong(int code) {
  Serial.printf("Handling Song: Code=%d\n", code);

  switch (code) {
    case 1: Otto.playGesture(OttoHappy); break;
    case 2: Otto.playGesture(OttoSuperHappy); break;
    case 7: Otto.playGesture(OttoLove); break;
    case 3: Otto.playGesture(OttoSad); break;
    case 13: Otto.playGesture(OttoFail); break;

    case 6: Otto.playGesture(OttoConfused); break;
    case 5: Otto.playGesture(OttoVictory); break;
    case 9: Otto.playGesture(OttoFretful); break;
    case 8: Otto.playGesture(OttoAngry); break;
    case 4: Otto.playGesture(OttoSleeping); break;

    case 10: Otto.playGesture(OttoWave); break;
    case 12: Otto.playGesture(OttoMagic); break;
    case 11: Otto.playGesture(OttoFart); break;
  }
}
