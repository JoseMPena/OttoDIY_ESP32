//----------------------------------------------------------------
//-- Club Electrónica --//
//-- Otto --//
//-- José M. Peña --//
//-----------------------------------------------------------------

#define BLESERIAL_USE_NIMBLE true
#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include <Otto.h>
#include <anyrtttl.h>
// #include <BLESerial.h>
#include <SerialCommand.h>


#define ECHO 0          // ultrasonic sensor echo pin
#define TRIGGER 1       // ultrasonic sensor trigger pin
#define LeftLeg 2       // left leg pin
#define RightLeg 3      // right leg pin
#define LeftFoot 4      // left foot pin
#define RightFoot 5     // right foot pin
#define Buzzer 6        // buzzer pin
#define ActionButton 7  // *rolls eyes*
#define PlayButton 8    // hmm idk... play button pin??

Otto Otto;

int T = 1000;

enum Mode {
  MODE_APP = 0,
  MODE_AVOID,
  MODE_FORCE,
  MODE_DANCE,
  MODE_COUNT
};

Mode currentMode = MODE_APP;

int currentSongIndex = 0;
int currentDanceIndex = 0;

volatile int toneFrequency = 0;
hw_timer_t* toneTimer = NULL;

// --- Forward declarations for new/modified functions ---
void cycleToNextMode();
long ultrasound();
void appMode();
void detectModeLoop();
void startDetectMode();
void stopDetectMode();
void avoidModeLoop();
void forceModeLoop();
void danceModeLoop();

// --- individual dances and song/dance execution ---
void executeCurrentDance();
void playNextDance();
void danceRoutine1();
void danceRoutine2();
void danceRoutine3();
void danceRoutine4();
void danceRoutine5();

void executeCurrentSong();
void playNextSong();
void stopCurrentSong();

void setup() {
  delay(1000);
  Serial.begin(115200);
  delay(500);

  Serial.println("Boot OK");
  Otto.init(LeftLeg, RightLeg, LeftFoot, RightFoot, true, Buzzer);

  pinMode(ActionButton, INPUT_PULLUP);
  pinMode(PlayButton, INPUT_PULLUP);
  pinMode(ECHO, INPUT);
  pinMode(TRIGGER, OUTPUT);
  pinMode(Buzzer, OUTPUT);

  Otto.sing(S_cuddly);
  Serial.println("Robot Ready!");
  Serial.print("Current Mode: ");
  Serial.println(currentMode);
}

// --- DEBOUNCING VARIABLES ---
// Variables to track the last actual reading of the button to detect changes
bool lastActionButtonReading = HIGH;  // Assume pullup, so HIGH initially
bool lastPlayButtonReading = HIGH;

// Variables to store the time when the button input last changed state
long lastActionButtonChangeTime = 0;
long lastPlayButtonChangeTime = 0;

// Variables to store the time when an action was last triggered (for cooldown)
long last_action_button_trigger_time = 0;
long last_play_button_trigger_time = 0;

const long DEBOUNCE_DELAY_MS = 50;     // Time to wait for button input to stabilize
const long ACTION_COOLDOWN_MS = 1000;  // Time to wait between successive actions (e.g., mode changes)

bool appModeEnabled = false;

void loop() {
  // --- Action Button Debounce and Cooldown Logic ---
  int currentActionButtonReading = digitalRead(ActionButton);

  // Detect a change in the button's physical reading
  if (currentActionButtonReading != lastActionButtonReading) {
    lastActionButtonChangeTime = millis();  // Reset the debounce timer
  }

  // If the reading has been stable for DEBOUNCE_DELAY_MS
  if ((millis() - lastActionButtonChangeTime) > DEBOUNCE_DELAY_MS) {
    // If the button is currently pressed (LOW after pullup) and has been stable
    if (currentActionButtonReading == LOW) {
      // And enough time has passed since the last action trigger (cooldown)
      if ((millis() - last_action_button_trigger_time) > ACTION_COOLDOWN_MS) {
        last_action_button_trigger_time = millis();  // Reset cooldown timer
        Otto.sing(S_buttonPushed);
        cycleToNextMode();
      }
    }
  }

  lastActionButtonReading = currentActionButtonReading;  // Save current reading for next iteration

  // --- Play Button Debounce and Cooldown Logic ---
  int currentPlayButtonReading = digitalRead(PlayButton);

  // Detect a change in the button's physical reading
  if (currentPlayButtonReading != lastPlayButtonReading) {
    lastPlayButtonChangeTime = millis();  // Reset the debounce timer
  }

  // If the reading has been stable for DEBOUNCE_DELAY_MS
  if ((millis() - lastPlayButtonChangeTime) > DEBOUNCE_DELAY_MS) {
    // If the button is currently pressed (LOW after pullup) and has been stable
    if (currentPlayButtonReading == LOW) {
      // And enough time has passed since the last action trigger (cooldown)
      if ((millis() - last_play_button_trigger_time) > ACTION_COOLDOWN_MS) {
        last_play_button_trigger_time = millis();  // Reset cooldown timer

        if (currentMode == MODE_DANCE) {
            playNextDance();
        } else {
            Serial.println("Play button has no action in this mode.");
            Otto.sing(S_OhOoh);  // Provide feedback that Play button is not active
        }
      }
    }
  }
  lastPlayButtonReading = currentPlayButtonReading;  // Save current reading for next iteration


  // --- Continuous Mode Execution (Non-blocking parts of modes) ---
  switch (currentMode) {
    case MODE_APP:
      if (appModeEnabled) {
        appMode();
      }
      break;
    case MODE_AVOID:
      avoidModeLoop();
      break;
    case MODE_DANCE:
      // No continuous loop for dance, actions triggered by buttons
      break;
  }
}

// This function is called when the ActionButton is pressed.
// It handles transitioning to the next mode and initiating its entry actions.
void cycleToNextMode() {
  // Stop any ongoing actions or cleanup from the previous mode
  // For now, Otto.home() is a good general cleanup
  Otto.home();
  stopCurrentSong();
  stopDetectMode();  // Ensure detect mode's flags are reset

  // Cycle to the next mode
  currentMode = (Mode)((currentMode + 1) % MODE_COUNT);

  Serial.print("Switched to mode: ");
  Serial.println(currentMode);

  // Perform entry actions for the new mode
  switch (currentMode) {
    case MODE_APP:
      Serial.println("App mode active (waiting for BLE commands)");
      appModeEnabled = false;  // Reset appModeEnabled, it will be set when BLE is active.
      break;
    case MODE_DETECT:
      Serial.println("Entering Detect Mode");
      startDetectMode();  // Setup for continuous detection loop
      break;
    case MODE_SONG:
      Serial.println("Entering Song Mode");
      currentSongIndex = 0;  // Reset song index for the new mode entry
      executeCurrentSong();  // Play the first song
      break;
    case MODE_AVOID:
      Serial.println("Entering Avoid Mode");
      break;
    case MODE_FORCE:
      Serial.println("Entering Force Mode");
      break;
    case MODE_DANCE:
      Serial.println("Entering Dance Mode");
      currentDanceIndex = 0;  // Reset dance index for the new mode entry
      executeCurrentDance();  // Play the first dance routine
      break;
  }
}

long ultrasound() {
  long duration, distance;
  digitalWrite(TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);
  duration = pulseIn(ECHO, HIGH);
  distance = duration / 58;
  return distance;
}

void appMode() {
  // This will be filled later with BLE logic
  // For now, it just prints and sets a flag that will be managed by BLE connection
  // Serial.println("Modo App activado"); // Avoid printing repeatedly in loop
  // appModeEnabled = true; // This flag should be set by BLE connection status
}

bool detectModeActive = false;
unsigned long lastDetectActionTime = 0;
const unsigned long detectInterval = 1000;
void startDetectMode() {
  detectModeActive = true;
  lastDetectActionTime = millis();
}
void stopDetectMode() {
  detectModeActive = false;
  Otto.home();
}
void detectModeLoop() {
  if (!detectModeActive) return;

  unsigned long now = millis();
  if ((now - lastDetectActionTime > detectInterval)) {
    int distance = ultrasound();
    if (distance > 0 && distance < 15) {
      Serial.print("Object detected at: ");
      Serial.print(distance);
      Serial.println(" cm");
      Otto.sing(S_surprise);
      Otto.playGesture(OttoConfused);
      Otto.flapping(2, 1000, 15, 1);
      Otto.shakeLeg(1, 2000, 1);
      Otto.home();                 // This will block the loop for the duration of these movements
      lastDetectActionTime = now;  // Reset timer after action
    }
  }
}

std::vector<char*> songs = {
  "Super Mario:d=4,o=5,b=100:16e6,16e6,32p,8e6,16c6,8e6,8g6,8p,8g,8p,8c6,16p,8g,16p,8e,16p,8a,8b,16a#,8a,16g.,16e6,16g6,8a6,16f6,8g6,8e6,16c6,16d6,8b,16p,8c6,16p,8g,16p,8e,16p,8a,8b,16a#,8a,16g.,16e6,16g6,8a6,16f6,8g6,8e6,16c6,16d6,8b,8p,16g6,16f#6,16f6,16d#6,16p,16e6,16p,16g#,16a,16c6,16p,16a,16c6,16d6,8p,16g6,16f#6,16f6,16d#6,16p,16e6,16p,16c7,16p,16c7,16c7,p,16g6,16f#6,16f6,16d#6,16p,16e6,16p,16g#,16a,16c6,16p,16a,16c6,16d6,8p,16d#6,8p,16d6,8p,16c6",
  "The Simpsons:d=4,o=5,b=160:c.6,e6,f#6,8a6,g.6,e6,c6,8a,8f#,8f#,8f#,2g,8p,8p,8f#,8f#,8f#,8g,a#.,8c6,8c6,8c6,c6",
  "Macarena:d=4,o=5,b=180:f,8f,8f,f,8f,8f,8f,8f,8f,8f,8f,8a,8c,8c,f,8f,8f,f,8f,8f,8f,8f,8f,8f,8d,8c,p,f,8f,8f,f,8f,8f,8f,8f,8f,8f,8f,8a,p,2c.6,a,8c6,8a,8f,p,2p",
  "Titanic:d=4,o=5,b=125:8f,8g,2a.,16a#,16a,16g,16f,8g,2c.6,8a.,8c.6,2d.6,8c.6,8a.,1g,2p,f.,8f,f,f,e,2f,f,e,2f,g,2a,2g,f.,8f,f,f,e,2f,f,1c",
  "20thCenFox:d=16,o=5,b=140:b,8p,b,b,2b,p,c6,32p,b,32p,c6,32p,b,32p,c6,32p,b,8p,b,b,b,32p,b,32p,b,32p,b,32p,b,32p,b,32p,b,32p,g#,32p,a,32p,b,8p,b,b,2b,4p,8e,8g#,8b,1c#6,8f#,8a,8c#6,1e6,8a,8c#6,8e6,1e6,8b,8g#,8a,2b",
  "TakeOnMe:d=4,o=4,b=160:8f#5,8f#5,8f#5,8d5,8p,8b,8p,8e5,8p,8e5,8p,8e5,8g#5,8g#5,8a5,8b5,8a5,8a5,8a5,8e5,8p,8d5,8p,8f#5,8p,8f#5,8p,8f#5,8e5,8e5,8f#5,8e5,8f#5,8f#5,8f#5,8d5,8p,8b,8p,8e5,8p,8e5,8p,8e5,8g#5,8g#5,8a5,8b5,8a5,8a5,8a5,8e5,8p,8d5,8p,8f#5,8p,8f#5,8p,8f#5,8e5,8e5",
  "Imperial:d=4,o=5,b=112:8g,16p,8g,16p,8g,16p,16d#.,32p,32a#.,8g,16p,16d#.,32p,32a#.,g,8p,32p,8d6,16p,8d6,16p,8d6,16p,16d#.6,32p,32a#.,8f#,16p,16d#.,32p,32a#.,g,8p,32p,8g6,16p,16g.,32p,32g.,8g6,16p,16f#.6,32p,32f.6,32e.6,32d#.6,16e6,8p,16g#,32p,8c#6,16p,16c.6,32p,32b.,32a#.,32a.,16a#,8p,16d#,32p,8f#,16p,16d#.,32p,32g.,8a#,16p,16g.,32p,32a#.,d6,8p,32p,8g6,16p,16g.,32p,32g.,8g6,16p,16f#.6,32p,32f.6,32e.6,32d#.6,16e6,8p,16g#,32p,8c#6,16p,16c.6,32p,32b.,32a#.,32a.,16a#,8p,16d#,32p,8f#,16p,16d#.,32p,32g.,8g,16p,16d#.,32p,32a#.,g",
  "StarWars:d=4,o=5,b=45:32p,32f#,32f#,32f#,8b.,8f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32e6,8c#.6,32f#,32f#,32f#,8b.,8f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32e6,8c#6",
  "greenday:d=4,o=6,b=50:32b5,32a,16g,16g,16g,32g,32g,16d,16d,16d,32d,32d,16e,8e,c,32b5,32a,8g,16g,32g,16g.,16d,16d,16d,32d,32d,16e,8e,c,32b5,32a,16g,16g,16g,32g,32g,16d,16d,16d,32d,32d,16e,8e,8c.",
  "KnightRider:d=4,o=5,b=125:16e,16p,16f,16e,16e,16p,16e,16e,16f,16e,16e,16e,16d#,16e,16e,16e,16e,16p,16f,16e,16e,16p,16f,16e,16f,16e,16e,16e,16d#,16e,16e,16e,16d,16p,16e,16d,16d,16p,16e,16d,16e,16d,16d,16d,16c,16d,16d,16d,16d,16p,16e,16d,16d,16p,16e,16d,16e,16d,16d,16d,16c,16d,16d,16d",
  "VanessaMae:d=4,o=6,b=70:32c7,32b,16c7,32g,32p,32g,32p,32d#,32p,32d#,32p,32c,32p,32c,32p,32c7,32b,16c7,32g#,32p,32g#,32p,32f,32p,16f,32c,32p,32c,32p,32c7,32b,16c7,32g,32p,32g,32p,32d#,32p,32d#,32p,32c,32p,32c,32p,32g,32f,32d#,32d,32c,32d,32d#,32c,32d#,32f,16g,8p,16d7,32c7,32d7,32a#,32d7,32a,32d7,32g,16d7,32p,32d7,32p,32d7,32p,16d7,32c7,32d7,32a#,32d7,32a,32d7,32g,16d7,32p,32d7,32p,32d7,32p,32g,32f,32d#, 32d,32c,32d,32d#,32c,32d#,32d,8c",
  "Birdy Song:o=5,d=16,b=100,b=100:g,g,a,a,e,e,8g,g,g,a,a,e,e,8g,g,g,a,a,c6,c6,8b,8b,8a,8g,8f,f,f,g,g,d,d,8f,f,f,g,g,d,d,8f,f,f,g,g,a,b,8c6,8a,8g,8e,4c",
  "Mozart:o=5,d=16,b=125,b=125:16d#,c#,c,c#,8e,8p,f#,e,d#,e,8g#,8p,a,g#,g,g#,d#6,c#6,c6,c#6,d#6,c#6,c6,c#6,4e6,8c#6,8e6,32b,32c#6,d#6,8c#6,8b,8c#6,32b,32c#6,d#6,8c#6,8b,8c#6,32b,32c#6,d#6,8c#6,8b,8a#,4g#,d#,32c#,c,c#,8e,8p,f#,e,d#",
  "Spiderman:o=6,d=4,b=200,b=200:c,8d#,g.,p,f#,8d#,c.,p,c,8d#,g,8g#,g,f#,8d#,c.,p,f,8g#,c7.,p,a#,8g#,f.,p,c,8d#,g.,p,f#,8d#,c,p,8g#,2g,p,8f#,f#,8d#,f,8d#,2c",
  "Adams Family:o=5,d=8,b=160,b=160:c,4f,a,4f,c,4b4,2g,f,4e,g,4e,g4,4c,2f,c,4f,a,4f,c,4b4,2g,f,4e,c,4d,e,1f,c,d,e,f,1p,d,e,f#,g,1p,d,e,f#,g,4p,d,e,f#,g,4p,c,d,e,f",
  "SuperMan:d=4,o=5,b=180:8g,8g,8g,c6,8c6,2g6,8p,8g6,8a.6,16g6,8f6,1g6,8p,8g,8g,8g,c6,8c6,2g6,8p,8g6,8a.6,16g6,8f6,8a6,2g.6,p,8c6,8c6,8c6,2b.6,g.6,8c6,8c6,8c6,2b.6,g.6,8c6,8c6,8c6,8b6,8a6,8b6,2c7,8c6,8c6,8c6,8c6,8c6,2c.6",
  "Indiana Jones:d=4,o=5,b=250:e,8p,8f,8g,8p,1c6,8p.,d,8p,8e,1f,p.,g,8p,8a,8b,8p,1f6,p,a,8p,8b,2c6,2d6,2e6,e,8p,8f,8g,8p,1c6,p,d6,8p,8e6,1f.6,g,8p,8g,e.6,8p,d6,8p,8g,e.6,8p,d6,8p,8g,f.6,8p,e6,8p,8d6,2c6",
  "Deep Purple-Smoke on the Water:o=4,d=4,b=112,b=112:c,d#,f.,c,d#,8f#,f,p,c,d#,f.,d#,c,2p,8p,c,d#,f.,c,d#,8f#,f,p,c,d#,f.,d#,c"
};
void executeCurrentSong() {
  Serial.print("Playing Song: ");
  Serial.println(currentSongIndex);
  if (currentSongIndex < songs.size()) {
    anyrtttl::nonblocking::begin(Buzzer, songs[currentSongIndex]);
  }
}

void playNextSong() {
  stopCurrentSong();
  currentSongIndex = (currentSongIndex + 1) % songs.size();
  executeCurrentSong();
}

void stopCurrentSong() {
  anyrtttl::nonblocking::stop();
  digitalWrite(Buzzer, LOW);
  Serial.println("Song stopped.");
}

void avoidModeLoop() {
  int distance = ultrasound();
  if (distance > 0 && distance < 15) {
    Otto.sing(S_surprise);
    Otto.playGesture(OttoConfused);
    Otto.walk(2, 1000, -1);  // This will block the loop for ~2 seconds
    Otto.turn(3, 1000, 1);   // This will block the loop for ~3 seconds
    delay(50);               // This will block the loop
  } else {
    Otto.walk(1, 1000, 1);  // This will block the loop for ~1 second
  }
}

void forceModeLoop() {
  int distance = ultrasound();
  if (distance > 0 && distance <= 10) {
    Otto.walk(1, 1000, -1);  // Blocking
  }
  if (distance > 10 && distance < 15) {
    Otto.home();  // Blocking
  }
  if (distance > 15 && distance < 30) {
    Otto.walk(1, 1000, 1);  // Blocking
  }
  if (distance < 0 || distance > 30) {
    Otto.home();  // Blocking
  }
}

// --- DANCE MODE ---
// Otto.home() should be called at the end of each routine.

void danceRoutine1() {
  Serial.println("Executing Dance Routine 1: Jitter & Moonwalker");
  Otto.playGesture(OttoSuperHappy);
  Otto.home();
  Otto.jitter(10, 500, 40);  // Blocking
  Otto.home();
  Otto.moonwalker(3, 1000, 25, 1);   //LEFT Blocking!!
  Otto.moonwalker(3, 1000, 25, -1);  //RIGHT Blocking!!
  Otto.home();
}

void danceRoutine2() {
  Serial.println("Executing Dance Routine 2: Ascending Turn & Tiptoe Swing");
  Otto.playGesture(OttoVictory);
  Otto.home();
  Otto.ascendingTurn(2, 500, 50);  // Blocking
  Otto.home();
  Otto.tiptoeSwing(2, 1000, 30);  // Blocking
  Otto.home();
}

void danceRoutine3() {
  Serial.println("Executing Dance Routine 3: Flapping & Crusaito");
  Otto.playGesture(OttoLove);
  Otto.home();
  Otto.flapping(2, 500, 40, 1);  // Blocking
  Otto.home();
  Otto.crusaito(2, 3000, 40, 1);  // Blocking
  Otto.home();
}

void danceRoutine4() {
  Serial.println("Executing Dance Routine 3: Jitter, Updown & Jump");
  Otto.playGesture(OttoWave);
  Otto.home();
  Otto.jitter(2, 1000, 20);  //(small T)
  Otto.home();
  Otto.updown(2, 1500, 20);  // 20 = H "HEIGHT of movement"T
  Otto.home();
  Otto.jump(1, 500);  // It doesn't really jumpl ;P
  Otto.home();
}

void danceRoutine5() {
  Serial.println("Executing Dance Routine 4: Shake Leg & Disconnection");
  Otto.playGesture(OttoMagic);
  Otto.home();
  Otto.shakeLeg(2, 1000, 1);  // Blocking
  Otto.home();
  Otto.swing(2, 1000, 20);
  Otto.home();
  Otto.sing(S_disconnection);  // Blocking
  delay(500);                  // This specific delay might be fine here if it's the very end of a routine.
}

// --- Vector of function pointers for dance routines ---
std::vector<void (*)()> danceRoutines = {
  danceRoutine1,
  danceRoutine2,
  danceRoutine3,
  danceRoutine4,
  danceRoutine5
};

void executeCurrentDance() {
  if (currentDanceIndex < danceRoutines.size()) {
    Serial.print("Executing Dance Routine: ");
    Serial.println(currentDanceIndex);
    danceRoutines[currentDanceIndex]();  // Execute the specific routine
  } else {
    Serial.println("Invalid dance routine index.");
  }
}

void playNextDance() {
  currentDanceIndex = (currentDanceIndex + 1) % danceRoutines.size();
  executeCurrentDance();
}
