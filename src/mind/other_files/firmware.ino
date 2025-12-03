/* ESP32 WebSocket + RoboEyes + UDP discovery
   - Sends IP info to new WS clients
   - Broadcasts UDP discovery packet every 5s so clients can find the ESP32 before connecting
   - Supports JSON commands:
       {"mode":"text","text":"Hello","x":0,"y":20,"size":2}
       {"mode":"eyes","mood":"happy","direction":"left"}
*/

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>

// ------- DISPLAY must be declared BEFORE including RoboEyes header -------
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#include "FluxGarage_RoboEyes.h"   // your header
roboEyes eyes;          // class name in that header is `roboEyes`

// ------- WiFi / WebSocket / UDP -------
const char* ssid     = "PMP32";      // change to your Wi-Fi
const char* password = "9640521625";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
WiFiUDP udp;
const uint16_t UDP_PORT = 4210;
int wsClientCount = 0;


// Helper: log to Serial + WS (if clients exist)
void logMessage(const String &msg) {
  Serial.println(msg);
  ws.textAll(msg);
}

// compute broadcast IP from IP + subnet
IPAddress getBroadcastIP() {
  IPAddress ip = WiFi.localIP();
  IPAddress mask = WiFi.subnetMask();
  IPAddress bc;
  for (int i = 0; i < 4; ++i) {
    bc[i] = (ip[i] & mask[i]) | (~mask[i]);
  }
  return bc;
}

// send a small UDP announcement JSON (broadcast)
void sendUdpAnnouncement() {
  IPAddress b = getBroadcastIP();
  String payload = String("{\"name\":\"esp32\",\"ip\":\"") + WiFi.localIP().toString() +
                   String("\",\"ws\":\"/ws\",\"mdns\":\"esp32.local\"}");
  udp.beginPacket(b, UDP_PORT);
  udp.print(payload);
  udp.endPacket();
}

enum PomodoroState { POMODORO_IDLE, POMODORO_WORK, POMODORO_BREAK };
enum DisplayMode { MODE_EYES, MODE_TEXT, MODE_TIMER, MODE_STOPWATCH, MODE_POMODORO };
DisplayMode currentMode = MODE_EYES;

// Timer variables
bool timerIsRunning = false;
unsigned long timerDuration = 0; // in milliseconds
unsigned long timerStartTime = 0;
unsigned long timerPausedTime = 0; // time when paused

// Stopwatch variables
bool stopwatchIsRunning = false;
unsigned long stopwatchStartTime = 0;
unsigned long stopwatchPausedTime = 0; // time when paused

// Pomodoro variables
bool pomodoroIsRunning = false;
PomodoroState pomodoroCurrentState = POMODORO_IDLE;
unsigned long pomodoroPhaseDuration = 0; // in milliseconds
unsigned long pomodoroPhaseStartTime = 0;
unsigned long pomodoroWorkDuration = 25 * 60 * 1000; // Default 25 minutes
unsigned long pomodoroBreakDuration = 5 * 60 * 1000; // Default 5 minutes

void pushTextToDisplay(const String &text, int x, int y, int size) {

  display.setTextSize(size);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(x, y);
  display.println(text);


  logMessage("[ESP32] OLED text: " + text);
}

// Timer functions
void updateTimerDisplay() {
  if (!timerIsRunning && timerPausedTime == 0) return; // Timer not active

  unsigned long currentTime = millis();
  unsigned long elapsedTime;

  if (timerIsRunning) {
    elapsedTime = currentTime - timerStartTime;
  } else { // Timer is paused
    elapsedTime = timerPausedTime - timerStartTime;
  }

  long remainingTime = (long)timerDuration - (long)elapsedTime;

  if (remainingTime <= 0) {
    remainingTime = 0;
    timerIsRunning = false;
    currentMode = MODE_EYES; // Revert to eyes mode when timer finishes
    logMessage("[ESP32] Timer finished!");
  }

  int seconds = (remainingTime / 1000) % 60;
  int minutes = (remainingTime / (1000 * 60)) % 60;
  int hours = (remainingTime / (1000 * 60 * 60));

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  if (hours > 0) {
    display.printf("%02d:%02d:%02d", hours, minutes, seconds);
  } else {
    display.printf("%02d:%02d", minutes, seconds);
  }
  display.display();
}

void start_timer(int duration) {
  currentMode = MODE_TIMER;
  timerDuration = (unsigned long)duration * 1000; // Convert to milliseconds
  timerStartTime = millis();
  timerPausedTime = 0;
  timerIsRunning = true;
  logMessage("[ESP32] Timer started for " + String(duration) + " seconds");
  updateTimerDisplay();
}

void pause_timer() {
  if (timerIsRunning) {
    timerPausedTime = millis();
    timerIsRunning = false;
    logMessage("[ESP32] Timer paused");
  }
}

void resume_timer() {
  if (!timerIsRunning && timerPausedTime > 0) {
    timerStartTime += (millis() - timerPausedTime); // Adjust start time
    timerIsRunning = true;
    timerPausedTime = 0;
    logMessage("[ESP32] Timer resumed");
  }
}

void stop_timer() {
  timerIsRunning = false;
  timerDuration = 0;
  timerStartTime = 0;
  timerPausedTime = 0;
  currentMode = MODE_EYES;
  display.clearDisplay();
  display.display();
  logMessage("[ESP32] Timer stopped");
}

// Stopwatch functions
void updateStopwatchDisplay() {
  if (!stopwatchIsRunning && stopwatchPausedTime == 0) return; // Stopwatch not active

  unsigned long currentTime = millis();
  unsigned long elapsedTime;

  if (stopwatchIsRunning) {
    elapsedTime = currentTime - stopwatchStartTime;
  } else { // Stopwatch is paused
    elapsedTime = stopwatchPausedTime - stopwatchStartTime;
  }

  int seconds = (elapsedTime / 1000) % 60;
  int minutes = (elapsedTime / (1000 * 60)) % 60;
  int hours = (elapsedTime / (1000 * 60 * 60));

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  if (hours > 0) {
    display.printf("%02d:%02d:%02d", hours, minutes, seconds);
  } else {
    display.printf("%02d:%02d", minutes, seconds);
  }
  display.display();
}

void start_stopwatch() {
  currentMode = MODE_STOPWATCH;
  stopwatchStartTime = millis();
  stopwatchPausedTime = 0;
  stopwatchIsRunning = true;
  logMessage("[ESP32] Stopwatch started");
  updateStopwatchDisplay();
}

void pause_stopwatch() {
  if (stopwatchIsRunning) {
    stopwatchPausedTime = millis();
    stopwatchIsRunning = false;
    logMessage("[ESP32] Stopwatch paused");
  }
}

void resume_stopwatch() {
  if (!stopwatchIsRunning && stopwatchPausedTime > 0) {
    stopwatchStartTime += (millis() - stopwatchPausedTime); // Adjust start time
    stopwatchIsRunning = true;
    stopwatchPausedTime = 0;
    logMessage("[ESP32] Stopwatch resumed");
  }
}

void stop_stopwatch() {
  stopwatchIsRunning = false;
  stopwatchStartTime = 0;
  stopwatchPausedTime = 0;
  currentMode = MODE_EYES;
  display.clearDisplay();
  display.display();
  logMessage("[ESP32] Stopwatch stopped");
}

void reset_stopwatch() {
  if (!stopwatchIsRunning) {
    stopwatchStartTime = 0;
    stopwatchPausedTime = 0;
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("00:00");
    display.display();
    logMessage("[ESP32] Stopwatch reset");
  }
}

// Pomodoro functions
void startPomodoroPhase(PomodoroState newState, unsigned long duration) {
  pomodoroCurrentState = newState;
  pomodoroPhaseDuration = duration;
  pomodoroPhaseStartTime = millis();
  pomodoroIsRunning = true;

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  if (newState == POMODORO_WORK) {
    display.println("Work Time!");
  } else if (newState == POMODORO_BREAK) {
    display.println("Break Time!");
  }
  display.display();
  logMessage("[ESP32] Pomodoro phase started: " + String(newState == POMODORO_WORK ? "Work" : "Break") + " for " + String(duration / 1000 / 60) + " minutes");
}

void updatePomodoroDisplay() {
  if (!pomodoroIsRunning) return;

  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - pomodoroPhaseStartTime;
  long remainingTime = (long)pomodoroPhaseDuration - (long)elapsedTime;

  if (remainingTime <= 0) {
    // Phase finished, transition to next phase
    if (pomodoroCurrentState == POMODORO_WORK) {
      startPomodoroPhase(POMODORO_BREAK, pomodoroBreakDuration);
    } else if (pomodoroCurrentState == POMODORO_BREAK) {
      startPomodoroPhase(POMODORO_WORK, pomodoroWorkDuration);
    }
    return;
  }

  int seconds = (remainingTime / 1000) % 60;
  int minutes = (remainingTime / (1000 * 60)) % 60; //this could be optimized to not using the division again and again

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  if (pomodoroCurrentState == POMODORO_WORK) {
    display.println("WORK");
  } else if (pomodoroCurrentState == POMODORO_BREAK) {
    display.println("BREAK");
  }
  display.setCursor(0, 20);
  display.printf("%02d:%02d", minutes, seconds);
  display.display();
}

void start_pomodoro(int workDuration, int breakDuration) {
  currentMode = MODE_POMODORO;
  pomodoroWorkDuration = (unsigned long)workDuration * 60 * 1000;
  pomodoroBreakDuration = (unsigned long)breakDuration * 60 * 1000;
  pomodoroIsRunning = true;
  startPomodoroPhase(POMODORO_WORK, pomodoroWorkDuration);
  logMessage("[ESP32] Pomodoro started: Work " + String(workDuration) + "min, Break " + String(breakDuration) + "min");
}

void pause_pomodoro() {
  if (pomodoroIsRunning) {
    pomodoroIsRunning = false;
    pomodoroPhaseStartTime = millis() - pomodoroPhaseStartTime; // Store elapsed time
    logMessage("[ESP32] Pomodoro paused");
  }
}

void resume_pomodoro() {
  if (!pomodoroIsRunning && pomodoroPhaseStartTime > 0) {
    pomodoroPhaseStartTime = millis() - pomodoroPhaseStartTime; // Restore start time
    pomodoroIsRunning = true;
    logMessage("[ESP32] Pomodoro resumed");
  }
}

void stop_pomodoro() {
  pomodoroIsRunning = false;
  pomodoroCurrentState = POMODORO_IDLE;
  pomodoroPhaseDuration = 0;
  pomodoroPhaseStartTime = 0;
  currentMode = MODE_EYES;
  display.clearDisplay();
  display.display();
  logMessage("[ESP32] Pomodoro stopped");
}

// ---- WebSocket message handler (JSON based) ----
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (!info) return;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    String jsonStr;
    for (size_t i = 0; i < len; i++) jsonStr += (char)data[i];

    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, jsonStr);
    if (error) {
      logMessage(String("[ESP32] JSON parse error: ") + error.c_str());
      return;
    }

    String mode = doc["mode"] | "eyes";

    if (mode == "text") {
      currentMode = MODE_TEXT;

      bool clear = doc["clear"] | true;  // clear screen before drawing

      if (clear) {
        display.clearDisplay();
      }

      // "items" is an array of objects
      JsonArray items = doc["items"].as<JsonArray>();
      for (JsonObject item : items) {
        String text = item["text"] | "";
        int x = item["x"] | 0;
        int y = item["y"] | 0;
        int size = item["size"] | 1;

        if (text.length() > 0) {
          pushTextToDisplay(text, x, y, size);
        }
      }

      // Update OLED once after drawing everything
      display.display();
    }

    else if (mode == "eyes") {
      currentMode = MODE_EYES;
      String mood = doc["mood"] | "happy";
      String direction = doc["direction"] | "center";

      display.clearDisplay();


      if (doc.containsKey("mood")) {
        String mood = doc["mood"].as<String>();
        if (mood.equalsIgnoreCase("happy")) eyes.setMood(HAPPY);
        else if (mood.equalsIgnoreCase("tired")) eyes.setMood(TIRED);
        else if (mood.equalsIgnoreCase("angry")) eyes.setMood(ANGRY);
        else if (mood.equalsIgnoreCase("confused")) eyes.anim_confused();
        else if (mood.equalsIgnoreCase("laugh")) eyes.anim_laugh();
        else eyes.setMood(DEFAULT);
      }


      if (doc.containsKey("direction")) {
        String dir = doc["direction"].as<String>();
        if (dir.equalsIgnoreCase("left")) eyes.setPosition(W);
        else if (dir.equalsIgnoreCase("right")) eyes.setPosition(E);
        else if (dir.equalsIgnoreCase("up")) eyes.setPosition(N);
        else if (dir.equalsIgnoreCase("down")) eyes.setPosition(S);
        else eyes.setPosition(DEFAULT);
      }


      display.display();
      logMessage("[ESP32] OLED eyes: " + mood + " " + direction);
    }

    else if (mode == "clear"){
      display.clearDisplay();
      display.display();
    }

    else if (mode == "clearRect") {
      int x = doc["x"] | 0;
      int y = doc["y"] | 0;
      int w = doc["w"] | SCREEN_WIDTH;
      int h = doc["h"] | SCREEN_HEIGHT;

      display.fillRect(x, y, w, h, SSD1306_BLACK);
      display.display();

      logMessage("[ESP32] OLED rect cleared at (" + String(x) + "," + String(y) +
                 ") size " + String(w) + "x" + String(h));
    }
    
    else if (mode == "timer"){
      String action = doc["action"];
      if (action == "start"){
        start_timer((int) doc["duration"]);
      }
      elif (action == "pause"){
        pause_timer();
      }
      elif (action == "resume"){
        resume_timer();
      }
      elif (action == "stop"){
        stop_timer();
      }
    }

    else if (mode == "stopwatch"){
      String action = doc["action"];
      if (action == "start"){
        start_stopwatch();
      }
      else if (action == "pause"){
        pause_stopwatch();
      }
      else if (action == "resume"){
        resume_stopwatch();
      }
      else if (action == "stop"){
        stop_stopwatch();
      }
      else if (action == "reset"){
        reset_stopwatch();
      }
    }

    else if (mode == "pomodoro"){
      String action = doc["action"];
      if (action == "start"){
        int workDuration = doc["workDuration"] | 25; // Default 25 minutes
        int breakDuration = doc["breakDuration"] | 5; // Default 5 minutes
        start_pomodoro(workDuration, breakDuration);
      }
      else if (action == "pause"){
        pause_pomodoro();
      }
      else if (action == "resume"){
        resume_pomodoro();
      }
      else if (action == "stop"){
        stop_pomodoro();
      }
    }



  }
}

// WebSocket events
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
             AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    wsClientCount++;
    // Send status info to just this client
    client->text(String("[ESP32] Connected, IP: ") + WiFi.localIP().toString());
    client->text(String("[ESP32] mDNS: http://esp32.local"));
    logMessage("[ESP32] New WebSocket client connected");
  } else if (type == WS_EVT_DISCONNECT) {
    wsClientCount--;
    logMessage("[ESP32] WebSocket client disconnected, total: " + String(wsClientCount));

    if (wsClientCount <= 0) {
      // fallback to eyes mode
      currentMode = MODE_EYES;
      display.clearDisplay();
      eyes.setMood(DEFAULT);   // or HAPPY if you prefer
      eyes.setPosition(DEFAULT);
      display.display();
      logMessage("[ESP32] No clients left, reverting to eyes mode");
    }

  } else if (type == WS_EVT_DATA) {
    handleWebSocketMessage(arg, data, len);
  }
}

void setup() {
  Serial.begin(115200);
  delay(10);

  // init display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[ESP32] SSD1306 allocation failed");
    for (;;) { delay(1000); } // halt
  }
  display.clearDisplay();
  display.display();

  // init RoboEyes
  // Start robo eyes
  eyes.begin(SCREEN_WIDTH, SCREEN_HEIGHT, 100);
  eyes.setAutoblinker(ON, 3, 2);
  eyes.setIdleMode(ON, 2, 2);  // startup mood

  display.display();

  // start Wi-Fi
  WiFi.begin(ssid, password);
  logMessage("[ESP32] Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(50);
    eyes.update();
    Serial.print(".");
  }
  logMessage("[ESP32] WiFi connected, IP: " + WiFi.localIP().toString());

  // mDNS
  if (!MDNS.begin("esp32")) {
    logMessage("[ESP32] mDNS start failed");
  } else {
    logMessage("[ESP32] mDNS started: http://esp32.local");
  }

  // start WebSocket server
  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.begin();
  logMessage("[ESP32] WebSocket server started at /ws");

  // start UDP (bind to UDP_PORT so we can broadcast)
  udp.begin(UDP_PORT);
  logMessage("[ESP32] UDP announcements on port " + String(UDP_PORT));
}



void loop() {
  ws.cleanupClients();
  if (currentMode == MODE_EYES) {
    eyes.update();
  } else if (currentMode == MODE_TIMER) {
    updateTimerDisplay();
  } else if (currentMode == MODE_STOPWATCH) {
    updateStopwatchDisplay();
  } else if (currentMode == MODE_POMODORO) {
    updatePomodoroDisplay();
  }

  static unsigned long lastBeat = 0;
  static unsigned long lastUdp = 0;
  unsigned long now = millis();

  if (now - lastBeat > 5000) {
    logMessage("[ESP32] Alive at " + String(now / 1000) + "s");
    lastBeat = now;
  }

  if (now - lastUdp > 5000) {
    // broadcast UDP announcement (so clients can discover IP before WS connect)
    sendUdpAnnouncement();
    lastUdp = now;
  }

  delay(10);
}
