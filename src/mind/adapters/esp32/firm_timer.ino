
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <ESPAsyncWiFiManager.h>
#include <DNSServer.h>
#include "time.h"

const long gmtOffset_sec = 19800;
const int daylightOffset_sec = 0;

#define BUTTON_PIN 26

bool buttonState = HIGH;
bool lastButtonState = HIGH;

unsigned long buttonPressStart = 0;
bool longPressTriggered = false;

const unsigned long LONG_PRESS_TIME = 1500;

#define BATTERY_PIN 34
static unsigned long lastBattery = 0;
float batteryVoltage = 0.0;




// ------- DISPLAY must be declared BEFORE including RoboEyes header -------
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#include "FluxGarage_RoboEyes.h"   // your header
roboEyes eyes;          // class name in that header is `roboEyes`



AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
WiFiUDP udp;
const uint16_t UDP_PORT = 4210;
int wsClientCount = 0;

DNSServer dns;
AsyncWiFiManager wifiManager(&server, &dns);

// Helper: log to Serial + WS (if clients exist)
void logMessage(const String &msg) {
  Serial.println(msg);
  ws.textAll(msg);
}

float readBatteryVoltage() {
  int raw = analogRead(BATTERY_PIN);
  float voltage = (raw / 4095.0) * 3.3 * 2;
  return voltage;
}

void pushTextToDisplay(const String &text, int x, int y, int size, bool log_it=false) {

  display.setTextSize(size);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(x, y);
  display.println(text);
  
  if (log_it){
  logMessage("[ESP32] OLED text: " + text);}
}

int getWiFiBars() {

  int rssi = WiFi.RSSI();

  if (rssi > -55) return 4;
  else if (rssi > -65) return 3;
  else if (rssi > -75) return 2;
  else if (rssi > -85) return 1;
  else return 0;
}

int getBatteryPercent(float voltage) {

  float minV = 3.3;
  float maxV = 4.2;

  int percent = (voltage - minV) * 100 / (maxV - minV);

  if (percent > 100) percent = 100;
  if (percent < 0) percent = 0;

  return percent;
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
enum DisplayMode {
  MODE_CLOCK,
  MODE_EYES,
  MODE_TEXT,
  MODE_TIMER,
  MODE_STOPWATCH,
  MODE_POMODORO
};

DisplayMode currentMode = MODE_CLOCK;

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
unsigned long pomodoroPhaseDuration = 0; // ms
unsigned long pomodoroPhaseStartTime = 0;
unsigned long pomodoroElapsed = 0;

unsigned long pomodoroWorkDuration = 25 * 60 * 1000;      // 25 min
unsigned long pomodoroShortBreakDuration = 5 * 60 * 1000; // 5 min
unsigned long pomodoroLongBreakDuration = 15 * 60 * 1000; // 15 min

int pomodoroCycleCount = 0; // 1-4
const int pomodoroMaxCycles = 4;
bool pomodoroIsLongBreak = false;


void drawWiFiBars(int bars) {

  int x = 105;
  int y = 0;

  for(int i=0;i<4;i++){

    int h = (i+1)*2;

    if(i < bars)
      display.fillRect(x+(i*5), y+(8-h), 3, h, SSD1306_WHITE);
    else
      display.drawRect(x+(i*5), y+(8-h), 3, h, SSD1306_WHITE);
  }
}

void drawBattery(int percent){

  int x = 98;
  int y = 54;

  display.drawRect(x,y,22,8,SSD1306_WHITE);
  display.fillRect(x+22,y+2,2,4,SSD1306_WHITE);

  int fill = map(percent,0,100,0,20);

  display.fillRect(x+1,y+1,fill,6,SSD1306_WHITE);
}

// Forward declaration for Pomodoro function
void startPomodoroPhase(PomodoroState newState, unsigned long duration, bool isLongBreak);


void updateClockDisplay() {

  static unsigned long lastUpdate = 0;

  if(millis() - lastUpdate < 1000) return;
  lastUpdate = millis();

  struct tm timeinfo;

  if(!getLocalTime(&timeinfo)){
    return;
  }

  char timeStr[6];
  char dayStr[4];

  strftime(timeStr, sizeof(timeStr), "%H:%M", &timeinfo);
  strftime(dayStr, sizeof(dayStr), "%a", &timeinfo);

  int battery = getBatteryPercent(batteryVoltage);
  int wifiBars = getWiFiBars();

  display.clearDisplay();

  pushTextToDisplay(dayStr, 0, 0, 1)

  // WiFi bars
  drawWiFiBars(wifiBars);

  // Clock
  display.setTextSize(3);
  display.setCursor(20,22);
  pushTextToDisplay(timeStr, 20, 22, 3)

  // Battery icon
  drawBattery(battery);

  // Charging indicator

  display.display();
}


// Timer functions
void updateTimerDisplay() {
  if (!timerIsRunning) return;
  unsigned long elapsed = millis() - timerStartTime;
  unsigned long remaining; 
  char buf[10];
  if (elapsed >= timerDuration){
    remaining = 0;
    display.clearDisplay();
    pushTextToDisplay("Timer", 0, 0, 1);
    pushTextToDisplay("Completed", 10, 32, 2);
    display.display();
    delay(1000);
    currentMode = MODE_EYES;
  }
  else{
    remaining = timerDuration - elapsed;
  }
  int h = remaining / 3600000;
  int m = (remaining % 3600000) / 60000;
  int s = (remaining % 60000) / 1000;
  sprintf(buf,"%02d:%02d:%02d" ,h ,m, s);
  display.clearDisplay();
  

  // Title (top-left, size 1)
  pushTextToDisplay("Timer", 0, 0, 1);

  // Time (center, size 2)
  pushTextToDisplay(buf, 20, (SCREEN_HEIGHT / 2) - 8, 2);

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
  if (!stopwatchIsRunning) return;
  unsigned long elapsed = millis() - stopwatchStartTime;
  char buf[10];

  int h = elapsed / 3600000;
  int m = (elapsed % 3600000) / 60000;
  int s = (elapsed % 60000) / 1000;
  sprintf(buf,"%02d:%02d:%02d" ,h ,m, s);
  display.clearDisplay();

  // Title
  pushTextToDisplay("StopWatch", 0, 0, 1);

  // Time
  pushTextToDisplay(buf, 20, (SCREEN_HEIGHT / 2) - 8, 2);

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
//lap has to be added
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


// Start a Pomodoro phase
void startPomodoroPhase(PomodoroState newState, unsigned long duration, bool isLongBreak = false) {
    pomodoroCurrentState = newState;
    pomodoroPhaseDuration = duration;
    pomodoroPhaseStartTime = millis();
    pomodoroIsRunning = true;
    pomodoroIsLongBreak = isLongBreak;

    display.clearDisplay();

    // Title top-left
    pushTextToDisplay("Pomodoro", 0, 0, 1);

    // Phase message center
    if (newState == POMODORO_WORK) pushTextToDisplay("WORK",(SCREEN_WIDTH / 2) - 30, (SCREEN_HEIGHT / 2) - 8, 2);
    else if (newState == POMODORO_BREAK) pushTextToDisplay(isLongBreak ? "LBREAK" : "BREAK", (SCREEN_WIDTH / 2) - 30, (SCREEN_HEIGHT / 2) - 8, 2);

    display.display();
    logMessage("[ESP32] Pomodoro phase: " + String(newState == POMODORO_WORK ? "WORK" : (isLongBreak ? "LBREAK" : "BREAK")));
    delay(500);
}

// Update Pomodoro display & transitions
void updatePomodoroDisplay() {
    if (!pomodoroIsRunning) return;

    unsigned long elapsed = millis() - pomodoroPhaseStartTime;
    long remaining = (pomodoroPhaseDuration > elapsed) ? (pomodoroPhaseDuration - elapsed) : 0;
    char buf[10];
    
    // Phase finished → decide next
    if (remaining == 0) {
        if (pomodoroCurrentState == POMODORO_WORK) {
            // Work finished → break
            if (pomodoroCycleCount < pomodoroMaxCycles) {
                startPomodoroPhase(POMODORO_BREAK, pomodoroShortBreakDuration);
            } else {
                startPomodoroPhase(POMODORO_BREAK, pomodoroLongBreakDuration, true);
            }
        } 
        else if (pomodoroCurrentState == POMODORO_BREAK) {
            // Break finished → next work or stop
            if (pomodoroIsLongBreak || pomodoroCycleCount >= pomodoroMaxCycles) {
                stop_pomodoro(); // Finished all cycles
            } else {
                pomodoroCycleCount++; // increment after short break
                startPomodoroPhase(POMODORO_WORK, pomodoroWorkDuration);
            }
        }
        return;
    }

    int h = remaining / 3600000;
    int m = (remaining % 3600000) / 60000;
    int s = (remaining % 60000) / 1000;
    sprintf(buf,"%02d:%02d:%02d" ,h ,m, s);
    display.clearDisplay();

    // Title top-left
    pushTextToDisplay("POMODORO", 0, 0, 1);

    // Timer center
    pushTextToDisplay(buf, 20, (SCREEN_HEIGHT / 2) - 8, 2);

    // Phase bottom-left
    pushTextToDisplay(pomodoroCurrentState == POMODORO_WORK ? "WORK" : (pomodoroIsLongBreak ? "LBREAK" : "BREAK"), 20, SCREEN_HEIGHT-10, 1);

    // Cycle count bottom-right
    pushTextToDisplay(String(pomodoroCycleCount), SCREEN_WIDTH - 20, SCREEN_HEIGHT-10, 1);

    display.display();
}

// Start Pomodoro
void start_pomodoro(int workMinutes, int shortBreakMinutes, int longBreakMinutes) {
    currentMode = MODE_POMODORO;
    pomodoroWorkDuration = workMinutes * 60 * 1000;
    pomodoroShortBreakDuration = shortBreakMinutes * 60 * 1000;
    pomodoroLongBreakDuration = longBreakMinutes * 60 * 1000;
    pomodoroCycleCount = 1;
    pomodoroIsRunning = true;
    startPomodoroPhase(POMODORO_WORK, pomodoroWorkDuration);
    logMessage("[ESP32] Pomodoro started: Work " + String(workMinutes) + "min, Break " + String(shortBreakMinutes) + "min, Long Break " + String(longBreakMinutes) + "min");
}

// Pause Pomodoro
void pause_pomodoro() {
    if (pomodoroIsRunning) {
        pomodoroElapsed = millis() - pomodoroPhaseStartTime;
        pomodoroIsRunning = false;
        logMessage("[ESP32] Pomodoro paused");
    }
}

// Resume Pomodoro
void resume_pomodoro() {
    if (!pomodoroIsRunning) {
        pomodoroPhaseStartTime = millis() - pomodoroElapsed;
        pomodoroIsRunning = true;
        logMessage("[ESP32] Pomodoro resumed");
    }
}

// Stop Pomodoro
void stop_pomodoro() {
    pomodoroIsRunning = false;
    pomodoroCurrentState = POMODORO_IDLE;
    pomodoroPhaseDuration = 0;
    pomodoroPhaseStartTime = 0;
    pomodoroIsLongBreak = false;
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
          pushTextToDisplay(text, x, y, size, true);
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
      else if (action == "pause"){
        pause_timer();
      }
      else if (action == "resume"){
        resume_timer();
      }
      else if (action == "stop"){
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
          int workDuration = doc["work"] | 25; // Default 25 minutes
          int breakDuration = doc["break"] | 5; // Default 5 minutes
          int longBreakDuration = doc["lbreak"] | 15;
          start_pomodoro(workDuration, breakDuration, longBreakDuration);
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
      currentMode = MODE_CLOCK;
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

void waitForTimeSync() {

  struct tm timeinfo;
  int retry = 0;

  while(!getLocalTime(&timeinfo) && retry < 20) {

    logMessage("Waiting for NTP...");
    delay(500);
    retry++;
  }

  if(retry < 20){
    logMessage("Time synced!");
  }else{
    logMessage("NTP FAILED");
  }
}



void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  delay(10);

  

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // init display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    logMessage("[ESP32] SSD1306 allocation failed");
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

  display.clearDisplay();
  pushTextToDisplay("WiFi Setup", 0, 0, 1);
  pushTextToDisplay("Connect to:", 0, 20, 1);
  pushTextToDisplay("ESP32-Setup", 0, 40, 1);
  display.display();
  wifiManager.setConnectTimeout(10);      // wait 10 sec for WiFi
  wifiManager.setConfigPortalTimeout(180); // portal active 3 minutes
  wifiManager.autoConnect("ESP32-Setup");


  if (WiFi.status() == WL_CONNECTED) {
    display.clearDisplay();
    pushTextToDisplay("Connected", 0, 0, 1);
    pushTextToDisplay(WiFi.localIP().toString(), 0, 20, 1);
    display.display();
    delay(1000);
    logMessage("[ESP32] WiFi connected");
    logMessage("[ESP32] IP: " + WiFi.localIP().toString());
    configTime(
      gmtOffset_sec,
      daylightOffset_sec,
      "pool.ntp.org",
      "time.nist.gov",
      "time.google.com"
    );
    waitForTimeSync();
}
 else {
  logMessage("[ESP32] WiFi connection failed, starting setup portal");
}




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

void handleButton() {

  buttonState = digitalRead(BUTTON_PIN);

  // Button pressed
  if (buttonState == LOW && lastButtonState == HIGH) {
    buttonPressStart = millis();
    longPressTriggered = false;
  
  }

  // Button held
  if (buttonState == LOW && !longPressTriggered) {

    if (millis() - buttonPressStart > LONG_PRESS_TIME) {

      longPressTriggered = true;

      // LONG PRESS ACTIONS

      if (currentMode == MODE_POMODORO) {

        if (pomodoroIsRunning)
          pause_pomodoro();
        else
          resume_pomodoro();
      }

      else if (currentMode == MODE_CLOCK) {

        // Long press from clock goes to eyes
        currentMode = MODE_EYES;
      }
    }
  }

  // Button released
  if (buttonState == HIGH && lastButtonState == LOW) {

    if (!longPressTriggered) {

      // SHORT PRESS ACTIONS

      switch(currentMode) {

        case MODE_CLOCK:
          currentMode = MODE_EYES;
          break;

        case MODE_EYES:
          currentMode = MODE_POMODORO;
          display.clearDisplay();
          pushTextToDisplay("Pomodoro", 20, (SCREEN_HEIGHT / 2) - 8, 2);
          break;

        case MODE_POMODORO:
          currentMode = MODE_CLOCK;
          break;

        default:
          currentMode = MODE_CLOCK;
          break;
      }

      display.clearDisplay();   // prevent artifacts
    }
  }

  lastButtonState = buttonState;
}


void loop() {

  ws.cleanupClients();
  handleButton();

  switch(currentMode) {

    case MODE_CLOCK:
      updateClockDisplay();
      break;

    case MODE_EYES:
      eyes.update();
      break;

    case MODE_TIMER:
      updateTimerDisplay();
      break;

    case MODE_STOPWATCH:
      updateStopwatchDisplay();
      break;

    case MODE_POMODORO:
      updatePomodoroDisplay();
      break;
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

    if (now - lastBattery > 5000) {
    batteryVoltage = readBatteryVoltage();
    lastBattery = now;
    logMessage("Battery "+ String(batteryVoltage) + "v");
  }


}


