/*
 *  This sketch sends Tello commands over UDP from a ESP32 device
 *
 */
#include "FS.h"
#include "SPIFFS.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <Adafruit_SSD1306.h>
#include <Button2.h>
#include <MPU6050_light.h>

// #define Huzzah
// #define Clone
#define PCB

#ifdef Huzzah
// LED configs Huzzah
#define LED_CONN_RED 12
#define IN_FLIGHT 27
#define LED_CONN_GREEN 33
#define LED_BATT_RED 26
#define LED_BATT_YELLOW 25
#define LED_BATT_GREEN 4
#define COMMAND_TICK 13
#define UP_PIN 32
#define TAKEOFF_PIN 15
#define CW_PIN 14
#define CCW_PIN 17
#define KILL_PIN 16
#define DOWN_PIN 21
#endif

#ifdef Clone
// LED configs Huzzah
#define LED_CONN_RED 13
#define IN_FLIGHT 12
#define LED_CONN_GREEN 27
#define LED_BATT_RED 26
#define LED_BATT_YELLOW 25
#define LED_BATT_GREEN 33
#define COMMAND_TICK 2
#define UP_PIN 18
#define TAKEOFF_PIN 32
#define CW_PIN 19
#define CCW_PIN 16
#define KILL_PIN 15
#define DOWN_PIN 17
#endif

#ifdef PCB
// LED configs Huzzah
#define LED_CONN_RED 26
#define IN_FLIGHT 25
#define LED_CONN_GREEN 21
#define LED_BATT_RED 27
#define LED_BATT_YELLOW 15
#define LED_BATT_GREEN 4
#define COMMAND_TICK 13
#define UP_PIN 34
#define TAKEOFF_PIN 33
#define CW_PIN 32
#define CCW_PIN 39
#define KILL_PIN 36
#define DOWN_PIN 14
#endif

#define VBATPIN 35
#define FORMAT_SPIFFS_IF_FAILED true

File flightFile;
const char *flightFilePath = "/flight_file.txt";

const float MAX_BATTERY_VOLTAGE = 4.2;  // Max LiPoly voltage of a 3.7 battery is 4.2

//IP address to send UDP data to:
// either use the ip address of the server or
// a network broadcast address
const char *udpAddress = "192.168.10.1";
const int udpPort = 8889;

MPU6050 mpu(Wire);  // Wire.h included by default for ESP32
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

Button2 takeoffButton, killButton;

int Roll;
int AbsRoll;
int Pitch;
int AbsPitch;
int Yaw;
int limitedPitch;
int limitedRoll;

String rcCmd = "rc 0 0 0 0";
String lastRcCmd = "rc 0 0 0 0";
String lastCommand;

int upState = 0;
int downState = 0;
int cwState = 0;
int ccwState = 0;

unsigned long last_since_takeoff = 0;
unsigned long this_since_takeoff = 0;
unsigned long takeoff_time = 0;
unsigned long commandDelay = 0;

//The udp library class
WiFiUDP udp;

//Are we currently connected?
boolean connected;
boolean lastCommandOK = false;
boolean in_flight = false;
long last_command_millis = 0;
boolean use_MPUtoFly = false;
boolean use_SDK_KeepAlive = true;

int disconnected_tick = 0;
uint8_t buffer[50];

Preferences preferences;

String saved_ssid;

void toggle_led(int ledToToggle) {
  // Toggle the state of the LED pin (write the NOT of the current state to the LED pin)
  digitalWrite(ledToToggle, !digitalRead(ledToToggle));
}

void animateEEKLEDs() {
  digitalWrite(LED_CONN_RED, HIGH);
  delay(250);
  digitalWrite(IN_FLIGHT, HIGH);
  delay(250);
  digitalWrite(LED_CONN_GREEN, HIGH);
  delay(250);
  digitalWrite(LED_BATT_GREEN, HIGH);
  delay(250);
  digitalWrite(LED_BATT_YELLOW, HIGH);
  delay(250);
  digitalWrite(LED_BATT_RED, HIGH);
  delay(250);
  digitalWrite(LED_CONN_RED, LOW);
  digitalWrite(IN_FLIGHT, LOW);
  digitalWrite(LED_CONN_GREEN, LOW);
  digitalWrite(LED_BATT_GREEN, LOW);
  digitalWrite(LED_BATT_YELLOW, LOW);
  digitalWrite(LED_BATT_RED, LOW);
}

void run_command(String command, int udp_delay_ticks, int waitAfterDelay) {
  int packetSize = 0;
  lastCommandOK = false;
  last_command_millis = millis();
  boolean responseExpected = true;
  Serial.println(command);
  display.clearDisplay();
  display.setCursor(0, 0);
  digitalWrite(COMMAND_TICK, LOW);
  display.println("Command:");
  display.println(command);
  display.display();
  // Special delay cases
  if (command.indexOf("takeoff") >= 0) udp_delay_ticks = 40;
  if (command.indexOf("land") >= 0) udp_delay_ticks = 20;
  if (command.indexOf("rc ") >= 0) {
    udp_delay_ticks = 0;
    responseExpected = false;
    lastCommandOK = true;  // rc commands assumed to be OK
    digitalWrite(COMMAND_TICK, LOW);
  }
  memset(buffer, 0, 50);
  command.getBytes(buffer, command.length() + 1);
  //only send data when connected
  //Send a packet
  udp.beginPacket(udpAddress, udpPort);
  udp.write(buffer, command.length() + 1);
  udp.endPacket();
  // Serial.println("endPacket called");
  // allow for an rc command to not need any further processing after sending.
  memset(buffer, 0, 50);
  for (int x = 0; x < udp_delay_ticks; x++) {
    delay(500);
    toggle_led(COMMAND_TICK);
    // Serial.print(x);
    packetSize = udp.parsePacket();
    if (packetSize) break;
  }
  // Serial.println("packetSize: " + String(packetSize));
  if (packetSize && responseExpected) {
    if (udp.read(buffer, 50) > 0) {
      digitalWrite(COMMAND_TICK, LOW);
      String commandResponse = String((char *)buffer);
      Serial.println(commandResponse);
      display.println("Response:");
      display.println(commandResponse);
      display.display();
      lastCommandOK = true;
      bool parseResponse = (commandResponse.indexOf("forced stop") == -1)
                           && (commandResponse.indexOf("error") == -1) && (commandResponse.indexOf("ok") == -1);
      if (command.equalsIgnoreCase("battery?") && parseResponse) {
        int battery = commandResponse.toInt();
        if (battery > 70) {
          digitalWrite(LED_BATT_GREEN, HIGH);
          digitalWrite(LED_BATT_RED, LOW);
          digitalWrite(LED_BATT_YELLOW, LOW);
        } else if (battery > 40) {
          digitalWrite(LED_BATT_GREEN, LOW);
          digitalWrite(LED_BATT_RED, LOW);
          digitalWrite(LED_BATT_YELLOW, HIGH);
        } else {
          digitalWrite(LED_BATT_GREEN, LOW);
          digitalWrite(LED_BATT_RED, HIGH);
          digitalWrite(LED_BATT_YELLOW, LOW);
        }
      } else if (commandResponse.indexOf("timeout") >= 0) {
        digitalWrite(COMMAND_TICK, LOW);
        Serial.println("Command timed out, ignoring for now");
      } else if (commandResponse.indexOf("forced stop") >= 0) {
        digitalWrite(COMMAND_TICK, LOW);
        Serial.println("Unexpected forced stop Response; treat as error");
        lastCommandOK = false;
      } else if (commandResponse.indexOf("error") >= 0) {
        digitalWrite(COMMAND_TICK, LOW);
        Serial.println("Unexpected error response; treat as error");
        lastCommandOK = false;
      }
    } else if (in_flight) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("No command response: ");
      display.println("Landing NOW!");
      display.display();
      lastCommandOK = false;
    }
  } else if (command.indexOf("command") >= 0) {  // no response
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("No command response: ");
    display.println("Restarting");
    display.display();
    lastCommandOK = false;
  }
  delay(waitAfterDelay);
}

void connectToWiFi(const char *ssid) {
  Serial.println("Connecting to WiFi network: " + String(ssid));
  String ssidString = String((char *)ssid);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Connecting to:");
  display.println(ssidString);
  display.display();
  // delete old config
  WiFi.disconnect(true);
  disconnected_tick = 0;
  //Initiate connection
  //WiFi.begin(ssid, pwd);
  WiFi.begin(ssid);
  Serial.println("Waiting for WIFI connection...");
  int reconnectTick = 0;
  while ((WiFi.status() != WL_CONNECTED) && reconnectTick < 10) {
    delay(1000);
    Serial.print(".");
    display.print(".");
    display.display();
    reconnectTick++;
  }
  if (reconnectTick >= 10) {  // waited 10 seconds
    Serial.println("NOT CONNECTED");
    Serial.println("Use connect Command in Serial Monitor Input with active SSID");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("NOT CONNECTED");
    display.println("Use connect command");
    display.println("in Serial Monitor");
    display.println("With active SSID");
    display.display();
    delay(2000);
  }
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      //When connected set
      Serial.print("WiFi connected! IP address: ");
      Serial.println(WiFi.localIP());
      display.println(" CONNECTED");
      display.display();
      delay(1000);
      //in case this is a new SSID
      saved_ssid = WiFi.SSID();
      //initializes the UDP state
      //This initializes the transfer buffer
      udp.begin(WiFi.localIP(), udpPort);
      run_command("command", 10, 0);
      if (lastCommandOK) {
        connected = true;
        digitalWrite(LED_CONN_GREEN, HIGH);
        digitalWrite(LED_CONN_RED, LOW);
        run_command("battery?", 10, 0);
        disconnected_tick = 0;
        delay(1000);
      } else {
        ESP.restart();  // first "command" attempt failed
      }
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      disconnected_tick++;
      connected = false;
      if (disconnected_tick < 10) {
        Serial.println("WiFi not connected, expecting reconnection or Choose new SSID");
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Disconnected from:");
        display.println(saved_ssid.c_str());
        display.println("Waiting...");
        display.println("Choose New SSID?");
        display.display();
        digitalWrite(LED_CONN_GREEN, LOW);
        digitalWrite(LED_BATT_GREEN, LOW);
        digitalWrite(LED_BATT_YELLOW, LOW);
        digitalWrite(LED_BATT_RED, LOW);
      }
      digitalWrite(LED_CONN_RED, HIGH);
      delay(2000);
      break;
    default:
      delay(1000);
      break;
  }
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("- failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Appending to file: %s\r\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("- failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("- message appended");
  } else {
    Serial.println("- append failed");
  }
  file.close();
}

void deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("Deleting file: %s\r\n", path);
  if (fs.remove(path)) {
    Serial.println("- file deleted");
  } else {
    Serial.println("- delete failed");
  }
}

void appendLastCommand() {
  this_since_takeoff = (millis() - takeoff_time);
  commandDelay = this_since_takeoff - last_since_takeoff;
  lastCommand = lastCommand + "," + commandDelay + "\n";
  appendFile(SPIFFS, flightFilePath, lastCommand.c_str());
  last_since_takeoff = this_since_takeoff;
}

void processCommand(String command) {
  appendLastCommand();
  run_command(command, 0, 0);
  lastCommand = command;
}

void run_flight_plan() {
  if (connected) {
    run_command("motoron", 10, 5000);
    run_command("motoroff", 10, 1000);
    run_command("takeoff", 40, 0);
    if (lastCommandOK) digitalWrite(IN_FLIGHT, HIGH);
    run_command("rc 0 0 0 0", 0, 1000);
    run_command("rc 0 50 0 -100", 0, 4000);  // fly a CCW circular arc
    run_command("rc 0 0 0 0", 0, 1000);
    run_command("land", 40, 0);
    if (lastCommandOK) digitalWrite(IN_FLIGHT, LOW);
  }
}

void onKillButtonPressed(Button2 &btn) {
  Serial.println("KILL button is pressed");
  if (!connected) {
    Serial.println("Kill Button Pressed, no connection, restarting");
    ESP.restart();
  }
  if (in_flight) {
    run_command("emergency", 10, 0);
    digitalWrite(IN_FLIGHT, LOW);
    in_flight = false;
    deleteFile(SPIFFS, flightFilePath);
  } else {
    // run saved flight file if there is one
    flightFile = SPIFFS.open(flightFilePath, FILE_READ);
    if (flightFile) {
      Serial.println("Start of Flight File...");
      while (flightFile.available()) {
        String command = flightFile.readStringUntil('\n');
        int commaPosition = command.indexOf(',');
        if (commaPosition != -1) {  // delay value present
          commandDelay = command.substring(commaPosition + 1, command.length()).toInt();
          // Serial.println(command);
          // Serial.println(command.substring(0, commaPosition));
          // Serial.println(commandDelay);
          run_command(command.substring(0, commaPosition), 20, commandDelay);
        } else {
          run_command(command, 40, 0);
          // Serial.println(command);
        }
        if (command.indexOf("takeoff") >= 0) {
            digitalWrite(IN_FLIGHT, HIGH);
            in_flight = true;
          } else if (command.indexOf("land") >= 0) {
            digitalWrite(IN_FLIGHT, LOW);
            in_flight = false;
          }
        // Serial.println(command);
      }
      Serial.println("... end of Flight File");
      flightFile.close();
    }
  }
}

void onTakeoffButtonPressed(Button2 &btn) {
  Serial.println("Takeoff button is pressed");
  if (in_flight) {
    appendLastCommand();
    run_command("land", 20, 0);
    appendFile(SPIFFS, flightFilePath, "land,4\n");
    digitalWrite(IN_FLIGHT, LOW);
    in_flight = false;
  } else {
    writeFile(SPIFFS, flightFilePath, "command,2\n");
    appendFile(SPIFFS, flightFilePath, "battery?,4\n");
    run_command("takeoff", 40, 0);
    digitalWrite(IN_FLIGHT, HIGH);
    takeoff_time = millis();
    last_since_takeoff = 0;
    in_flight = true;
    lastCommand = "takeoff";
    appendLastCommand();  // will be takeoff
  }
}

void setup() {

  Wire.begin();

#ifdef Huzzah
  pinMode(UP_PIN, INPUT_PULLUP);
  pinMode(DOWN_PIN, INPUT_PULLUP);
  pinMode(CW_PIN, INPUT_PULLUP);
  pinMode(CCW_PIN, INPUT_PULLUP);
#endif

// PCB board is not using the pullup resistors!!
#ifdef PCB
  pinMode(UP_PIN, INPUT);
  pinMode(DOWN_PIN, INPUT);
  pinMode(CW_PIN, INPUT);
  pinMode(CCW_PIN, INPUT);
#endif

  pinMode(LED_CONN_RED, OUTPUT);
  pinMode(LED_CONN_GREEN, OUTPUT);
  pinMode(LED_BATT_RED, OUTPUT);
  pinMode(LED_BATT_YELLOW, OUTPUT);
  pinMode(LED_BATT_GREEN, OUTPUT);
  pinMode(COMMAND_TICK, OUTPUT);
  pinMode(IN_FLIGHT, OUTPUT);

  // Initialize each button.
  takeoffButton.begin(TAKEOFF_PIN);
  takeoffButton.setTapHandler(onTakeoffButtonPressed);
  killButton.begin(KILL_PIN);
  killButton.setTapHandler(onKillButtonPressed);

  // Initilize hardware serial:
  Serial.begin(115200);
  // Serial.setTimeout(0);

  if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
    Serial.println("SPIFFS Mount Failed");
    return;
  }

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  display.display();
  delay(2000);  // Pause for 2 seconds
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setRotation(0);
  display.clearDisplay();
  display.setCursor(0, 0);

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  display.println("MPU6050 status: ");
  Serial.println(status);
  display.println(status);
  display.display();

  while (status != 0) {}  // stop everything if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  display.println("Hold EEK still");
  mpu.calcOffsets();  // gyro and accelero
  Serial.println("Done!\n");
  display.println("Done!");
  display.display();
  delay(2000);

  animateEEKLEDs();
  digitalWrite(LED_CONN_RED, HIGH);
  use_MPUtoFly = false;
  use_SDK_KeepAlive = true;

  int rawValue = analogRead(VBATPIN);
  float voltageLevel = (rawValue / 4095.0) * 2 * 1.1 * 3.3;  // calculate voltage level
  int batteryFraction = min(100, int(trunc(voltageLevel / MAX_BATTERY_VOLTAGE * 100)));
  Serial.print("Controller Battery %: ");
  Serial.println(batteryFraction);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Controller Batt %:");
  display.println(batteryFraction);
  display.display();
  delay(2000);

  connected = false;
  WiFi.mode(WIFI_STA);
  //register event handler
  WiFi.onEvent(WiFiEvent);
  preferences.begin("tello-control", false);
  saved_ssid = preferences.getString("tello_ssid", "");
  display.clearDisplay();
  display.setCursor(0, 0);
  if (saved_ssid == "") {
    Serial.println("No value saved for Tello SSID");
    display.println("No saved Tello SSID");
    display.println("Use connect command");
    display.println("in Serial Monitor");
    display.println("With active SSID");
    display.display();
    delay(2000);
  } else {
    Serial.print("Using saved Tello SSID: ");
    Serial.println(saved_ssid);
    display.println("Saved Tello SSID:");
    display.println(saved_ssid);
    display.display();
    delay(1000);
    connectToWiFi(saved_ssid.c_str());
  }
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    char SSID[65];
    if (command.length() == 0) command = Serial.readStringUntil('\r');
    if (command.length() > 0) {
      command.trim();
      if (command.startsWith("connect")) {
        command.replace("connect", "");
        command.trim();
        strcpy(SSID, command.c_str());
        preferences.putString("tello_ssid", command);
        connectToWiFi(command.c_str());
      } else if (connected) {
        if (command.indexOf("takeoff") >= 0) {
          run_command(command, 40, 0);
          digitalWrite(IN_FLIGHT, HIGH);
          in_flight = true;
        } else if (command.indexOf("land") >= 0) {
          run_command(command, 40, 0);
          digitalWrite(IN_FLIGHT, LOW);
          in_flight = false;
        } else {
          run_command(command, 40, 0);
        }
      }
    }
  }

  if (!lastCommandOK) {
    if (in_flight && connected) {
      Serial.println("Command Error: Attempt to Land");
      run_command("land", 40, 0);
      digitalWrite(IN_FLIGHT, LOW);
      in_flight = false;
    }
    lastCommandOK = true;  // clear Command Error
  }

// change these states to !digitalRead for the 2024 form of the Breadboard
#ifdef PCB
  upState = digitalRead(UP_PIN);
  downState = digitalRead(DOWN_PIN);
  cwState = digitalRead(CW_PIN);
  ccwState = digitalRead(CCW_PIN);
#endif

#ifdef Huzzah
  upState = !digitalRead(UP_PIN);
  downState = !digitalRead(DOWN_PIN);
  cwState = !digitalRead(CW_PIN);
  ccwState = !digitalRead(CCW_PIN);
#endif

  takeoffButton.loop();
  killButton.loop();

  mpu.update();
  Roll = mpu.getAngleX();
  Pitch = mpu.getAngleY();
  Yaw = mpu.getAngleZ();

  lastRcCmd = rcCmd;
  rcCmd = "rc 0 0 0 0";  // default is hover

  if (in_flight) {
    if (upState == HIGH) {
      rcCmd = "rc 0 0 60 0";
    } else if (downState == HIGH) {
      rcCmd = "rc 0 0 -60 0";
    } else if (cwState == HIGH) {
      rcCmd = "rc 0 0 0 60";
    } else if (ccwState == HIGH) {
      rcCmd = "rc 0 0 0 -60";
    } else {  // no buttons pressed
      if (use_MPUtoFly) {
        AbsPitch = abs(Pitch);
        AbsRoll = abs(Roll);
        // set dead zone for EEK tilt angle
        if (AbsRoll <= 10) {
          limitedRoll = 0;
        } else {
          AbsRoll = constrain(AbsRoll, 30, 50);
          switch (AbsRoll) {
            case 30:
              AbsRoll = 20;
              break;
            case 50:
              AbsRoll = 60;
              break;
            default:
              AbsRoll = 40;
              break;
          }
          if (Roll < -10) {
            // go right
            limitedRoll = AbsRoll;
          } else if (Roll > 10) {
            // go left
            limitedRoll = -AbsRoll;
          }
        }
        // set dead zone for EEK tilt angle
        if (AbsPitch <= 15) {
          limitedPitch = 0;
        } else {
          AbsPitch = constrain(AbsPitch, 30, 50);
          switch (AbsPitch) {
            case 30:
              AbsPitch = 20;
              break;
            case 50:
              AbsPitch = 60;
              break;
            default:
              AbsPitch = 40;
              break;
          }
          if (Pitch < -15) {
            // go forward
            limitedPitch = AbsPitch;
          } else if (Pitch > 15) {
            // go back
            limitedPitch = -AbsPitch;
          }
        }

        if ((limitedRoll != 0) || (limitedPitch != 0)) {
          rcCmd = "rc ";
          rcCmd = rcCmd + limitedRoll + " " + limitedPitch + " 0 0";
        }
      }
    }
    if (lastRcCmd != rcCmd) {
      lastCommand = lastRcCmd;
      appendLastCommand();
      run_command(rcCmd, 0, 0);
      //      Serial.println(goCommand);
    } else {
      lastCommand = "rc 0 0 0 0";  //default last command: hover
    }

  } else {  // not in_flight
    if (upState == HIGH) {
      Serial.println("Up button is pressed when not in flight");
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Up Button Pressed");
      display.println("Running Flight Plan");
      display.display();
      delay(2000);
      run_flight_plan();
    } else if (downState == HIGH) {
      Serial.println("Down button is pressed when not in flight");
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Down Button Pressed");
      display.println("Changing MPU Status");
      if (use_MPUtoFly) {
        use_MPUtoFly = false;
        display.println("MPU Flying Disabled");
      } else {
        use_MPUtoFly = true;
        display.println("MPU Flying Enabled");
      }
      display.display();
      delay(2000);
    } else if (ccwState == HIGH) {
      // Serial.println("CCW button is pressed when not in flight");
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("CCW Button Pressed");
      display.println("Changing Keep Alive");
      display.println("Status");
      if (use_SDK_KeepAlive) {
        use_SDK_KeepAlive = false;
        display.println("Keep Alive Disabled");
      } else {
        use_SDK_KeepAlive = true;
        display.println("Keep Alive Enabled");
      }
      display.display();
      delay(2000);
    }
  }

  if (use_SDK_KeepAlive && ((millis() - last_command_millis) > 15000)) {  // keep SDK alive
    if (connected) run_command("battery?", 10, 0);
  }

  //delay(500);
}
