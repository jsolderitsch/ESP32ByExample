/*
 *  This sketch sends Tello commands over UDP from a ESP32 device
 *
 */
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <Adafruit_SSD1306.h>
#include <Bluepad32.h>

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
#endif

//IP address to send UDP data to:
// either use the ip address of the server or
// a network broadcast address
const char *udpAddress = "192.168.10.1";
const int udpPort = 8889;

GamepadPtr myGamepad;
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

//The udp library class
WiFiUDP udp;

//Are we currently connected?
boolean connected;
boolean gpConnected = false;
boolean lastCommandOK = false;
boolean in_flight = false;
long last_command_millis = 0;
int disconnected_tick = 0;
uint8_t buffer[50];

Preferences preferences;

String saved_ssid;

int gpYaw = 0;
int gpThrottle = 0;
int gpRoll = 0;
int gpPitch = 0;

String gpCmd = "rc 0 0 0 0";
String lastGpCmd = "rc 0 0 0 0";

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedGamepad(GamepadPtr gp) {
  if (myGamepad == nullptr) {
    Serial.printf("CALLBACK: Gamepad is now connected\n");
    // Additionally, you can get certain gamepad properties like:
    // Model, VID, PID, BTAddr, flags, etc.
    GamepadProperties properties = gp->getProperties();
    Serial.printf("Gamepad model: %s, VID=0x%04x, PID=0x%04x\n",
                  gp->getModelName().c_str(), properties.vendor_id,
                  properties.product_id);
    myGamepad = gp;
    gpConnected = true;
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(gp->getModelName().c_str());
    display.println("connected");
    display.display();
  }
}

void onDisconnectedGamepad(GamepadPtr gp) {
  if (myGamepad == gp) {
    if (in_flight) {
      // Treat as emergency
      onKillButtonPressed();
    }
    Serial.printf("CALLBACK: Gamepad is disconnected\n");
    gpConnected = false;
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(gp->getModelName().c_str());
    display.println("disconnected");
    display.display();
    myGamepad = nullptr;
  }
}

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
    lastCommandOK = true;  // rc commands are always OK
    responseExpected = false;
    digitalWrite(COMMAND_TICK, LOW);
  } else {
    Serial.println(command);
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
      digitalWrite(COMMAND_TICK, LOW);
    }
  } else if (command.indexOf("command") >= 0) {  // no response
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("No command response: ");
    display.println("Restarting");
    display.display();
    lastCommandOK = false;
    digitalWrite(COMMAND_TICK, LOW);
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
        digitalWrite(LED_BATT_YELLOW, HIGH);
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

void run_flight_plan() {
  if (connected) {
    run_command("motoron", 10, 5000);
    run_command("motoroff", 10, 1000);
    run_command("takeoff", 40, 0);
    if (lastCommandOK) digitalWrite(IN_FLIGHT, HIGH);
    run_command("up 70", 20, 2000);
    run_command("cw 90", 20, 2000);
    run_command("ccw 90", 20, 2000);
    run_command("down 70", 20, 2000);
    run_command("land", 40, 0);
    if (lastCommandOK) digitalWrite(IN_FLIGHT, LOW);
  }
}

void run_flight_plan_A() {
  if (connected) {
    run_command("motoron", 10, 5000);
    run_command("motoroff", 10, 1000);
    run_command("takeoff", 40, 0);
    if (lastCommandOK) digitalWrite(IN_FLIGHT, HIGH);
    run_command("up 70", 20, 2000);
    run_command("forward 70", 20, 2000);
    run_command("left 70", 20, 2000);
    run_command("back 70", 20, 2000);
    run_command("right 70", 20, 2000);
    run_command("land", 40, 0);
    if (lastCommandOK) digitalWrite(IN_FLIGHT, LOW);
  }
}

void run_flight_plan_B() {
  if (connected) {
    run_command("motoron", 10, 5000);
    run_command("motoroff", 10, 1000);
    run_command("takeoff", 40, 0);
    if (lastCommandOK) digitalWrite(IN_FLIGHT, HIGH);
    // to-do: Add several flight commands here

    run_command("land", 40, 0);
    if (lastCommandOK) digitalWrite(IN_FLIGHT, LOW);
  }
}

void run_circle_flight() {
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

void onKillButtonPressed() {
  Serial.println("KILL button is pressed");
  if (in_flight) {
    run_command("emergency", 10, 0);
    digitalWrite(IN_FLIGHT, LOW);
    in_flight = false;
  }
}

void onTakeoffButtonPressed() {
  if (in_flight) {
    run_command("land", 20, 0);
    if (lastCommandOK) digitalWrite(IN_FLIGHT, LOW);
    in_flight = false;
  } else {
    run_command("takeoff", 40, 0);
    if (lastCommandOK) digitalWrite(IN_FLIGHT, HIGH);
    in_flight = true;
  }
}

void setup() {

  pinMode(LED_CONN_RED, OUTPUT);
  pinMode(LED_CONN_GREEN, OUTPUT);
  pinMode(LED_BATT_RED, OUTPUT);
  pinMode(LED_BATT_YELLOW, OUTPUT);
  pinMode(LED_BATT_GREEN, OUTPUT);
  pinMode(COMMAND_TICK, OUTPUT);
  pinMode(IN_FLIGHT, OUTPUT);

  // Initilize hardware serial:
  Serial.begin(115200);
  // Serial.setTimeout(0);

  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2],
                addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
  // BP32.forgetBluetoothKeys();

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

  animateEEKLEDs();
  digitalWrite(LED_CONN_RED, HIGH);
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

  BP32.update();

  if (gpConnected) {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...

    if (myGamepad->a() && myGamepad->y()) {  //press up and down at the same time
      ESP.restart();
    }

    if (myGamepad->b() && myGamepad->x()) {  //press CW and CCW at the same time
      if (connected && !in_flight) {
        run_circle_flight();
      }
    }

    if (myGamepad->l1()) {
      // Serial.println("Left Shoulder button pressed");
      onTakeoffButtonPressed();
    }

    if (myGamepad->r1()) {
      // Serial.println("Right Shoulder button pressed");
      onKillButtonPressed();
    }

    if (myGamepad->thumbL()) {
      Serial.println("Left Joystick Button pressed");
      if (connected && !in_flight) {
        // to-do: Run Flight Plan A here

      }
    }

    if (myGamepad->thumbR()) {
      Serial.println("Right Joystick Button pressed");
      if (connected && !in_flight) {
        // to-do: Run Flight Plan B here

      }
    }

    lastGpCmd = gpCmd;
    gpCmd = "rc 0 0 0 0";

    // process dpad and button values before joystick values
    if (in_flight) {
      if (myGamepad->a()) {
        // go Down
        gpCmd = "rc 0 0 -60 0";
      } else if (myGamepad->b()) {
        // yaw CW
        gpCmd = "rc 0 0 0 60";
      } else if (myGamepad->x()) {
        // to-do: yaw CCW here

      } else if (myGamepad->y()) {
        // go Up
        gpCmd = "rc 0 0 60 0";
      } else if (myGamepad->brake() || myGamepad->l2()) {
        // fly a CCW circular arc
        gpCmd = "rc 0 40 0 -100";
      } else if (myGamepad->throttle() || myGamepad->r2()) {
        // fly a CW circular arc
        gpCmd = "rc 0 40 0 100";
      } else if (myGamepad->dpad() == 0x01) {
        // go Forward
        gpCmd = "rc 0 60 0 0";
      } else if (myGamepad->dpad() == 0x02) {
        // go Backward
        gpCmd = "rc 0 -60 0 0";
      } else if (myGamepad->dpad() == 0x04) {
        // Go Right
        gpCmd = "rc 60 0 0 0";
      } else if (myGamepad->dpad() == 0x05) {
        // Go Forward Right
        gpCmd = "rc 60 60 0 0";
      } else if (myGamepad->dpad() == 0x06) {
        // to-do: Go Back Right here

      } else if (myGamepad->dpad() == 0x08) {
        // Go Left
        gpCmd = "rc -60 0 0 0";
      } else if (myGamepad->dpad() == 0x09) {
        // Go Forward Left
        gpCmd = "rc -60 60 0 0";
      } else if (myGamepad->dpad() == 0x0a) {
        // to-do: Go Back Left Here

      } else {  // no buttons or Dpad
        // process joystick values
        // to-do: change the max joystick speeds from 90 cm/sec to 80 cm/sec 

        gpYaw = map(myGamepad->axisX(), -512, 512, -90, 90);
        gpThrottle = map(myGamepad->axisY(), -512, 512, 90, -90);
        gpRoll = map(myGamepad->axisRX(), -512, 512, -90, 90);
        gpPitch = map(myGamepad->axisRY(), -512, 512, 90, -90);
        // experimental dead zone values
        if (abs(gpYaw) <= 15) {
          gpYaw = 0;
        }
        if (abs(gpThrottle) <= 15) {
          gpThrottle = 0;
        }
        if (abs(gpPitch) <= 15) {
          gpPitch = 0;
        }
        if (abs(gpRoll) <= 15) {
          gpRoll = 0;
        }
        if ((gpPitch != 0) || (gpRoll != 0) || (gpThrottle != 0) || (gpYaw != 0)) {  // use joystick values if some are non-zero
          gpCmd = "rc ";
          gpCmd = gpCmd + gpRoll + " " + gpPitch + " " + gpThrottle + " " + gpYaw;
        }
      }
      if (lastGpCmd != gpCmd) {
        run_command(gpCmd, 0, 0);
        // Serial.println(gpadCommand);
      }
    }
  }

  if ((millis() - last_command_millis) > 15000) {  // keep SDK alive
    if (connected) run_command("battery?", 10, 0);
  }

  vTaskDelay(1);
}
