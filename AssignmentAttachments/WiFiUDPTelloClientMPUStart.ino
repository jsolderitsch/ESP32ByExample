/*
 *  This sketch sends Tello commands over UDP from a ESP32 device
 *
 */
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>

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

//IP address to send UDP data to:
// either use the ip address of the server or
// a network broadcast address
const char *udpAddress = "192.168.10.1";
const int udpPort = 8889;

Adafruit_MPU6050 mpu;
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

int Xacceleration;
int Yacceleration;
int Zacceleration;

//The udp library class
WiFiUDP udp;

//Are we currently connected?
boolean connected;
boolean in_flight = false;
boolean command_error = false;
long last_command_millis = 0;
boolean use_MPUtoFly = false;
int takeoff_btn_state = 0;
int cw_btn_state = 0;
int ccw_btn_state = 0;
int up_btn_state = 0;
int down_btn_state = 0;
int kill_btn_state = 0;
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

void run_command(String command, int udp_delay_ticks) {
  int packetSize = 0;
  last_command_millis = millis();
  boolean responseExpected = true;
  command_error = false;
  display.clearDisplay();
  display.setCursor(0, 0);
  digitalWrite(COMMAND_TICK, LOW);
  Serial.println(command);
  display.println("Command:");
  display.println(command);
  display.display();
  // Special delay cases
  if (command.indexOf("takeoff") >= 0) udp_delay_ticks = 40;
  if (command.indexOf("land") >= 0) udp_delay_ticks = 20;
  if (command.indexOf("rc ") >= 0) {
    udp_delay_ticks = 0;
    responseExpected = false;
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
      bool parseResponse = (commandResponse.indexOf("forced stop") == -1)
                           && (commandResponse.indexOf("error") == -1) 
                           && (commandResponse.indexOf("ok") == -1);
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
        command_error = true;
      } else if (commandResponse.indexOf("error") >= 0) {
        digitalWrite(COMMAND_TICK, LOW);
        Serial.println("Unexpected error response; treat as error");
        command_error = true;
      }
    } else if (in_flight) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("No command response: ");
      display.println("Landing NOW!");
      display.display();
      command_error = true;
    }
  } else if (command.indexOf("command") >= 0) {  // no response
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("No command response: ");
    display.println("Restarting");
    display.display();
    command_error = true;
  }

  //  delay(100);
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
      run_command("command", 10);
      if (!command_error) {
        connected = true;
        digitalWrite(LED_CONN_GREEN, HIGH);
        digitalWrite(LED_CONN_RED, LOW);
        run_command("battery?", 10);
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

void setup() {

#ifdef PCB
  pinMode(UP_PIN, INPUT);
  pinMode(TAKEOFF_PIN, INPUT);
  pinMode(CW_PIN, INPUT);
  pinMode(CCW_PIN, INPUT);
  pinMode(KILL_PIN, INPUT);
  pinMode(DOWN_PIN, INPUT);
#else
  pinMode(UP_PIN, INPUT_PULLUP);
  pinMode(TAKEOFF_PIN, INPUT_PULLUP);
  pinMode(CW_PIN, INPUT_PULLUP);
  pinMode(CCW_PIN, INPUT_PULLUP);
  pinMode(KILL_PIN, INPUT_PULLUP);
  pinMode(DOWN_PIN, INPUT_PULLUP);
#endif

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

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

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

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }

  Serial.println("");
  delay(100);

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
          run_command(command, 40);
          digitalWrite(IN_FLIGHT, HIGH);
          in_flight = true;
        } else if (command.indexOf("land") >= 0) {
          run_command(command, 40);
          digitalWrite(IN_FLIGHT, LOW);
          in_flight = false;
        } else {
          run_command(command, 40);
        }
      }
    }
  }

#ifdef PCB
  // buttons are inputs
  takeoff_btn_state = digitalRead(TAKEOFF_PIN);
  cw_btn_state = digitalRead(CW_PIN);
  ccw_btn_state = digitalRead(CCW_PIN);
  down_btn_state = digitalRead(DOWN_PIN);
  up_btn_state = digitalRead(UP_PIN);
  kill_btn_state = digitalRead(KILL_PIN);
#else
  // buttons are input pullups
  takeoff_btn_state = !digitalRead(TAKEOFF_PIN);
  cw_btn_state = !digitalRead(CW_PIN);
  ccw_btn_state = !digitalRead(CCW_PIN);
  down_btn_state = !digitalRead(DOWN_PIN);
  up_btn_state = !digitalRead(UP_PIN);
  kill_btn_state = !digitalRead(KILL_PIN);
#endif

  if (command_error) {
    if (in_flight && connected) {
      Serial.println("Command Error: Attempt to Land");
      run_command("land", 40);
      run_command("battery?", 30);
      digitalWrite(IN_FLIGHT, LOW);
      in_flight = false;
    }
    command_error = false;
  }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  Xacceleration = a.acceleration.x;
  Yacceleration = a.acceleration.y;
  Zacceleration = a.acceleration.z;
  // Tello nose direction is pilot perspective.
  if (connected) {
    if ((millis() - last_command_millis) > 15000) { // keep SDK alive
      run_command("battery?", 10);
    } else {
      if (!in_flight) {
        if (kill_btn_state == HIGH) {
          Serial.println("KILL button is pressed when not in flight");
          display.clearDisplay();
          display.setCursor(0, 0);
          display.println("Kill Button Pressed");
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
        }
      } else {  // in_flight
        if (use_MPUtoFly) {
          if ((Yacceleration == 0) && (Xacceleration > 4)) {
            run_command("forward 70", 20);
          } else if ((Yacceleration == 0) && (Xacceleration < -4)) {
            // do a back 70 command here similar to the forward 70 command
          } else if ((Xacceleration == 0) && (Yacceleration < -3)) {
            run_command("right 70", 20);
          } else if ((Xacceleration == 0) && (Yacceleration > 3)) {
            run_command("left 70", 20);
          }
        }
        if (up_btn_state == HIGH) {
          run_command("up 70", 20);
        }
        if (down_btn_state == HIGH) {
          run_command("down 70", 20);
        }
        if (cw_btn_state == HIGH) {
          run_command("cw 90", 20);
        }
        if (ccw_btn_state == HIGH) {
          // do the ccw command here
        }
        if (kill_btn_state == HIGH) {
          Serial.println("KILL button is pressed");
          run_command("emergency", 20);
          digitalWrite(IN_FLIGHT, LOW);
          digitalWrite(LED_CONN_GREEN, LOW);
          digitalWrite(LED_CONN_RED, HIGH);
          in_flight = false;
        }
      }
    }

    if (takeoff_btn_state == HIGH) {
      if (in_flight) {
        run_command("land", 20);
        digitalWrite(IN_FLIGHT, LOW);
        in_flight = false;
      } else {
        run_command("takeoff", 40);
        digitalWrite(IN_FLIGHT, HIGH);
        in_flight = true;
      }
    }
  }


  //delay(500);
}
