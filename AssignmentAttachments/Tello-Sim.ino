/* 
 * Joseph DeMarco Jim Solderitsch
 * 
 * Tello Drone Simulator Using Dev Kit Clone or Huzzah
 *
*/
#include <WiFi.h>
#include <WiFiUdp.h>
#include "Wire.h"
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>

#define Clone
// #define Huzzah

#ifdef Clone
#define INFLIGHT 14
#define CW 25
#define DOWN 26
#define CCW 27
#define UP 12
#define LEFT 33
#define FORWARD 15
#define BACK 32
#define RIGHT 4
#define CONNECTED 13
//String serialNumber = "DEVSNTELLOSIMA0";
#endif

#ifdef Huzzah
#define INFLIGHT 21
#define CW 33
#define DOWN 16
#define CCW 12
#define UP 27
#define LEFT 15
#define FORWARD 32
#define BACK 17
#define RIGHT 14
#define CONNECTED 19
String serialNumber = "HUZSNTELLOSIMA";
#endif

WiFiUDP Udp;  // Creation of wifi Udp instance

char packetBuffer[255];
char responseBuffer[255];

unsigned int localPort = 8889;
unsigned long batTimer = 0;
unsigned long motorStart = 0;
unsigned long lastCommandTime = 0;
int batLevel = 100;
int roll, pitch, throttle, yaw;
int motorOn = 0;
int SDKtimeOut = 0;
bool SDKenabled = true;
unsigned long simStartFlightTime = 0;
unsigned long simFlightTime = 0;
unsigned long simTotalFlightTime = 0;
int inFlight = 0;
int commandMode = 0;
int speed = 100;

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

//const char *ssid = "TELLO-SIM000";
const char *password = "";

IPAddress local_IP(192, 168, 10, 1);
IPAddress gateway(192, 168, 10, 1);
IPAddress subnet(255, 255, 255, 0);

void toggle_led(int ledToToggle) {
  // Toggle the state of the LED pin (write the NOT of the current state to the LED pin)
  digitalWrite(ledToToggle, !digitalRead(ledToToggle));
  delay(250);
  digitalWrite(ledToToggle, !digitalRead(ledToToggle));
  delay(250);
}

void animateSimLEDs() {
  digitalWrite(CONNECTED, HIGH);
  digitalWrite(INFLIGHT, HIGH);
  analogWrite(UP, 255);
  delay(250);
  analogWrite(CW, 255);
  delay(250);
  analogWrite(DOWN, 255);
  delay(250);
  analogWrite(CCW, 255);
  delay(250);
  analogWrite(UP, 0);
  analogWrite(CW, 0);
  analogWrite(DOWN, 0);
  analogWrite(CCW, 0);
  analogWrite(FORWARD, 255);
  delay(250);
  analogWrite(RIGHT, 255);
  delay(250);
  analogWrite(BACK, 255);
  delay(250);
  analogWrite(LEFT, 255);
  delay(250);
  analogWrite(FORWARD, 0);
  analogWrite(RIGHT, 0);
  analogWrite(BACK, 0);
  analogWrite(LEFT, 0);
  digitalWrite(INFLIGHT, LOW);
  digitalWrite(CONNECTED, LOW);
}

void rcResponse(String command) {
  String rcParameters = "";
  // extract the numbers
  int index = 0;
  int rcValues[4];
  index = command.indexOf(' ');
  rcParameters = command.substring(index + 1);
  for (int i = 0; i < 4; i++) {
    index = rcParameters.indexOf(' ');
    rcValues[i] = rcParameters.substring(0, index).toInt();
    rcParameters = rcParameters.substring(index + 1);
  }
  roll = rcValues[0];
  pitch = rcValues[1];
  throttle = rcValues[2];
  yaw = rcValues[3];
  //  Serial.println(roll);
  //  Serial.println(pitch);
  //  Serial.println(throttle);
  //  Serial.println(yaw);
  if (throttle > 0) {  //up
    analogWrite(UP, map(throttle, 0, 100, 0, 255));
  }
  if (pitch > 0) {  //forward
    analogWrite(FORWARD, map(pitch, 0, 100, 0, 255));
  }
  if (throttle < 0) {  //down
    analogWrite(DOWN, map(abs(throttle), 0, 100, 0, 255));
  }
  if (pitch < 0) {  //back
    analogWrite(BACK, map(abs(pitch), 0, 100, 0, 255));
  }
  if (yaw < 0) {  //ccw
    analogWrite(CCW, map(abs(yaw), 0, 100, 0, 255));
  }
  if (roll < 0) {  //left
    analogWrite(LEFT, map(abs(roll), 0, 100, 0, 255));
  }
  if (yaw > 0) {  //cw
    analogWrite(CW, map(yaw, 0, 100, 0, 255));
  }
  if (roll > 0) {  //right
    analogWrite(RIGHT, map(roll, 0, 100, 0, 255));
  }
  if (roll == 0 && pitch == 0 and throttle == 0 and yaw == 0) {  //hover, stop LEDs
    analogWrite(UP, 0);
    analogWrite(DOWN, 0);
    analogWrite(CW, 0);
    analogWrite(CCW, 0);
    analogWrite(FORWARD, 0);
    analogWrite(BACK, 0);
    analogWrite(LEFT, 0);
    analogWrite(RIGHT, 0);
  }
}

void setup() {
  pinMode(INFLIGHT, OUTPUT);
  pinMode(CONNECTED, OUTPUT);
  pinMode(CW, OUTPUT);
  pinMode(CCW, OUTPUT);
  pinMode(FORWARD, OUTPUT);
  pinMode(BACK, OUTPUT);
  pinMode(UP, OUTPUT);
  pinMode(DOWN, OUTPUT);
  pinMode(LEFT, OUTPUT);
  pinMode(RIGHT, OUTPUT);
  digitalWrite(INFLIGHT, LOW);
  digitalWrite(CONNECTED, LOW);
  digitalWrite(CW, LOW);
  digitalWrite(CCW, LOW);
  digitalWrite(FORWARD, LOW);
  digitalWrite(BACK, LOW);
  digitalWrite(UP, LOW);
  digitalWrite(DOWN, LOW);
  digitalWrite(LEFT, LOW);
  digitalWrite(RIGHT, LOW);

  Serial.begin(115200);
  while (!Serial) {
    delay(10);
    if (millis() > 5000) break;  // don't wait forever
  }
  delay(500);
  if (Serial) Serial.println("\nSerial Connected");

  //  WiFi.mode(WIFI_MODE_AP);
  Serial.print("Setting soft-AP configuration ... ");
  Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Ready" : "Failed!");
  // WiFi.softAP(ssid);  // ESP-32 as access point
  Serial.print("Setting soft-AP ... ");
  Serial.println(WiFi.softAP(ssid) ? "Ready" : "Failed!");
  Serial.print("Soft-AP IP address = ");
  Serial.println(WiFi.softAPIP());
  Serial.print("Serial Number: ");
  Serial.println(serialNumber);

  Wire.begin();

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  display.display();
  delay(1000);  // Pause for 1 second
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setRotation(0);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Beginning Tello");
  display.println("Simulation");
  display.display();
  delay(2000);
  animateSimLEDs();
  motorOn = 0;
  commandMode = 0;
  inFlight = 0;
  SDKenabled = true;

  Udp.begin(localPort);
}

void loop() {
  if (motorOn) {
    toggle_led(INFLIGHT);
  }
  if ((millis() - batTimer) > 30000) {
    batLevel = batLevel - 2;
    batTimer = millis();
  }
  if (SDKtimeOut) {
    toggle_led(CONNECTED);
  }

  if (commandMode && SDKenabled) {
    if ((millis() - lastCommandTime) > 21000) {
      SDKtimeOut++;
      if (SDKtimeOut == 1) {          // print once
        digitalWrite(INFLIGHT, LOW);  // In Flight status not valid
        Serial.println("SDK Timeout; no new command in over 20 seconds");
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("SDK TimeOut");
        display.println("No new command");
        display.println("In over 15 seconds");
        display.println("Restart Tello-Sim");
        display.display();
      }
    } else if (SDKtimeOut > 0) {
      Serial.println("SDK Timeout Cleared");
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("New SDK command");
      display.println("Received");
      display.display();
      digitalWrite(CONNECTED, HIGH);
      SDKtimeOut = 0;
    }
  }

  int packetSize = Udp.parsePacket();
  if (packetSize) {
    lastCommandTime = millis();
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Received:");
    display.println(packetBuffer);
    display.display();
    String command = String((char *)packetBuffer);
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    if (command.indexOf("command") >= 0) {
      commandMode = 1;
      batTimer = millis();
      digitalWrite(CONNECTED, HIGH);
      Udp.printf("ok");
      Udp.endPacket();
    } else if (command.indexOf("sdktimeoff") >= 0) {
      SDKenabled = false;
      Udp.printf("ok");
      Udp.endPacket();
    } else if (command.indexOf("sdktimeon") >= 0) {
      SDKenabled = true;
      Udp.printf("ok");
      Udp.endPacket();
    } else if (commandMode) {
      Serial.print("Received(IP/Size/Data): ");
      Serial.print(Udp.remoteIP());
      Serial.print(" / ");
      Serial.print(packetSize);
      Serial.print(" / ");
      Serial.println(packetBuffer);
      if ((command.indexOf("rc") >= 0) && inFlight) {  // do rc command first
        rcResponse(command);
      } else {
        if (command.indexOf("battery?") >= 0) {
          // Udp.printf("100\r\n");
          String batString = String(batLevel) + "\r\n";
          batString.toCharArray(responseBuffer, batString.length() + 1);
          Udp.printf(responseBuffer);
        } else if (command.indexOf("reboot") >= 0) {
          ESP.restart();
        } else if (command.indexOf("time?") >= 0) {
          String timeString = String(simTotalFlightTime / 1000) + "s\r\n";
          timeString.toCharArray(responseBuffer, timeString.length() + 1);
          Udp.printf(responseBuffer);
        } else if (command.indexOf("speed?") >= 0) {
          String speedString = String(speed) + "\r\n";
          speedString.toCharArray(responseBuffer, speedString.length() + 1);
          Udp.printf(responseBuffer);
        } else if (command.indexOf("speed") >= 0) {
          int speedPos = command.indexOf(' ') + 1;
          speed = command.substring(speedPos).toInt();
          Udp.printf("ok");
        } else if (command.indexOf("ssid?") >= 0) {
          Udp.printf(ssid);
        } else if (command.indexOf("wifi?") >= 0) {
          String snrString = String(random(50, 90)) + "\r\n";
          snrString.toCharArray(responseBuffer, snrString.length() + 1);
          Udp.printf(responseBuffer);
        } else if (command.indexOf("sdk?") >= 0) {
          String sdkString = "30";
          sdkString.toCharArray(responseBuffer, sdkString.length() + 1);
          Udp.printf(responseBuffer);
        } else if (command.indexOf("sn?") >= 0) {
          serialNumber.toCharArray(responseBuffer, serialNumber.length() + 1);
          Udp.printf(responseBuffer);
        } else if (command.indexOf("temp?") >= 0) {
          String tempString = String((int)(temperatureRead() + 0.5)) + "C\r\n";
          tempString.toCharArray(responseBuffer, tempString.length() + 1);
          Udp.printf(responseBuffer);
        } else if (command.indexOf("motoron") >= 0) {
          motorStart = millis();  // mark time of motor on
          motorOn = 1;
          delay(1000);
          digitalWrite(INFLIGHT, HIGH);  // temporary
          Udp.printf("ok");
        } else if (command.indexOf("motoroff") >= 0) {
          motorOn = 0;
          delay(1000);
          digitalWrite(INFLIGHT, LOW);
          Udp.printf("ok");
        } else if ((command.indexOf("takeoff") >= 0) && !inFlight) {
          motorStart = millis();  // mark time of motor start
          inFlight = 1;
          simStartFlightTime = millis();
          delay(5000);
          digitalWrite(INFLIGHT, HIGH);
          Udp.printf("ok");
        } else if (inFlight) {
          if (command.indexOf("land") >= 0) {
            inFlight = 0;
            simFlightTime = millis() - simStartFlightTime;
            simTotalFlightTime = simTotalFlightTime + simFlightTime;
            delay(4000);
            digitalWrite(INFLIGHT, LOW);
            Udp.printf("ok");
          } else if (command.indexOf("emergency") >= 0) {  // stop all LEDs
            toggle_led(CONNECTED);
            digitalWrite(CONNECTED, HIGH);
            digitalWrite(INFLIGHT, LOW);
            analogWrite(UP, 0);
            analogWrite(DOWN, 0);
            analogWrite(CW, 0);
            analogWrite(CCW, 0);
            analogWrite(FORWARD, 0);
            analogWrite(BACK, 0);
            analogWrite(LEFT, 0);
            analogWrite(RIGHT, 0);
            delay(5000);
            digitalWrite(CONNECTED, LOW);
            inFlight = 0;
            commandMode = 0;
            Udp.printf("ok");
          } else if (command.indexOf("stop") >= 0) {
            analogWrite(UP, 0);
            analogWrite(DOWN, 0);
            analogWrite(CW, 0);
            analogWrite(CCW, 0);
            analogWrite(FORWARD, 0);
            analogWrite(BACK, 0);
            analogWrite(LEFT, 0);
            analogWrite(RIGHT, 0);
            Udp.printf("ok");
          } else if (command.indexOf("ccw") >= 0) {  // catch ccw first because cw satisfies ccw
            analogWrite(CCW, 255);
            delay(3000);
            analogWrite(CCW, 0);
            Udp.printf("ok");
          } else if (command.indexOf("cw") >= 0) {
            analogWrite(CW, 255);
            delay(3000);
            analogWrite(CW, 0);
            Udp.printf("ok");
          } else if (command.indexOf("up") >= 0) {
            analogWrite(UP, 255);
            delay(3000);
            analogWrite(UP, 0);
            Udp.printf("ok");
          } else if (command.indexOf("down") >= 0) {
            analogWrite(DOWN, 255);
            delay(3000);
            analogWrite(DOWN, 0);
            Udp.printf("ok");
          } else if (command.indexOf("right") >= 0) {
            analogWrite(RIGHT, 255);
            delay(3000);
            analogWrite(RIGHT, 0);
            Udp.printf("ok");
          } else if (command.indexOf("left") >= 0) {
            analogWrite(LEFT, 255);
            delay(3000);
            analogWrite(LEFT, 0);
            Udp.printf("ok");
          } else if (command.indexOf("forward") >= 0) {
            analogWrite(FORWARD, 255);
            delay(3000);
            analogWrite(FORWARD, 0);
            Udp.printf("ok");
          } else if (command.indexOf("back") >= 0) {
            analogWrite(BACK, 255);
            delay(3000);
            analogWrite(BACK, 0);
            Udp.printf("ok");
          } else if (command.indexOf("go ") >= 0) {
            int parmsPos = command.indexOf(' ');
            String goParms = command.substring(parmsPos + 1);  // like "-40 40"
            int yPos = goParms.indexOf(' ');
            int x = goParms.substring(0, yPos).toInt();
            int y = goParms.substring(yPos + 1).toInt();
            if ((x > 0) && (y == 0)) {
              analogWrite(FORWARD, 255);
              delay(3000);
              analogWrite(FORWARD, 0);
            } else if ((x < 0) && (y == 0)) {
              analogWrite(BACK, 255);
              delay(3000);
              analogWrite(BACK, 0);
            } else if ((x == 0) && (y < 0)) {
              analogWrite(RIGHT, 255);
              delay(3000);
              analogWrite(RIGHT, 0);
            } else if ((x == 0) && (y > 0)) {
              analogWrite(LEFT, 255);
              delay(3000);
              analogWrite(LEFT, 0);
            } else if ((x > 0) && (y < 0)) {
              analogWrite(FORWARD, 255);
              analogWrite(RIGHT, 255);
              delay(3000);
              analogWrite(FORWARD, 0);
              analogWrite(RIGHT, 0);
            } else if ((x < 0) && (y < 0)) {
              analogWrite(BACK, 255);
              analogWrite(RIGHT, 255);
              delay(3000);
              analogWrite(BACK, 0);
              analogWrite(RIGHT, 0);
            } else if ((x > 0) && (y > 0)) {
              analogWrite(FORWARD, 255);
              analogWrite(LEFT, 255);
              delay(3000);
              analogWrite(FORWARD, 0);
              analogWrite(LEFT, 0);
            } else if ((x < 0) && (y > 0)) {
              analogWrite(BACK, 255);
              analogWrite(LEFT, 255);
              delay(3000);
              analogWrite(BACK, 0);
              analogWrite(LEFT, 0);
            }
            Udp.printf("ok");
          }
        } else {
          Udp.printf("error");
        }
        if (!(command.indexOf("rc ") >= 0)) {  //respond if not an rc command
          Udp.endPacket();
        }
      }
    }
    // vTaskDelay(1);
  }
}