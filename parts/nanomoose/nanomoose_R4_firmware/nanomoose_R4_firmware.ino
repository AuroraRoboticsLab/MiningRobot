/*
 Arduino Uno R4 wifi to robot motor shield driver.

 Uses AdaFruit motor shield.

Open issues:
  - Wifi IP reported on serial as 0.0.0.0 initially (press 'w' after startup, dhcp delay?)
  - AuroraRoboticsLab wifi connects, but IP still seems unreachable.

Resolved Issues:
  - Servo.h gave bad jitter
    - Fix was this PWM library https://github.com/PaulStoffregen/PWMServo  (with R4 typecast fix: )
  - AFMotor library doesn't work with R4, so we made a generic analogWrite version AFMotorG,
    - Might consider https://github.com/PhoenixSmaug/AFMotor-Shield-R4-Compatible

 Authors:
   Idea and integration by Dr. Orion Lawlor, lawlor@alaska.edu 2026-03-19 (Public Domain)
   Wifi and websockets example code by Google Gemini 3 Pro
*/
#include <WiFiS3.h> // builtin on R4
#include <WebSocketsServer.h> // WebSockets by Markus Sattler
#include <ArduinoJson.h> // ArduinoJson library

#include "PWMServo.h" // avoid servo jitter with hardware PWM, by https://github.com/PaulStoffregen/PWMServo
#include "AFMotorG.h" // Adafruit V1 Motor Shield library, my generic (G) version

// Brushless motor controller on SERVO2
PWMServo throwServo;
const int throwStopA = 90;
const int throwRangeA = 60; // range of PWM command values

// Create motor objects
AF_DCMotor M1(1);
AF_DCMotor M2(2);
AF_DCMotor M3(3);
AF_DCMotor M4(4);

int drive = 200; // drive power (when driving)
const char *dirnames[5]={"nul","FW","BW","B","R"};

void motorLog(char side,int speed,int dir)
{
  Serial.print(side);
  Serial.print(' ');
  Serial.print(speed);
  Serial.print(' ');
  Serial.println(dirnames[dir]);
}

// Set left side motors to this speed and direction
void motorLeft(int speed,int dir)
{
  M2.setSpeed(speed); M2.run(dir); // left side
  M3.setSpeed(speed); M3.run(dir);
  motorLog('L',speed,dir);
}

// Set right side motors to this speed and direction
void motorRight(int speed,int dir)
{
  M1.setSpeed(speed); M1.run(dir); // right side
  M4.setSpeed(speed); M4.run(dir);
  motorLog('R',speed,dir);
}









#include "arduino_secrets.h" 
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
const char ssid[] = SECRET_SSID;        // your network SSID (name)
const char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

// HTTP Server on port 80, WebSocket Server on port 81
WiFiServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// HTML & JavaScript Payload
const char htmlPage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Rover Control</title>
  <style>
    body { font-family: sans-serif; text-align: center; margin-top: 50px; }
    h1 { color: #333; }
    #status { font-weight: bold; color: blue; }
  </style>
</head>
<body>
  <h1>Rover Control</h1>
  <p id="status">Connecting...</p>
  <p>Use W, A, S, D to drive. X to run snowblower.</p>
  <script>
    // Dynamically grab the Arduino's IP address!
    const ws = new WebSocket('ws://' + window.location.hostname + ':81/');
    
    ws.onopen = () => document.getElementById('status').innerText = 'Connected!';
    ws.onclose = () => document.getElementById('status').innerText = 'Disconnected.';

    // Track key states (avoid duplicate sends on key repeat)
    let keys = {w: false, a: false, s: false, d: false, x: false, ' ': false};

    document.addEventListener('keydown', (e) => {
      let key = e.key.toLowerCase();
      if(keys.hasOwnProperty(key) && !keys[key]) {
        keys[key] = true;
        sendRobotState();
      }
    });

    document.addEventListener('keyup', (e) => {
      let key = e.key.toLowerCase();
      if(keys.hasOwnProperty(key)) {
        keys[key] = false;
        sendRobotState();
      }
    });

    function sendRobotState() {
      // Float math: W is 1.0, S is -1.0. Right-hand rule: A is 1.0 (left), D is -1.0 (right).
      let forwardVal = (keys.w ? 1.0 : 0.0) + (keys.s ? -1.0 : 0.0);
      let turnVal = (keys.a ? 1.0 : 0.0) + (keys.d ? -1.0 : 0.0);
      let throwVal = (keys.x ? 1.0 : 0.0);
      if (keys[' ']) forwardVal=turnVal=throwVal=0;
      
      if(ws.readyState === WebSocket.OPEN) {
        let payload = JSON.stringify({ forward: forwardVal, turn: turnVal, throw: throwVal });
        ws.send(payload);
      }
    }
  </script>
</body>
</html>
)rawliteral";


// Turn float drive speed (-1 .. +1) into PWM speed (0..255)
int pwm_from_float(float speed) {
  if (speed<0) speed=-speed;
  if (speed<0.1) return 0;
  if (speed>1.0) speed=1.0;
  return (int)(drive * speed);
}

// Drive direction from float speed
int dir_from_float(float speed) {
  if (speed>0.1) return FORWARD;
  if (speed<-0.1) return BACKWARD;
  return RELEASE;
}

// --- WEBSOCKET EVENT HANDLER ---
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    JsonDocument doc; 
    DeserializationError error = deserializeJson(doc, payload);

    if (!error) {
      float forwardCmd = doc["forward"] | 0.0f;
      float turnCmd = doc["turn"] | 0.0f;
      float throwCmd = doc["throw"] | 0.0f;

      float leftF = forwardCmd - turnCmd;
      float rightF = -(forwardCmd + turnCmd);

      throwServo.write(throwStopA+throwRangeA*throwCmd);
      
      motorLeft (pwm_from_float(leftF),dir_from_float(leftF));
      motorRight(pwm_from_float(rightF),dir_from_float(rightF));
    } else {
      Serial.println("Failed to parse JSON");
    }
  }
}





void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait for R4 serial to set up */ }
  delay(500); // wait a half second longer, so we can see our startup messages (hacky!)
  
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true);
  }

  Serial.print("Connecting to WiFi...");
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected!");

  // Create server
  server.begin();

  printWifiStatus();

  throwServo.attach(SERVO_PIN_A); // 9);
  throwServo.write(throwStopA);

  // Start HTTP and WebSocket servers
  webSocket.begin();
  
  // Attach the event handler
  webSocket.onEvent(webSocketEvent);
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void loop() {
  // 1. Keep the WebSocket server ticking
  webSocket.loop();

  // 2. Handle new HTTP clients (Serving the Webpage)
  WiFiClient client = server.available();
  if (client) {
    String currentLine = "";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        if (c == '\n') {
          // Empty line means the HTTP request is finished
          if (currentLine.length() == 0) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            client.print(htmlPage); // Send the payload
            break;
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    client.stop();
  }

  // Poll serial
  while (Serial.available()>0) {
    char c=Serial.read();
    switch(c) {
    case 'L': motorLeft(drive,BACKWARD); break;
    case 'l': motorLeft(drive,FORWARD); break;

    // right side motors need the opposite drive command
    case 'R': motorRight(drive,FORWARD); break;
    case 'r': motorRight(drive,BACKWARD); break;

    
    
    // spacebar stop
    case ' ': 
      motorLeft(0,RELEASE);
      motorRight(0,RELEASE);
      break; 
    
    // Drive power
    case 'd': case 'D':
      drive=Serial.parseInt();
      Serial.print("drive power to ");
      Serial.println(drive);
      break;

    // s: servo command in microseconds
    case 's': {
        int w = Serial.parseInt();
        if (w<=0) w=throwStopA;
        throwServo.write(w);
        Serial.println(w);
      }
      break;
    case '\n': case '\r': break;

    // Re-print wifi status
    case 'w': 
      printWifiStatus();
      break;

    default:
      Serial.println(F("Unknown command. \n"
        " Valid commands: lr (forward) LR (backward) lR (turn right) Lr (turn left) \n"
        "   space stops.  d255 sets full drive power.  s90 sends servo to center.\n"));
      break;
    }
  }
}


