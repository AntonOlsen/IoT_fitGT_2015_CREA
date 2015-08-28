/*
  WiFi Web Server Control Robot
  
  Based on the SimpleWebServerWiFi example.

  This example is written for a network using WPA encryption. For
  WEP or WPA, change the Wifi.begin() call accordingly.

  Circuit:
  * CC3200 WiFi LaunchPad or CC3100 WiFi BoosterPack
    with TM4C or MSP430 LaunchPad
  * Launchpad MSP430 5529 w/ CC3100 Booster and TI DRV8835 on a Pololu Carrier board.
  * Pins to DRV8835
  * P6_1 - AEN (enable)
  * P6_2 - APH (direction)
  * P6_3 - BEN (enable)
  * P6_4 - BPH (direction)
  
  * P4_1 - V+ for Left LED
  * P3_5 - GND for Left LED

  * P8_1 - V+ for Right LED
  * P8_2 - GND for Right LED
  
  * P6)0 - Analog Input

  created 25 Nov 2012
  by Tom Igoe
  modified 6 July 2014
  by Noah Luskey
  modified 26 August 2015 - Gutted the LED control and added Robot Control
  by Anton Olsen
 */

#ifndef __CC3200R1M1RGC__
// Do not include SPI for CC3200 LaunchPad
#include <SPI.h>
#endif
#include <WiFi.h>

char ssid[] = "energia1";           // your network name also called SSID
char password[] = "launchpad";      // your network password
int keyIndex = 0;                   // your network key Index number (needed only for WEP)

// Motor control pins
int TURN_EN  = P6_1;     // Enable the motors. PWM to control speed
int TURN_DIR = P6_2;     // 0 or 1 for left or right.
int WALK_EN  = P6_3;     // Enable walking motor
int WALK_DIR = P6_4;     // 0 or 1 for forward or back.

// LEDs
int LED_LEFT1 = P4_1;    // +V for LED
int LED_LEFT2 = P3_5;    // GND for LED

int LED_RIGHT1 = P8_1;   // +V for LED
int LED_RIGHT2 = P8_2;   // GND for LED

int ANALOG_IN = P6_0;    // Analog Input

String response;
int    executed = 0;

WiFiClient client;              // Initialize the WiFi client.
char server[] = "crea.arduinogt.com";   // server address:
//IPAddress server(50,62,217,1);

unsigned long lastConnectionTime = 0;                // last time you connected to the server, in milliseconds
const unsigned long postingInterval = 1L * 1000L;    // delay between updates, in milliseconds


void setup() {
  int ctr=0;
  
  Serial.begin(115200);      // initialize serial communication
  
  pinMode(TURN_EN,  OUTPUT);
  pinMode(TURN_DIR, OUTPUT);
  pinMode(WALK_EN,  OUTPUT);
  pinMode(WALK_DIR, OUTPUT);
  
  digitalWrite(TURN_EN,  LOW);
  digitalWrite(TURN_DIR, LOW);
  digitalWrite(WALK_EN,  LOW);
  digitalWrite(WALK_DIR, LOW);
  
  pinMode(LED_LEFT1, OUTPUT);
  pinMode(LED_LEFT2, OUTPUT);
  digitalWrite( LED_LEFT2, 0 );
  analogWrite( LED_LEFT1, 128 );

  pinMode(LED_RIGHT1, OUTPUT);
  pinMode(LED_RIGHT2, OUTPUT);
  digitalWrite( LED_RIGHT2, 0 );
  analogWrite( LED_RIGHT1, 128 );
  
  pinMode( ANALOG_IN, INPUT );
  
  for(int x=0;x<10;x++) {
    analogWrite( LED_LEFT1, 128*(x%2) );
    analogWrite( LED_RIGHT1, 128*(x%2) );
    delay(100);
  }  
  analogWrite( LED_RIGHT1, 0 );
  
  Serial.print("Attempting to connect to Network named: ");  // attempt to connect to Wifi network:
  Serial.println(ssid);                                      // print the network name (SSID);
  WiFi.begin(ssid, password);                                // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
  Serial.print("WiFi has begun");
  while ( WiFi.status() != WL_CONNECTED) {
    Serial.print(".");                        // Print dots while we wait to connect
    analogWrite( LED_LEFT1, 128*(ctr++%2) );  // Blink the left LED
    delay(125);
  }
  analogWrite( LED_LEFT1, 128 );              // Turn on the left LED when we are connected.
  
  Serial.println("\nYou're connected to the network");
  Serial.println("Waiting for an ip address");
  
  while (WiFi.localIP() == INADDR_NONE) {
    Serial.print(".");                        // Print dots while we wait for an ip addresss
    analogWrite( LED_RIGHT1, 128*(ctr++%2) );  // Blink the left LED
    delay(125);
  }
  analogWrite( LED_RIGHT1, 128 );              // Turn on the left LED when we have an IP.

  Serial.println("\nIP Address obtained");
  analogWrite( LED_RIGHT1, 128 );            // Turn on the right LED to indicate we have an IP.
  
  // you're connected now, so print out the status  
  printWifiStatus();

}

void loop() {
  int i = 0;
  int sensor = 0;

  // Used for Ultrasonic  
  int   duration;
  float distance;
  int   conversion;
  
  sensor = analogRead( ANALOG_IN );

  // if there's incoming data from the net connection.
  // send it out the serial port.  This is for debugging
  // purposes only:
  while (client.available()) {
    char c = client.read();
    Serial.write(c);
    
    response += c;
    int st = response.indexOf('<');
    int ed = response.indexOf('>');
    if (st > 0 and ed > 0){
      response = response.substring(st+1,ed);
      executed = true;
      break;
    }
  }

  if (executed) {

    executed=0;
    Serial.println();
    Serial.print("response: ");
    Serial.println( response );
    Serial.println();

    // Forward or Back
    if (response.indexOf("FORWARD")>0) {
      digitalWrite(WALK_EN, HIGH);   // Enable the motor
      digitalWrite(WALK_DIR,HIGH);   // Go Forward
      digitalWrite(TURN_EN, LOW);    // Stop Turning
      digitalWrite(TURN_DIR, LOW);   // Stop Turning
    }
    if (response.indexOf("BACK")>0) {
      digitalWrite(WALK_EN,  HIGH);  // Enable the motor
      digitalWrite(WALK_DIR, LOW);   // Go Back
      digitalWrite(TURN_EN, LOW);    // Stop Turning
      digitalWrite(TURN_DIR, LOW);   // Stop Turning
    }
        
    // Left or Right
    if (response.indexOf("LEFT")>0) {
      digitalWrite(TURN_EN,  HIGH);    // Enable the motor
      digitalWrite(TURN_DIR, HIGH);    // Turn Left
    }
    if (response.indexOf("RIGHT")>0) {
      digitalWrite(TURN_EN,  HIGH);    // Enable the motor
      digitalWrite(TURN_DIR, LOW);     // Turn Right
    }

    // STOP
    if (response.indexOf("STOP")>0) {
      digitalWrite(WALK_EN, LOW);    // Disable the motor
      digitalWrite(TURN_EN, LOW);    // Disable the motor
      digitalWrite(WALK_DIR, LOW);   // Disable the motor
      digitalWrite(TURN_DIR, LOW);   // Disable the motor
    }   
    
  }  
  
  // if ten seconds have passed since your last connection,
  // then connect again and send data:
  if (millis() - lastConnectionTime > postingInterval) {
    httpRequest(sensor);
  }
  
}

// this method makes a HTTP connection to the server:
void httpRequest(int sensor) {
  // close any connection before send a new request.
  // This will free the socket on the WiFi shield
  client.stop();

  // if there's a successful connection:
  if (client.connect(server, 80)) {
    Serial.println("connecting...");
    
    // send the HTTP PUT request:
    client.print("GET /v1/aw/a1c8cea2b4bc73fbe71aa33b4d6feaf4");
//    client.print(sensor);
    client.println(" HTTP/1.1");
    
    client.print("Host: ");
    client.println(server);
    
    client.println("Authorization: Basic MTc2MjIyYTI0OTYyNDJmODU4YzEwNmFiZmQ4YjUyNmE6MjA2ZjdlMjdjYWNmY2RlM2Y4MzQ3MjE2ZjVlMjZhMDM=");
    
    client.println("User-Agent: Energia/1.1");
    client.println("Connection: close");
    client.println();

    // note the time that the connection was made:
    lastConnectionTime = millis();
  }
  else {
    // if you couldn't make a connection:
    Serial.println("connection failed");
  }
}

void flashLEDs() {
  
  for(int x=0;x<10;x++) {
    analogWrite( LED_LEFT1, 128*(x%2) );
    analogWrite( LED_RIGHT1, 128*(x%2) );
    delay(100);
  }  
  analogWrite( LED_LEFT1,  128 );
  analogWrite( LED_RIGHT1, 128 );
  
}

//
//a way to check if one array ends with another array
//
boolean endsWith(char* inString, char* compString) {
  int compLength = strlen(compString);
  int strLength = strlen(inString);
  
  //compare the last "compLength" values of the inString
  int i;
  for (i = 0; i < compLength; i++) {
    char a = inString[(strLength - 1) - i];
    char b = compString[(compLength - 1) - i];
    if (a != b) {
      return false;
    }
  }
  return true;
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
