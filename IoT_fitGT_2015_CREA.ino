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

WiFiServer server(80);

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
  
  // you're connected now, so print out the status  
  printWifiStatus();

  Serial.println("Starting webserver on port 80");
  server.begin();                            // Start the web server on port 80
  analogWrite( LED_RIGHT1, 128 );            // Turn on the right LED to indicate the webserver is started.
  Serial.println("Webserver started!");
}

void loop() {
  int i = 0;
  int sensor = 0;
  WiFiClient client = server.available();   // listen for incoming clients
  
  sensor = analogRead( ANALOG_IN );
  
  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    char buffer[150] = {0};                 // make a buffer to hold incoming data
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (strlen(buffer) == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.println("<html><head><title>Energia CC3200 WiFi Web Server</title></head><body align=center>");
            client.println("<h1 align=center><font color=\"red\">Welcome to the CC3200 WiFi Web Server</font></h1>");
            
            client.println(" <button onclick=\"location.href='/F'\">Forward</button><br>");
            client.println(" <button onclick=\"location.href='/L'\">Left</button>");
            client.println("&nbsp; <button onclick=\"location.href='/S'\">STOP</button> &nbsp;");
            client.println(" <button onclick=\"location.href='/R'\">Right</button><br>");
            client.println(" <button onclick=\"location.href='/B'\">Back</button><br>");

            client.println(" <br><br>Analog: ");
            client.println(sensor);
            client.println(" <br>");

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          }
          else {      // if you got a newline, then clear the buffer:
            memset(buffer, 0, 150);
            i = 0;
          }
        }
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          buffer[i++] = c;      // add it to the end of the currentLine
        }

        // Forward or Back
        if (endsWith(buffer, "GET /F")) {
          digitalWrite(WALK_EN, HIGH);   // Enable the motor
          digitalWrite(WALK_DIR,HIGH);   // Go Forward
          digitalWrite(TURN_EN, LOW);    // Stop Turning
          digitalWrite(TURN_DIR, LOW);   // Stop Turning
        }
        if (endsWith(buffer, "GET /B")) {
          digitalWrite(WALK_EN,  HIGH);  // Enable the motor
          digitalWrite(WALK_DIR, LOW);   // Go Back
          digitalWrite(TURN_EN, LOW);    // Stop Turning
          digitalWrite(TURN_DIR, LOW);   // Stop Turning
        }
        
        // Left or Right
        if (endsWith(buffer, "GET /L")) {
          digitalWrite(TURN_EN,  HIGH);    // Enable the motor
          digitalWrite(TURN_DIR, HIGH);    // Turn Left
        }
        if (endsWith(buffer, "GET /R")) {
          digitalWrite(TURN_EN,  HIGH);    // Enable the motor
          digitalWrite(TURN_DIR, LOW);     // Turn Right
        }

        // STOP
        if (endsWith(buffer, "GET /S")) {
          digitalWrite(WALK_EN, LOW);    // Disable the motor
          digitalWrite(TURN_EN, LOW);    // Disable the motor
          digitalWrite(WALK_DIR, LOW);   // Disable the motor
          digitalWrite(TURN_DIR, LOW);   // Disable the motor
        }   

        // TEST
        if (endsWith(buffer, "GET /T")) {
          digitalWrite(WALK_EN, HIGH);    // Set all pins high
          digitalWrite(WALK_DIR,HIGH);    // Set all pins high
          digitalWrite(TURN_EN, HIGH);    // Set all pins high
          digitalWrite(TURN_DIR,HIGH);    // Set all pins high
        }   
      }
    }
    
    // close the connection:
    client.stop();
    Serial.println("client disonnected");
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
  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}
