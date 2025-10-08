#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <Servo.h>

// Pin Definitions 
#define RELAY 3  // Relay for pump control
#define SERVO_PIN 4  // Linear actuator (servo)
#define IN4 5
#define IN3 6
#define IN2 7
#define IN1 8
#define GROUND_SERVO_PIN 9
#define DHTPIN 10
#define DHTTYPE DHT11
#define TRIG_PIN 11
#define ECHO_PIN 12
#define SCAN_SERVO_PIN 13
#define LEFT_IR A0
#define RIGHT_IR A1
#define MOISTURE_BACK_SENSOR A2 //ground soil moisture sensor 
#define MOISTURE_SENSOR A3  // Analog input

DHT dht(DHTPIN, DHTTYPE);
Servo soilServo;
Servo secServo;
Servo ultServo;
LiquidCrystal_I2C lcd(0x27, 16, 2);

char command;
int mode = 1; // Default mode is Manual Mode

void setup() {
    Serial.begin(9600);
    
    // Sensor Initialization
    dht.begin();
    
    // LCD Initialization
    lcd.init();
    lcd.backlight();
    
    // Pin Configurations
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(RELAY, OUTPUT);
    pinMode(MOISTURE_SENSOR, INPUT);
    pinMode(MOISTURE_BACK_SENSOR, INPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    
    // Servo Initialization
    soilServo.attach(SERVO_PIN); //linear actuator servo
    soilServo.write(0);  // Set to initial position

    // Servo Initialization
    ultServo.attach(SCAN_SERVO_PIN); //ultrasonic scan servo
    ultServo.write(90);  // Set to initial position

    // Servo Initialization
    secServo.attach(GROUND_SERVO_PIN); //ground moisture servo
    secServo.write(0);  // Set to initial position
    
    // Ensure pump is OFF at startup
    digitalWrite(RELAY, HIGH);
}

void loop() {
    if (Serial.available() > 0) {
        command = Serial.read();
        Serial.println(command);
        handleMode(command); // Switch between modes
    }

    // Mode Handling
    if (mode == 1) {        // Manual Pot Mode
        manualMode();
    } 
    else if(mode == 2) {
        groundSoilMoisture(); // Manual Ground Mode
    }
    else (mode == 3) {
        autoSoilMoisture(); // Auto Ground Mode
    }
}

void handleMode(char cmd) {
    if (cmd == 'M') mode = 1;  // Switch to Manual Mode
    if (cmd == 'G') mode = 2;  // Switch to ground testing Mode
    if (cmd == 'A') mode = 2;  // Switch to auto ground testing Mode
}

void manualMode (char cmd) {
    switch (cmd) {
        case 'F': moveForward(); break;
        case 'B': moveBackward(); break;
        case 'L': turnLeft(); break;
        case 'R': turnRight(); break;
        case 'X': testSoil(); break;  // Test soil moisture and control pump
        case 'S': stopMotors(); break;
    }
}

void autoSoilMoisture() {
  
        moveForward(); // Move freely
        
        long duration;
        long distance;
      
          // Ultrasonic Sensor Trigger
          digitalWrite(TRIG_PIN, LOW);
          delayMicroseconds(2);
          digitalWrite(TRIG_PIN, HIGH);
          delayMicroseconds(10);
          digitalWrite(TRIG_PIN, LOW);
          
          duration = pulseIn(ECHO_PIN, HIGH);
          distance = duration * 0.034 / 2;  // Convert to cm
          
          if (distance < 5) {  // Object detected
              stopMotors();
              delay(200);
        
        ultServo.write(150);  // Look Right
        delay(500);
        long rightDistance = getDistance();

        ultServo.write(90);  //come to initial position
        delay(500);
    
        ultServo.write(30);  // Look Left
        delay(500);
        long leftDistance = getDistance();
        
        ultServo.write(90);  //come to initial position
        
        // Corrected movement logic
        if (rightDistance > leftDistance) {
            turnRight(); 
        } 
        else {
            turnLeft();
        }
        delay(1000);
        
        moveForward();
       }
        int leftIR = digitalRead(LEFT_IR);
        int rightIR = digitalRead(RIGHT_IR);
        
        if (rightIR == 1){
            stopMotors();
            delay(8000);
            testGroundSoil();
        }
        else if (leftIR == 1) {
            turnRight();
            delay(1000);
            stopMotors();
            delay(8000);
            testGroundSoil();
            testGroundSoil();
            turnLeft();
            delay(1000);
        } 
        else {
        moveForward(); // Move freely
        }
      }

void groundSoilMoisture (char cmd) {
    switch (cmd) {
        case 'F': moveForward(); break;
        case 'B': moveBackward(); break;
        case 'L': turnLeft(); break;
        case 'R': turnRight(); break;
        case 'X': testGroundSoil(); break;  // Test soil moisture and control pump
        case 'S': stopMotors(); break;
    }
}

// Motor Functions
void moveForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void moveBackward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void turnLeft() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void turnRight() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

// Soil Moisture Test (Controls Pump & Servo)

void testSoil() {
    // Ensure pump is OFF before testing
    digitalWrite(RELAY, HIGH);
    digitalWrite(RELAY, HIGH); // Ensure pump is OFF before testing
    digitalWrite(RELAY, HIGH); // Ensure pump starts in OFF state
    soilServo.write(90);
    delay(7000);
    int moisture = digitalRead(MOISTURE_SENSOR);
    Serial.print("Moisture Level: ");
    Serial.println(moisture);
    
    if (moisture == 1) {
        digitalWrite(RELAY, LOW);
        lcd.setCursor(0, 1);
        lcd.print("Pump: ON ");
        delay(5000); // Keep pump on for 1 second
        digitalWrite(RELAY, HIGH); // Turn pump off after 1 second
    } else {
        digitalWrite(RELAY, HIGH);
        lcd.setCursor(0, 1);
        lcd.print("Pump: OFF");
    }
    
    soilServo.write(0);
}

void testGroundSoil() {
    // Ensure pump is OFF before testing
    digitalWrite(RELAY, HIGH);
    digitalWrite(RELAY, HIGH); // Ensure pump is OFF before testing
    digitalWrite(RELAY, HIGH); // Ensure pump starts in OFF state
    secServo.write(90);
    delay(7000);
    int moisture = digitalRead(MOISTURE_BACK_SENSOR);
    if (moisture == 1) {
        digitalWrite(RELAY, LOW);
        lcd.setCursor(0, 1);
        lcd.print("Pump: ON ");
        delay(5000); // Keep pump on for 1 second
        digitalWrite(RELAY, HIGH); // Turn pump off after 1 second
    } else {
        digitalWrite(RELAY, HIGH);
        lcd.setCursor(0, 1);
        lcd.print("Pump: OFF");
    }
    
    secServo.write(0);
}
// Weather Update (Temperature & Humidity)
void updateWeather() {
    Serial.print("Temperature: ");
    Serial.print(dht.readTemperature());
    Serial.print(" C, Humidity: ");
    Serial.print(dht.readHumidity());
    Serial.println(" %");
    float temp = dht.readTemperature();
    float hum = dht.readHumidity();
    lcd.setCursor(0, 0);
    lcd.print("T:");
    lcd.print(temp);
    lcd.print("C H:");
    lcd.print(hum);
    lcd.print("%");
}