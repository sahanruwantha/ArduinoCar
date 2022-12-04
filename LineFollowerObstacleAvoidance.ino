#include<NewPing.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>        

LiquidCrystal_I2C lcd(0x27, 16, 2);

int ENA = 9;  
int ENB = 10;  
int MOTOR_A1 = 7; 
int MOTOR_A2 = 6; 
int MOTOR_B1 = 5; 
int MOTOR_B2 = 4; 
long duration; 
int distance; 
int RIGHT = A0; 
int LEFT = A1;
float adc_voltage = 0.0;
float in_voltage = 0.0; 
float R1 = 30000.0;
float R2 = 7500.0; 
float ref_voltage = 5.0;
int adc_value = 0;

int encoder_pin = A2;
unsigned int rpm;
volatile byte pulses; 
unsigned long timeold;
unsigned int pulsesperturn = 12;

int speed;

#define trigPin 3 
#define echoPin 2 
#define MAX_DISTANCE 100
#define ANALOG_IN_PIN A3 

NewPing sonar(trigPin, echoPin, MAX_DISTANCE); 

void setup() {
  lcd.init();         
  lcd.backlight();    

  Serial.begin(9600);
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 
  pinMode(ENA, OUTPUT); 
  pinMode(ENB, OUTPUT); 
  pinMode(MOTOR_A1, OUTPUT); 
  pinMode(MOTOR_A2, OUTPUT); 
  pinMode(MOTOR_B1, OUTPUT); 
  pinMode(MOTOR_B2, OUTPUT); 
  pinMode(RIGHT, INPUT); 
  pinMode(LEFT, INPUT);  

  pinMode(encoder_pin, INPUT);
  attachInterrupt(0, counter, FALLING);
  pulses = 0;
  rpm = 0;
  timeold = 0;

}

void loop() {
  getBatteryLife();
  getSpeed();
  if (digitalRead(RIGHT)==0 && digitalRead(LEFT)==0) {
    moveForward();
    
  }else if (digitalRead(RIGHT)==0 && !digitalRead(LEFT)==0) {
   turnRight();

  }else if (!digitalRead(RIGHT)==0 && digitalRead(LEFT)==0) {
    turnLeft(); 
    
  }else if (!digitalRead(RIGHT)==0 && !digitalRead(LEFT)==0) {
    Stop();
  }
}
void Stop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_B2, LOW);
  speed = 0;
  printData();
}

void turnRight() {
  analogWrite(ENA,200);
  analogWrite(ENB, 200);
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_B2, LOW);
  speed = 12;
  printData();
}

void turnLeft() {
  analogWrite(ENA,200);
  analogWrite(ENB, 200);
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, LOW);
  speed = 12;
  printData();
}

void moveForward() {
  analogWrite(ENA, 80);
  analogWrite(ENB, 80);
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_B2, LOW);
  speed = 8;
  printData();
}

int getDistance(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

void printData(){
  lcd.clear();                      
  lcd.setCursor (2, 0);             
  lcd.print("Battery  ");
  lcd.print(getBatteryLife());
  lcd.print("%");
  lcd.setCursor (2, 1);
  lcd.print("Speed  ");
  lcd.print(speed);
  lcd.print(" mph");         
  delay(100);
}

int getBatteryLife(){
  adc_value = analogRead(ANALOG_IN_PIN);
  adc_voltage  = (adc_value * ref_voltage) / 1024.0; 
  in_voltage =  int(((adc_voltage / (R2/(R1+R2)))/12)*100); 
  return in_voltage;
}

void counter()
{
   pulses++;
}

void getSpeed(){
  if (millis() - timeold >= 1000) {
      detachInterrupt(0);
      rpm = (60 * 1000 / pulsesperturn )/ (millis() - timeold)* pulses;
      timeold = millis();
      pulses = 0;
      Serial.print("RPM = ");
      Serial.println(rpm,DEC);
      attachInterrupt(0, counter, FALLING);
   }
}
