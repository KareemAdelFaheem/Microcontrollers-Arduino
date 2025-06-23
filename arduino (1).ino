#include <Wire.h>
#include <SoftwareSerial.h>
SoftwareSerial SLAVE_SERVO_SERIAL(2, 3); //rx,tx

const byte I2C_ADDRESS = 8;

#define IN1 6
#define IN2 7
#define IN3 8
#define IN4 9
#define ENA 5
#define ENB 10

const int soil_moist_pin1 = A0;
const int soil_moist_pin2 = A1;
const int soil_moist_pin3 = A2;

#define water_motor_pin 4

#define tank_threshold 25




int rain_sens = 12;

int tank_flag = 0;

int slider_flag = 0;

#define TRIGGER_PIN  3
#define ECHO_PIN  11

long duration;
int distance;


int ceilingStatus = 0;
int dry = 0;

volatile int receivedPrediction = 0;
int rainToday = 0;

float soil_moist(int pin) {
  return analogRead(pin);
}

// Called when ESP requests data
void send_sensors_data() {

  
  int soil1 = analogRead(soil_moist_pin1);
  int soil2 = analogRead(soil_moist_pin2);
  int soil3 = analogRead(soil_moist_pin3);
  Wire.write(rainToday>> 8); Wire.write(rainToday & 0xFF);
  Wire.write(soil1 >> 8); Wire.write(soil1 & 0xFF);
  Wire.write(soil2 >> 8); Wire.write(soil2 & 0xFF);
  Wire.write(soil3 >> 8); Wire.write(soil3 & 0xFF);
  Wire.write(distance >> 8); Wire.write(distance & 0xFF);
  Wire.write(ceilingStatus);
  Wire.write(tank_flag);
  Wire.write(slider_flag);
  
}

void openCeiling() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, 200);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW); analogWrite(ENB, 0);
  delay(3000);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0);
  ceilingStatus = 1;
}

void closeCeiling() {
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, 200);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW); analogWrite(ENA, 0);
  delay(3000);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENB, 0);
  ceilingStatus = 0;
}

// Called when ESP sends data
void receive_data(int data_size) {
  if (data_size >= 2) {
    int highByte = Wire.read(); // Prediction high byte
    int lowByte = Wire.read();  // Prediction low byte
    receivedPrediction = (highByte << 8) | lowByte;
  }
  while (Wire.available()) Wire.read(); // Flush any extra
}

int tank_Check(){

digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  // Read the echo pin and calculate distance
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration * 0.0343) / 2; // speed of sound = 343 m/s

  

  return distance;

  delay(500); 


} 


//////////////////////////////////////////////////////////////////////////////////////////// trace conditions
void irregate() {


  openCeiling();

  dry = 0; // Reset dry flag
  const int DRY_THRESHOLD = 600;  // 0–1023: lower value means wetter soil. Threshold 200 = very dry.

  


  float moist[3] = {soil_moist(A0), soil_moist(A1), soil_moist(A2)};
  for (int i = 0; i < 3; i++) {
    if (moist[i] > 600 ) {
      Serial.println("Soil is dry!");
      dry = 1;
      break;
    }
    else if(moist[i] < 600){

      Serial.println("Soil is Wet!");
        dry = 0;
}

  }

  if (receivedPrediction == 1 && dry == 1) {
    digitalWrite(water_motor_pin, HIGH);
    Serial.print(" Watre pumb OFF \n");

    openCeiling();
    Serial.print(" ceiling is Open \n");
  } if (receivedPrediction == 0 && dry == 1) {
    digitalWrite(water_motor_pin, LOW);
    Serial.print(" Watre pumb ON \n");
    openCeiling();
    Serial.print(" cieling is Open  \n");
  } if (receivedPrediction == 1 && dry == 0) {
    closeCeiling();
    Serial.print(" cieling is closed  \n");
    digitalWrite(water_motor_pin, HIGH);
    Serial.print(" Watre motor OFF  \n");
  }

  
  
}

void setup() {
  Serial.begin(115200);

  SLAVE_SERVO_SERIAL.begin(9600); 

  pinMode(soil_moist_pin1, INPUT);
  pinMode(soil_moist_pin2, INPUT);
  pinMode(soil_moist_pin3, INPUT);
  pinMode(rain_sens, INPUT);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(water_motor_pin, OUTPUT);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);


  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receive_data);
  Wire.onRequest(send_sensors_data);
}

void loop() {
  Serial.print("Prediction: ");
  Serial.print(receivedPrediction);


  



  int rain_value = digitalRead(rain_sens);


  
  Serial.print(" | Rain value: ");
  Serial.println(rain_value);

  if(rain_value == 0){

      rainToday = 1;

      closeCeiling();
      Serial.print(" cieling is Closed because of RAIN sensor  \n");

  }
Serial.print(" | Rain today: ");
  Serial.println(rainToday);



  irregate();


  int tank_lvl = tank_Check();

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if(tank_lvl > tank_threshold){
if (receivedPrediction == 1 || rain_value == 0) {
      // Rain expected or detected, and tank is low → command Arduino #2
      SLAVE_SERVO_SERIAL.println("OPEN_SLIDER");
    } else if (receivedPrediction == 0 ) {
      // No rain expected and tank low → send alert to Firebase replace with firebase alert ya kimo
      Serial.println("[ALERT] Tank is low and no rain is predicted");
      //fire base flag of tank low lvl 
      tank_flag = 1;
      

    }
  } else {
    Serial.println("Tank level good, no servo action");
    tank_flag = 0;
  }

 if (SLAVE_SERVO_SERIAL.available()) {
    String response = SLAVE_SERVO_SERIAL.readStringUntil('\n');
    if (response == "SLIDER_DONE") {
      Serial.println("[INFO] Slider operation completed");
      //slider is open notfiy firebase for the app 
        slider_flag = 1;
    
    }
    else{ slider_flag = 0 ; }
  }

delay(3000);
}
