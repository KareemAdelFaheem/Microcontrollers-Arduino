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

#define tank_threshold 12 /// modify 7sb 2l ra8ba




int rain_sens = 12;

int tank_flag = 0;

int waterPump_flag = 0;

int ceilingEnable = 0;

int pumpEnable = 0; 





#define TRIGGER_PIN  13
#define ECHO_PIN  11

long duration;
int distance;


int ceilingStatus = 0;
int dry = 0;
int ceilingActionTaken = 0;


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
  Wire.write(waterPump_flag);
  Wire.write(ceilingStatus);
  Wire.write(tank_flag);
  
}

void openCeiling(){
  analogWrite(ENA, 255);  // Full speed to open-side motors
  analogWrite(ENB, 255);  // Full speed to close-side motors

  // A+B: Push (reverse)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  // C+D: Pull (forward)
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  delay(2000); // Run for 5 sec

  stopCeiling();  // Stop all
  ceilingStatus = 1; //  OPEN = 1
}

void closeCeiling() {
  analogWrite(ENA, 255);  // Full speed to open-side motors
  analogWrite(ENB, 255);  // Full speed to close-side motors

  // A+B: Pull (e.g., forward)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // C+D: Push (reverse)
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  delay(2000); // Run for 5 sec

  stopCeiling();  // Stop all
  ceilingStatus = 0; //  CLOSED = 0
}




void stopCeiling() {
  // Stop motor A+B
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(ENA, LOW);

  // Stop motor C+D
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(ENB, LOW);
}



// Called when ESP sends data
void receive_data(int data_size) {
  if (data_size >= 4) {
  int highByte = Wire.read();
  int lowByte = Wire.read();
  receivedPrediction = (highByte << 8) | lowByte;
  ceilingEnable = Wire.read();
  pumpEnable = Wire.read();
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

void checkSoilDryStatus() {
  dry = 0;
  const int DRY_THRESHOLD = 600;
  float moist[3] = {
    soil_moist(A0),
    soil_moist(A1),
    soil_moist(A2)
  };

  for (int i = 0; i < 3; i++) {
    if (moist[i] > DRY_THRESHOLD) {
      dry = 1;
      break;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////// trace conditions
void irregate() {

if (pumpEnable == 1) {
  // proceed to control water pump
  do {
    digitalWrite(water_motor_pin, LOW);
    Serial.print("Water pump ON\n");
    waterPump_flag = 1;

    if (pumpEnable == 0 || dry == 0) {
      digitalWrite(water_motor_pin, HIGH);
      Serial.print("Water motor OFF\n");
      waterPump_flag = 0;
      break;
    }

  } while (dry == 1);
}


if (ceilingEnable == 1) {
  // proceed to open/close ceiling
  if (ceilingActionTaken == 0 && ceilingStatus != 1) {
      openCeiling();
      Serial.print("Ceiling is opened by the User Manually\n");
      ceilingActionTaken = 1;
    }
}
else if(ceilingEnable == 0){
  if (ceilingActionTaken == 0 && ceilingStatus != 0) {
      closeCeiling();
      Serial.print("Ceiling is Closed by the User Manually\n");
      ceilingActionTaken = 1;
    }

}


  if (receivedPrediction == 1 && dry == 1) {
    digitalWrite(water_motor_pin, HIGH);
    Serial.print("Water pump OFF\n");

    if (ceilingActionTaken == 0 && ceilingStatus != 1) {
      openCeiling();
      Serial.print("Ceiling is Open\n");
      ceilingActionTaken = 1;
    }
  }
  else if (receivedPrediction == 0 && dry == 1) {
    digitalWrite(water_motor_pin, LOW);
    Serial.print("Water pump ON\n");
    waterPump_flag = 1;

    if (ceilingActionTaken == 0 && ceilingStatus != 1) {
      openCeiling();
      Serial.print("Ceiling is Open\n");
      ceilingActionTaken = 1;
    }
  }
  else if (receivedPrediction == 1 && dry == 0) {
    if (ceilingActionTaken == 0 && ceilingStatus != 0) {
      closeCeiling();
      Serial.print("Ceiling is Closed\n");
      ceilingActionTaken = 1;
    }

    digitalWrite(water_motor_pin, HIGH);
    Serial.print("Water motor OFF\n");
    waterPump_flag = 0;
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

  ceilingActionTaken = 0; 


  checkSoilDryStatus();

  Serial.print("Prediction: ");
  Serial.print(receivedPrediction);



  int rain_value = digitalRead(rain_sens);


  
  Serial.print(" | Rain value: ");
  Serial.println(rain_value);


  if(rain_value == 0){

    digitalWrite(water_motor_pin, HIGH);
    Serial.print("Water motor OFF\n");
    waterPump_flag = 0;

  }

  

  if (rain_value == 0 && dry == 0) {
  rainToday = 1;
  if (ceilingStatus != 0) {
    closeCeiling();
    Serial.print("Ceiling is Closed because of RAIN sensor\n");
    ceilingActionTaken = 1;
  }
}
else if (rain_value == 0 && dry == 1) {
  rainToday = 1;

  if (ceilingStatus != 1) {
    openCeiling();
    Serial.print("Ceiling is Opened to water plants\n");
    ceilingActionTaken = 1;
  }
}
else if (rain_value == 1) {
  rainToday = 0;
}

  
  
Serial.print(" | Rain today: ");
  Serial.println(rainToday);


if(rainToday != 1){

  irregate();

}

Serial.print("Current ceiling status: ");
Serial.println(ceilingStatus == 1 ? "OPEN" : "CLOSED"); /////tracing 



  int tank_lvl = tank_Check();

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if(tank_lvl > tank_threshold){
if (receivedPrediction == 1 || rain_value == 0) {
      // Rain expected or detected, and tank is low → command Arduino #2
      SLAVE_SERVO_SERIAL.println("OPEN_SLIDER");
      SLAVE_SERVO_SERIAL.println(" ");
      
    } else if (receivedPrediction == 0 ) {
      // No rain expected and tank low → send alert to Firebase replace with firebase alert ya kimo
      Serial.println("[ALERT] Tank is low and no rain is predicted");
      //fire base flag of tank low lvl 
      tank_flag = 1;
      
    }
  } else {
    Serial.println("Tank level good, no servo action");
    SLAVE_SERVO_SERIAL.println("CLOSE_SLIDER");
    SLAVE_SERVO_SERIAL.println(" ");
    
    Serial.println("Sent CLOSE_SLIDER command.");
    tank_flag = 0;
    
  }

  

delay(3000);
}