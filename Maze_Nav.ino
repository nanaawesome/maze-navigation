#include <MPU6050_tockn.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <stdlib.h> 
#include <map>
#include <Wire.h>
#include <Math.h>

#define I2C_SLAVE_ADDR 0x04
//#include <MPU6050_light.h>
MPU6050 mpu(Wire);
int16_t leftMotor_speed, rightMotor_speed, servoAngle,enc_distance;
std::map<int, String> directions;

const byte ROWS = 4; //four rows
const byte COLS = 3; //three columns
char keys[ROWS][COLS] = {
 {'1','2','3'},
 {'4','5','6'},
 {'7','8','9'},
 {'*','0','#'}
};

byte rowPins[ROWS] = {13, 33, 32, 15}; 
byte colPins[COLS] = {4, 16, 17}; 
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

 
// Create the lcd object address 0x3F (Steven) 0x27(Me) and 16 columns x 2 rows 
LiquidCrystal_I2C lcd (0x3F, 16,2);  //
 
// Set up the LCD and keypad
void setup() {
  lcd.begin();
  lcd.backlight();
  Serial.begin(9600);
  directions[2]='F';
  directions[8]='B';
  directions[4]='L';
  directions[6]='R';
  Wire.begin();   
  mpu.begin();
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcGyroOffsets(true);
  Serial.println("Done!\n");
}



// Print the current menu options to the LCD display
void menu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Enter number of");
  lcd.setCursor(0, 1);
  lcd.print("commands:");
  String numCommands = "";
  char key;
  do{
    key = keypad.waitForKey();
    if (key != NO_KEY) {
      lcd.print(key);
      if (numCommands.toInt()>20 && key=='#'){
        numCommands="";
      }
      else{
      numCommands += key;
      }
    }
  }while (key!='#' || (numCommands.toInt()>20) || (numCommands.toInt()==0));
  int n = numCommands.toInt();
  lcd.clear();
  lcd.print(n);
  delay(1000);
  
  //Creates arrays depending on number of commands
  String commands[n];
  int quantities[n];
  for (int i = 0; i < n; i++) {
    lcd.setCursor(0, 0);
    lcd.print("Enter direction ");
    lcd.print(i + 1);
    lcd.setCursor(0, 1);
    String command = "";
    while (command == "") {
      char key = keypad.waitForKey();
      if ((key != NO_KEY) && (key=='2' || key=='8' || key=='4' || key=='6')) {
        lcd.clear();
        command += key;
        lcd.print(directions[command.toInt()]);
        delay(1000);
      }
      else{
        lcd.clear();
        lcd.print("Invalid command");
      }
    }
    commands[i] = directions[command.toInt()];
    lcd.clear();
    
    lcd.print("Enter quantity ");
    lcd.print(i + 1);
    lcd.setCursor(0, 1);
    String quantity = "";
    char key;
    do{
      key = keypad.waitForKey();
      if (((key != NO_KEY) && (commands[i]=="L" || commands[i]=="R")) && (key!='1' && key!='2')) {
            lcd.clear();
            lcd.print("Invalid quantity");
            lcd.setCursor(0, 1);
            lcd.print("1->90, 2->180");
      }
      else if(key!=NO_KEY){
          lcd.clear();
          quantity += key;
          if(quantity.toInt()==1 && (commands[i]=="L" || commands[i]=="R")){
            lcd.print(90);
          }
          else if(quantity.toInt()==2 && (commands[i]=="L" || commands[i]=="R")){
            lcd.print(180);
          }
          else{
            lcd.print(quantity.toInt());
          }
       } 
    }while(key!='#');
  
    quantities[i] = quantity.toInt();
    lcd.clear();
  }
  lcd.print("[");
  Serial.print("[");
  for(int i=0;i<n;i++){
    lcd.print(commands[i]);
    Serial.print(commands[i]);
    lcd.print(quantities[i]);
    Serial.print(quantities[i]);
    if(i<n-1){
      lcd.print(",");
      Serial.print(",");
    }
  }
  lcd.print("]");
  Serial.print("]");
  lcd.setCursor(0, 1);
  lcd.print("# to Confirm");
  while(keypad.waitForKey()!='#'){
    lcd.clear();
    lcd.print("# to Confirm");
  }
  for(int i=0;i<n;i++){
    if (commands[i] == "F") {
      forward(quantities[i]);
    } else if (commands[i] == "B") {
        backward(quantities[i]);
    } else if (commands[i] == "L") {
        left(quantities[i]);
    } else if (commands[i] == "R") {
        right(quantities[i]);
    }
  }
}
  
void loop() {
  menu();
}


void transmit_to_arduino(int16_t leftMotor_speed,int16_t rightMotor_speed, int16_t servoAngle){
  Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
    Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
    Wire.write((byte)(leftMotor_speed & 0x000000FF));           // second byte of x, containing the 8 LSB - bits 8 to 1
    Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));    // first byte of y, containing bits 16 to 9
    Wire.write((byte)(rightMotor_speed & 0x000000FF));           // second byte of y, containing the 8 LSB - bits 8 to 1
    Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));    // first byte of y, containing bits 16 to 9
    Wire.write((byte)(servoAngle & 0x000000FF));           // second byte of y, containing the 8 LSB - bits 8 to 1
  Wire.endTransmission();   // stop transmitting 
  
}

float get_distance(void){
  int16_t encoder1=0;
  int16_t encoder2=0;
  Wire.requestFrom(I2C_SLAVE_ADDR,4); // request two bytes of data from the slave device (1 for each number)
  
  while (Wire.available()) { // check if the data is available
    uint8_t enc1_16_9 = Wire.read();  // receive bits 16 to 9 of x (one byte)
    uint8_t enc1_8_1 = Wire.read();   // receive bits 8 to 1 of x (one byte)
    uint8_t enc2_16_9 = Wire.read();   // receive bits 16 to 9 of y (one byte)
    uint8_t enc2_8_1 = Wire.read();   // receive bits 8 to 1 of y (one byte)
    encoder1 = (enc1_16_9 << 8) | enc1_8_1; // combine the two bytes into a 16 bit number
    encoder2 = (enc2_16_9 << 8) | enc2_8_1; // combine the two bytes into a 16 bit number
  }

  // distance in cm
  float distance=(encoder1*(0.06*M_PI)/24)*100;
  //Serial.println(distance);
  delay(100);
  return distance;
}

void straight_line_PID(int16_t leftMotor_speed, int16_t rightMotor_speed, int16_t servoAngle, int setpoint){
  int error,prev_error;
  double pid,Kp=3.9,Ki=0,Kd=0,cumulative_error=0;
  mpu.update();
  error = mpu.getAngleZ()-setpoint;
  Serial.print("Setpoint: ");Serial.println(setpoint);
  Serial.print("Z angle: ");Serial.println(mpu.getAngleZ());
  Serial.print("Error: ");Serial.println(error);
  pid = (error*Kp); //+(cumulative_error*Ki)+((prev_error-error)*Kd);
  Serial.print("PID: ");Serial.println(pid);
  servoAngle = 90+pid;
  Serial.print("Servo: ");Serial.println(servoAngle);
  //cumulative_error+=error;
  //prev_error = error;
  transmit_to_arduino(leftMotor_speed,rightMotor_speed,servoAngle);
  Serial.print("Setpoint: ");Serial.println(setpoint);
  delay(60);
}

void straight_line_PID2(int16_t leftMotor_speed, int16_t rightMotor_speed, int16_t servoAngle, int setpoint){
  int error,prev_error;
  double pid,Kp=3.9,Ki=0,Kd=0,cumulative_error=0;
  mpu.update();
  error = setpoint - mpu.getAngleZ();
  Serial.print("Setpoint: ");Serial.println(setpoint);
  Serial.print("Z angle: ");Serial.println(mpu.getAngleZ());
  Serial.print("Error: ");Serial.println(error);
  pid = (error*Kp); //+(cumulative_error*Ki)+((prev_error-error)*Kd);
  Serial.print("PID: ");Serial.println(pid);
  servoAngle = 90+pid;
  Serial.print("Servo: ");Serial.println(servoAngle);
  //cumulative_error+=error;
  //prev_error = error;
  transmit_to_arduino(leftMotor_speed,rightMotor_speed,servoAngle);
  Serial.print("Setpoint: ");Serial.println(setpoint);
  delay(60);
}

// Define maze navigation functions
void forward(int distance) {
  // Code to move forward a given distance
  lcd.setCursor(0, 1);
  lcd.print("Moving forward");
  float current_distance = get_distance();
  mpu.update();
  int setpoint =mpu.getAngleZ();
  //Serial.println(setpoint);
  while((get_distance()-current_distance)<(distance*10)){
    leftMotor_speed=150;
    rightMotor_speed=150;
    servoAngle=90;
    straight_line_PID(leftMotor_speed,rightMotor_speed,servoAngle, setpoint);
  }
  leftMotor_speed=0;
  rightMotor_speed=0;
  servoAngle=90;
  transmit_to_arduino(leftMotor_speed,rightMotor_speed,servoAngle);
  delay(40);
  lcd.clear();
}

void backward(int distance) {
  // Code to move backward a given distance
  lcd.setCursor(0, 1);
  lcd.print("Moving backward");
  float current_distance = get_distance();
  mpu.update();
  int setpoint =mpu.getAngleZ();
  while((get_distance()-current_distance)>(distance*10*-1)){
    leftMotor_speed=-150;
    rightMotor_speed=-150;
    servoAngle=90;
    straight_line_PID2(leftMotor_speed,rightMotor_speed,servoAngle, setpoint);
  }
  
  leftMotor_speed=0;
  rightMotor_speed=0;
  servoAngle=90;
  transmit_to_arduino(leftMotor_speed,rightMotor_speed,servoAngle);
  delay(40);
  lcd.clear();
}

void left(int angle) {
  // Code to turn left by a given angle
  lcd.setCursor(0, 1);
  lcd.print("Turning left");
  if (angle==1){
    angle=90;
  }
  else{
    angle=180;
  }
  mpu.update();
  float current_angle = mpu.getAngleZ();
  while((mpu.getAngleZ()-current_angle)<angle){
    mpu.update();
    Serial.println(mpu.getAngleZ()-current_angle);
    leftMotor_speed=125;
    rightMotor_speed=155;
    servoAngle=60;
    transmit_to_arduino(leftMotor_speed,rightMotor_speed,servoAngle);
    delay(50);
  }
  leftMotor_speed=0;
  rightMotor_speed=0;
  servoAngle=90;
  transmit_to_arduino(leftMotor_speed,rightMotor_speed,servoAngle);
  delay(30);
  lcd.clear();
}

void right(int angle) {
  // Code to turn right by a given angle
  lcd.setCursor(0, 1);
  lcd.print("Turning right");
  if (angle==1){
    angle=90;
  }
  else{
    angle=180;
  }
  mpu.update();
  float current_angle = mpu.getAngleZ();
  while((current_angle-mpu.getAngleZ())<angle){
    mpu.update();
    Serial.println(current_angle-mpu.getAngleZ());
    leftMotor_speed=155;
    rightMotor_speed=125;
    servoAngle=126;
    transmit_to_arduino(leftMotor_speed,rightMotor_speed,servoAngle);
    delay(50);
  }
  leftMotor_speed=0;
  rightMotor_speed=0;
  servoAngle=90;
  transmit_to_arduino(leftMotor_speed,rightMotor_speed,servoAngle);
  delay(30);
  lcd.clear();
}
