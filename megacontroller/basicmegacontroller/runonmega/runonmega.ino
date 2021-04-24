#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.
   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3-5V DC
   Connect GROUND to common ground
   History
   =======
   2015/MAR/03  - First release (KTOWN)
   2021/APR/16  - Integration into robosub (OTUS)
*/

double xPos = 0, yPos = 0, zPos = 0, headingVel = 0;
double xOffset, yOffset, zOffset;

int SystemCalib, GyroCalib, AccelCalib = 0;

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100; //how often to read data from the board
uint16_t PRINT_DELAY_MS = 500; // how often to print the data
uint16_t printCount = 0; //counter to avoid printing every 10MS sample

double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

byte LBpin = 2; //left back
byte LFpin = 3; //left front
byte RBpin = 4; //right back
byte RFpin = 5; //right front
byte BLpin = 6; //back left
byte BRpin = 7; //back right
byte FLpin = 8; //front left
byte FRpin = 9; //front right

Servo LBthruster;
Servo LFthruster;
Servo RBthruster;
Servo RFthruster;
Servo BLthruster;
Servo BRthruster;
Servo FLthruster;
Servo FRthruster;

bool thrustersValid;
bool secondSet;
bool stopped = true;
int thrusterpower[8];

char *strings[8];

char *ptr = NULL;
byte index;
byte indexed;
  
char input[40];

String strinput;


#define arr_len( x )  ( sizeof( x ) / sizeof( *x ) )

void displaySensorDetails(void) {
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
void setup() {
  Serial1.begin(115200);
  Serial.begin(115200);
  // put your setup code here, to run once:
  LBthruster.attach(2);
  LFthruster.attach(3);
  RBthruster.attach(4);
  RFthruster.attach(5);
  BLthruster.attach(6);
  BRthruster.attach(7);
  FLthruster.attach(8);
  FRthruster.attach(9);

  LBthruster.writeMicroseconds(1500); // send "arm" signal to ESC. Also necessary to arm the ESC.
  LFthruster.writeMicroseconds(1500);
  RBthruster.writeMicroseconds(1500);
  RFthruster.writeMicroseconds(1500);
  BLthruster.writeMicroseconds(1500);
  BRthruster.writeMicroseconds(1500);
  FLthruster.writeMicroseconds(1500);
  FRthruster.writeMicroseconds(1500);
  delay(7000); // delay to allow the ESC to recognize the stopped signal.
  Serial1.println("Thrusters armed, resetting to stop.");
  LBthruster.writeMicroseconds(0); // send "stop"/voltage off signal to ESC.
  LFthruster.writeMicroseconds(0);
  RBthruster.writeMicroseconds(0);
  RFthruster.writeMicroseconds(0);
  BLthruster.writeMicroseconds(0);
  BRthruster.writeMicroseconds(0);
  FLthruster.writeMicroseconds(0);
  FRthruster.writeMicroseconds(0);
  Serial.println("Orientation Sensor Test."); Serial.println("");
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial1.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  Serial.println("Calibration post start:");
  displayCalStatus();
  
  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
   
  /* Display some basic information on this sensor */
  displaySensorDetails();
  secondSet = false;
  

}

void writeMotors(void) {
  LBthruster.writeMicroseconds(thrusterpower[0]); // sending driving values to arduino
  LFthruster.writeMicroseconds(thrusterpower[1]);
  RBthruster.writeMicroseconds(thrusterpower[2]);
  RFthruster.writeMicroseconds(thrusterpower[3]);
  BLthruster.writeMicroseconds(thrusterpower[4]);
  BRthruster.writeMicroseconds(thrusterpower[5]);
  FLthruster.writeMicroseconds(thrusterpower[6]);
  FRthruster.writeMicroseconds(thrusterpower[7]);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  sensors_event_t orientationData , angVelData , linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
  yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;
  zPos = zPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.z;
  
  headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);
  thrustersValid = true;
//  Serial.print("LB: ");
//  Serial.print(thrusterpower[0]);
//  Serial.print("LF: ");
//  Serial.print(thrusterpower[1]);
//  Serial.print("RB: ");
//  Serial.print(thrusterpower[2]);
//  Serial.print("RF: ");
//  Serial.print(thrusterpower[3]);
//  Serial.print("BL: ");
//  Serial.print(thrusterpower[4]);
//  Serial.print("BR: ");
//  Serial.print(thrusterpower[5]);
//  Serial.print("FL: ");
//  Serial.print(thrusterpower[6]);
//  Serial.print("FR: ");
//  Serial.print(thrusterpower[7]);
//  Serial.println("");
  printEvent(&orientationData, Serial);
  //printEvent(&linearAccelData, Serial);
  //displayCalStatus();
  
  int j = 0;
  while(GyroCalib < 3){
    delay(1000);
    displayCalStatus();
    Serial.print("Calibrating... On run: ");
    Serial.print(j);
    Serial.println(". Wait 1.");
    j = j + 1;
    xOffset = orientationData.orientation.x;
    yOffset = orientationData.orientation.y;
    zOffset = orientationData.orientation.z;
    Serial.println(xOffset);
    Serial.println(yOffset);
    Serial.println(zOffset);
  }
  if(Serial1.available() > 0){ 
     
    strinput = Serial.readStringUntil('\n');
    strinput.toCharArray(input, 40);
//    Serial.print("Stop?: "); 
//    Serial.println((strinput.compareTo("STOP")));
//    Serial.print("Start?: "); 
//    Serial.println((strinput.compareTo("START")));
    if((strinput.compareTo("INIT")==0)) {
      Serial.println("Init. Wait 3.");
      thrusterpower[0] = 0;
      thrusterpower[1] = 0;
      thrusterpower[2] = 0;
      thrusterpower[3] = 0;
      thrusterpower[4] = 0;
      thrusterpower[5] = 0;
      thrusterpower[6] = 0;
      thrusterpower[7] = 0;
      writeMotors();
      printEvent(&orientationData, Serial1);
//      printEvent(&linearAccelData, Serial1);
      delay(3000);
    }
    else if((strinput.compareTo("STOP")==0)) {
      Serial.println("Thrusters disengaging. Wait 3.");
      stopped = true;
      thrusterpower[0] = 0;
      thrusterpower[1] = 0;
      thrusterpower[2] = 0;
      thrusterpower[3] = 0;
      thrusterpower[4] = 0;
      thrusterpower[5] = 0;
      thrusterpower[6] = 0;
      thrusterpower[7] = 0;
      writeMotors();
      delay(3000);
    }
    else if((strinput.compareTo("START")==0)) {
      Serial.println("Thrusters engaging. Wait 3.");
      stopped = false;
      thrusterpower[0] = 1500;
      thrusterpower[1] = 1500;
      thrusterpower[2] = 1500;
      thrusterpower[3] = 1500;
      thrusterpower[4] = 1500;
      thrusterpower[5] = 1500;
      thrusterpower[6] = 1500;
      thrusterpower[7] = 1500;
      writeMotors();
      delay(3000);
    } else if (!stopped){
      if(secondSet) {
      index = 3;
      indexed = 7;
    } else {
      index = 0;
      indexed = 3;
    }
    ptr = strtok(input, ":");  // takes a list of delimiters
    while(index <= indexed)
      {
          strings[index] = ptr;
          thrusterpower[index] = atoi((char *)strings[index]);
          if((thrusterpower[index] < 1100) || (thrusterpower[index] > 1900)){
            
          }
          index++;
          ptr = strtok(NULL, ":");  // takes a list of delimiters
      }
    secondSet = !secondSet;
    }
    // This is the testing shit, sends over USB instead of rxtx wire
//    Serial.print("LB: ");
//    Serial.print(thrusterpower[0]);
//    Serial.print("LF: ");
//    Serial.print(thrusterpower[1]);
//    Serial.print("RB: ");
//    Serial.print(thrusterpower[2]);
//    Serial.print("RF: ");
//    Serial.print(thrusterpower[3]);
//    Serial.print("BL: ");
//    Serial.print(thrusterpower[4]);
//    Serial.print("BR: ");
//    Serial.print(thrusterpower[5]);
//    Serial.print("FL: ");
//    Serial.print(thrusterpower[6]);
//    Serial.print("FR: ");
//    Serial.print(thrusterpower[7]);
//    Serial.println("");
    if(!stopped){
      writeMotors();
    }
    
    printEvent(&orientationData, Serial1);
    printEvent(&linearAccelData, Serial1);
    
//    Serial1.print("Orientation:");
//    Serial1.print(orientationData.orientation.x);
//    Serial1.print(":");
//    Serial1.print(orientationData.orientation.y);
//    Serial1.print(":");
//    Serial1.print(orientationData.orientation.z);
//    Serial1.println("");

//    Serial1.print("Position:");
//    Serial1.print(xPos);
//    Serial1.print(":");
//    Serial1.print(yPos);
//    Serial1.print(":");
//    Serial1.print(zPos);
//    Serial1.println("");
  }
}
void printEvent(sensors_event_t* event, Stream &serial) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    serial.print("Orient");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
    serial.print(":x:");
    if(x > 180) {
      serial.print((x - 180));
    } else if (x <= 180) {
      serial.print(-(x + 180));
      
    }
    serial.print(":y:");
    serial.print(y - yOffset);
    serial.print(":z:");
    serial.println(z - zOffset);
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
//    serial.print("Linear");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    serial.print("Unk:");
  }

  
}

void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
 
  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }
 
  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  SystemCalib = system;
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  GyroCalib = gyro;
  Serial.print(" A:");
  Serial.print(accel, DEC);
  AccelCalib = accel;
  Serial.print(" M:");
  Serial.println(mag, DEC);
}
