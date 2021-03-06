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

uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
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

int thrusterpower[8];

char *strings[8];

char *ptr = NULL;
  
char input[40];

String strinput;

byte index;

#define arr_len( x )  ( sizeof( x ) / sizeof( *x ) )

void displaySensorDetails(void)
{
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
  // put your setup code here, to run once:
  LBthruster.attach(2);
  LFthruster.attach(3);
  RBthruster.attach(4);
  RFthruster.attach(5);
  BLthruster.attach(6);
  BRthruster.attach(7);
  FLthruster.attach(8);
  FRthruster.attach(9);

  LBthruster.writeMicroseconds(1500); // send "stop" signal to ESC. Also necessary to arm the ESC.
  LFthruster.writeMicroseconds(1500);
  RBthruster.writeMicroseconds(1500);
  RFthruster.writeMicroseconds(1500);
  BLthruster.writeMicroseconds(1500);
  BRthruster.writeMicroseconds(1500);
  FLthruster.writeMicroseconds(1500);
  FRthruster.writeMicroseconds(1500);
  
  delay(7000); // delay to allow the ESC to recognize the stopped signal.
  Serial.begin(115200);
  Serial.println("Thrusters armed.");
  Serial.println("Orientation Sensor Test"); Serial.println("");
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);

  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
   
  /* Display some basic information on this sensor */
  displaySensorDetails();
}

void loop() {
  // put your main code here, to run repeatedly:
  
  sensors_event_t orientationData , linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  //  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
  yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;
  zPos = zPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.z;
  
  headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);

  if(Serial.available()){
    byte index = 0;
    strinput = Serial.readString();
    strinput.toCharArray(input, 40);
    Serial.print("Input string = ");
    Serial.println(input);
    ptr = strtok(input, ":");  // takes a list of delimiters
    if((strinput.compareTo("STOP")==10)) {
      Serial.println("Stopping.");
      for(int i=0; i < arr_len(thrusterpower); i++){
        thrusterpower[i] = 1500;
      }
    }
    else{
    while(ptr != NULL)
      {
          strings[index] = ptr;
          thrusterpower[index] = atoi((char *)strings[index]);
          index++;
          ptr = strtok(NULL, ":");  // takes a list of delimiters
      }
    }
  }
  LBthruster.writeMicroseconds(thrusterpower[0]); // send "stop" signal to ESC. Also necessary to arm the ESC.
  LFthruster.writeMicroseconds(thrusterpower[1]);
  RBthruster.writeMicroseconds(thrusterpower[2]);
  RFthruster.writeMicroseconds(thrusterpower[3]);
  BLthruster.writeMicroseconds(thrusterpower[4]);
  BRthruster.writeMicroseconds(thrusterpower[5]);
  FLthruster.writeMicroseconds(thrusterpower[6]);
  FRthruster.writeMicroseconds(thrusterpower[7]);
  
  if (printCount * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {
    Serial.print("Orientation:");
    Serial.print(orientationData.orientation.x);
    Serial.print(":");
    Serial.print(orientationData.orientation.y);
    Serial.print(":");
    Serial.print(orientationData.orientation.z);
    Serial.println("");

//    Serial.print("Position:");
//    Serial.print(xPos);
//    Serial.print(":");
//    Serial.print(yPos);
//    Serial.print(":");
//    Serial.print(zPos);
//    Serial.println("");
  }
  else {
    printCount = printCount + 1;
  }


}
