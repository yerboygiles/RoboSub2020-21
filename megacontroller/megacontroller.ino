#include <Servo.h>
/*      
self.ThrusterLB = ThrusterDriver(2, self.board)  # left back
self.ThrusterLF = ThrusterDriver(4, self.board)  # left front
self.ThrusterRB = ThrusterDriver(3, self.board)  # right back
self.ThrusterRF = ThrusterDriver(5, self.board)  # right front
self.ThrusterBL = ThrusterDriver(6, self.board)  # back left
self.ThrusterBR = ThrusterDriver(7, self.board)  # back right
self.ThrusterFL = ThrusterDriver(8, self.board)  # front left
self.ThrusterFR = ThrusterDriver(9, self.board)  # front right
*/

byte stopped = 1;
byte waiting = 2;
byte active = 3;
byte state = stopped;
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
void setup() {
  const char delim[2] = "-";
  const char delimalt[2] = " ";
  int thrusterpower[6];
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
}

void loop(){
  
  String token;
    /*I want the motor data in stuff to be structured like:
     * LBpower - LFpower - RBpower - RFpower - BLpower - BRpower - FLpower - FRpower
     *
     */
    if (Serial.available()) {
      char* datain = Serial.readString(); //reading
      Serial.print("Received Data => ");
      Serial.println(datain);//display same received Data back in serial monitor.
      if(state == stopped){
            if (datain == "START"){
                state = waiting;
            }
        }
      else if(state == waiting){
//        if (!bno.begin())
//        {
//            /* There was a problem detecting the BNO055 ... check your connections */
//            Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
//        }
//        if (bno.isFullyCalibrated())
//        {
//            Serial.print("Calibrated! Starting BNO055 Sensor Reads");
//        }
          state = active;
      }
      else if(state == active){
        //thruster power control, receive from the fruit
        int i = 0;
        if (datain != "END"){

            token = strtok(datain,',');
            /* walk through values in the datain we're getting */
            while( token != NULL ) {
              thrusterpower[i] = int(token);
              i = i + 1;
              token = strtok(NULL,",");
            }
            updatethrusters();
            //BNO055 controller, send to raspberry*
//            sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
//            bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
//            bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
//            bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
//            bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
//            bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
//            bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
//
//            uint8_t system, gyro, accel, mag = 0;
//            bno.getCalibration(&system, &gyro, &accel, &mag);
//            Serial.println();
//            Serial.print("Calibration: Sys=");
//            Serial.print(system);
//            Serial.print(" Gyro=");
//            Serial.print(gyro);
//            Serial.print(" Accel=");
//            Serial.print(accel);
//            Serial.print(" Mag=");
//            Serial.println(mag);
      }
      else:
            state = stopped;
    }
}
void updatethrusters(){
    LBthruster.writeMicroseconds(thrusterpower[0]); // send "stop" signal to ESC. Also necessary to arm the ESC.
    LFthruster.writeMicroseconds(thrusterpower[1]);
    RBthruster.writeMicroseconds(thrusterpower[2]);
    RFthruster.writeMicroseconds(thrusterpower[3]);
    BLthruster.writeMicroseconds(thrusterpower[4]);
    BRthruster.writeMicroseconds(thrusterpower[5]);
    FLthruster.writeMicroseconds(thrusterpower[6]);
    FRthruster.writeMicroseconds(thrusterpower[7]);
}
