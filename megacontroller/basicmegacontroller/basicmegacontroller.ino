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
  Serial.begin(9600);
  Serial.println("Thrusters armed.");
}

void loop() {
  // put your main code here, to run repeatedly:
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


}
