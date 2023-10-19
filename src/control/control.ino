#include "BluetoothSerial.h"
#include <ESP32Servo.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define BASE_HGT 8      //base hight 2.65"
#define HUMERUS 12.5      //shoulder-to-elbow "bone" 5.75"
#define ULNA 9.5       //elbow-to-wrist "bone" 7.375"
#define GRIPPER 17          //gripper (incl.heavy duty wrist rotate mechanism) length 3.94"

#define ftl(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))  //float to long conversion

float hum_sq = HUMERUS*HUMERUS;
float uln_sq = ULNA*ULNA;



BluetoothSerial SerialBT;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;// create servo object to control a servo
// 16 servo objects can be created on the ESP32
int pos = 0;    // variable to store the servo position
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
int servoPin1 = 12;
int servoPin2 = 13;
int servoPin3 = 15;
int servoPin4 = 14;
int servoPin5 = 2;
int servoPin6 = 4;
int servo1Pos;
int servo2Pos;
int servo3Pos;
int servo4Pos;
int servo5Pos;
int servo6Pos;
int servo1PPos;
int servo2PPos;
int servo3PPos;
int servo4PPos;
int servo5PPos;
int servo6PPos;
int servo01SP[50];
int servo02SP[50];
int servo03SP[50];
int servo04SP[50];
int servo05SP[50];
int servo06SP[50];
int speedDelay = 20;
String dataIn = "";
int circle_count=0;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

    // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(50);    // standard 50 hz servo
  servo2.setPeriodHertz(50);    // standard 50 hz servo
  servo3.setPeriodHertz(50);    // standard 50 hz servo1
  servo4.setPeriodHertz(50);    // standard 50 hz servo
  servo5.setPeriodHertz(50);    // standard 50 hz servo
  servo6.setPeriodHertz(50);    // standard 50 hz servo
  servo1.attach(servoPin1,1000,2500);
  servo2.attach(servoPin2,1000,2500);
  servo3.attach(servoPin3,1000,2500);
  servo4.attach(servoPin4,1000,2500);
  servo5.attach(servoPin5,1000,2500);
  servo6.attach(servoPin6,1000,2500);// attaches the servo on pin 18 to the servo object
  // using default min/max of 1000us and 2000us
  // different servos may require different min/max settings
  // for an accurate 0 to 180 sweep

  // Robot arm initial position
  servo1PPos = 90;
  servo1.write(servo1PPos);
  delay(700);
  servo2PPos = 90;
  servo2.write(servo2PPos);
  delay(700);
  servo3PPos = 90;
  servo3.write(servo3PPos);
  delay(700);
  servo4PPos = 90;
  servo4.write(servo4PPos);
  delay(700);
  servo5PPos = 90;
  servo5.write(servo5PPos);
  delay(700);
  servo6PPos = 180;
  servo6.write(servo6PPos);
  
}


void loop() {
//  // put your main code here, to run repeatedly:
//    if(circle_count==0||circle_count==1){
////      circle();
//      line(circle_count);
//      
//      delay(1000);
//      circle_count = circle_count+1;
//    }
    if (SerialBT.available() > 0) {
    Serial.println(SerialBT.available());
    dataIn = SerialBT.readString();  // Read the data as string
    Serial.println(dataIn);

    if(dataIn=="SAVE"){
      IK(26.5,0,20.5);
    }

      if (dataIn.startsWith("px")) {
       int px = dataIn.substring(dataIn.lastIndexOf("px")+2, dataIn.lastIndexOf("py")).toInt();
       int py = dataIn.substring(dataIn.lastIndexOf("py")+2, dataIn.lastIndexOf("pz")-1).toInt();
       int pz = dataIn.substring(dataIn.lastIndexOf("pz")+2, dataIn.length()).toInt();
       IK(px,py,pz);
        }
    // If "Waist" slider has changed value - Move Servo 1 to position
    if (dataIn.startsWith("s1")) {
      Serial.println(dataIn.lastIndexOf("s1"));
      String dataInS = dataIn.substring(dataIn.lastIndexOf("s1")+2, dataIn.length()); // Extract only the number. E.g. from "s1120" to "120"
      Serial.println(dataInS);
      servo1Pos = dataInS.toInt();  // Convert the string into integer
      // We use for loops so we can control the speed of the servo
      // If previous position is bigger then current position
      Serial.println(servo1Pos);
      Serial.println(servo1PPos);
      if (servo1PPos > servo1Pos) {
        for ( int j = servo1PPos; j >= servo1Pos; j--) {   // Run servo down
          Serial.println(j);
          servo1.write(j);
          delay(20);    // defines the speed at which the servo rotates
        }

        
      }
      // If previous position is smaller then current position
      if (servo1PPos < servo1Pos) {
        for ( int j = servo1PPos; j <= servo1Pos; j++) {   // Run servo up
          Serial.println(j);
          servo1.write(j);
          delay(20);
        }

      }
      servo1PPos = servo1Pos;   // set current position as previous position
    }

    // If "Waist" slider has changed value - Move Servo 2 to position
    if (dataIn.startsWith("s2")) {
      Serial.println(dataIn.lastIndexOf("s2"));
      String dataInS = dataIn.substring(dataIn.lastIndexOf("s2")+2, dataIn.length()); // Extract only the number. E.g. from "s1120" to "120"
      Serial.println(dataInS);
      servo2Pos = dataInS.toInt();  // Convert the string into integer
      // We use for loops so we can control the speed of the servo
      // If previous position is bigger then current position
      Serial.println(servo2Pos);
      Serial.println(servo2PPos);
      if (servo2PPos > servo2Pos) {
        for ( int j = servo2PPos; j >= servo2Pos; j--) {   // Run servo down
          //Serial.println(j);
          servo2.write(j);
          delay(20);    // defines the speed at which the servo rotates
        }

        
      }
      // If previous position is smaller then current position
      if (servo2PPos < servo2Pos) {
        for ( int j = servo2PPos; j <= servo2Pos; j++) {   // Run servo up
          //Serial.println(j);
          servo2.write(j);
          delay(20);
        }

      }
      servo2PPos = servo2Pos;   // set current position as previous position
    }

    // If "Waist" slider has changed value - Move Servo 3 to position
    if (dataIn.startsWith("s3")) {
      Serial.println(dataIn.lastIndexOf("s3"));
      String dataInS = dataIn.substring(dataIn.lastIndexOf("s3")+2, dataIn.length()); // Extract only the number. E.g. from "s1120" to "120"
      Serial.println(dataInS);
      servo3Pos = dataInS.toInt();  // Convert the string into integer
      // We use for loops so we can control the speed of the servo
      // If previous position is bigger then current position
      Serial.println(servo3Pos);
      Serial.println(servo3PPos);
      if (servo3PPos > servo3Pos) {
        for ( int j = servo3PPos; j >= servo3Pos; j--) {   // Run servo down
          Serial.println(j);
          servo3.write(j);
          delay(20);    // defines the speed at which the servo rotates
        }

        
      }
      // If previous position is smaller then current position
      if (servo3PPos < servo3Pos) {
        for ( int j = servo3PPos; j <= servo3Pos; j++) {   // Run servo up
          Serial.println(j);
          servo3.write(j);
          delay(20);
        }

      }
      servo3PPos = servo3Pos;   // set current position as previous position
    }

    // If "Waist" slider has changed value - Move Servo 4 to position
    if (dataIn.startsWith("s4")) {
      Serial.println(dataIn.lastIndexOf("s4"));
      String dataInS = dataIn.substring(dataIn.lastIndexOf("s4")+2, dataIn.length()); // Extract only the number. E.g. from "s1120" to "120"
      Serial.println(dataInS);
      servo4Pos = dataInS.toInt();  // Convert the string into integer
      // We use for loops so we can control the speed of the servo
      // If previous position is bigger then current position
      Serial.println(servo4Pos);
      Serial.println(servo4PPos);
      if (servo4PPos > servo4Pos) {
        for ( int j = servo4PPos; j >= servo4Pos; j--) {   // Run servo down
          Serial.println(j);
          servo4.write(j);
          delay(20);    // defines the speed at which the servo rotates
        }

        
      }
      // If previous position is smaller then current position
      if (servo4PPos < servo4Pos) {
        for ( int j = servo4PPos; j <= servo4Pos; j++) {   // Run servo up
          Serial.println(j);
          servo4.write(j);
          delay(20);
        }

      }
      servo4PPos = servo4Pos;   // set current position as previous position
    }

    // If "Waist" slider has changed value - Move Servo 5 to position
    if (dataIn.startsWith("s5")) {
      Serial.println(dataIn.lastIndexOf("s5"));
      String dataInS = dataIn.substring(dataIn.lastIndexOf("s5")+2, dataIn.length()); // Extract only the number. E.g. from "s1120" to "120"
      Serial.println(dataInS);
      servo5Pos = dataInS.toInt();  // Convert the string into integer
      // We use for loops so we can control the speed of the servo
      // If previous position is bigger then current position
      Serial.println(servo5Pos);
      Serial.println(servo5PPos);
      if (servo5PPos > servo5Pos) {
        for ( int j = servo5PPos; j >= servo5Pos; j--) {   // Run servo down
          Serial.println(j);
          servo5.write(j);
          delay(20);    // defines the speed at which the servo rotates
        }

        
      }
      // If previous position is smaller then current position
      if (servo5PPos < servo5Pos) {
        for ( int j = servo5PPos; j <= servo5Pos; j++) {   // Run servo up
          Serial.println(j);
          servo5.write(j);
          delay(20);
        }

      }
      servo5PPos = servo5Pos;   // set current position as previous position
    }

    // If "Waist" slider has changed value - Move Servo 6 to position
    if (dataIn.startsWith("s6")) {
      Serial.println(dataIn.lastIndexOf("s6"));
      String dataInS = dataIn.substring(dataIn.lastIndexOf("s6")+2, dataIn.length()); // Extract only the number. E.g. from "s1120" to "120"
      Serial.println(dataInS);
      servo6Pos = dataInS.toInt();  // Convert the string into integer
      // We use for loops so we can control the speed of the servo
      // If previous position is bigger then current position
      Serial.println(servo6Pos);
      Serial.println(servo6PPos);
      if (servo6PPos > servo6Pos) {
        for ( int j = servo6PPos; j >= servo6Pos; j--) {   // Run servo down
          Serial.println(j);
          servo6.write(j);
          delay(20);    // defines the speed at which the servo rotates
        }

        
      }
      // If previous position is smaller then current position
      if (servo6PPos < servo6Pos) {
        for ( int j = servo6PPos; j <= servo6Pos; j++) {   // Run servo up
          Serial.println(j);
          servo6.write(j);
          delay(20);
        }

      }
      servo6PPos = servo6Pos;   // set current position as previous position
    }

    
  }
}


/* arm positioning routine utilizing inverse kinematics */
/* z is height, y is distance from base center out, x is side to side. y,z can only be positive */
//void set_arm( uint16_t x, uint16_t y, uint16_t z, uint16_t grip_angle )
void set_arm( float x, float y, float z, float grip_angle_d )
{
  float grip_angle_r = radians( grip_angle_d );    //grip angle in radians for use in calculations
  /* Base angle and radial distance from x,y coordinates */
  float bas_angle_r = atan2( x, y );
  float rdist = sqrt(( x * x ) + ( y * y ));
  /* rdist is y coordinate for the arm */
  y = rdist;
  /* Grip offsets calculated based on grip angle */
  float grip_off_z = ( sin( grip_angle_r )) * GRIPPER;
  float grip_off_y = ( cos( grip_angle_r )) * GRIPPER;
  /* Wrist position */
  float wrist_z = ( z - grip_off_z ) - BASE_HGT;
  float wrist_y = y - grip_off_y;
  /* Shoulder to wrist distance ( AKA sw ) */
  float s_w = ( wrist_z * wrist_z ) + ( wrist_y * wrist_y );
  float s_w_sqrt = sqrt( s_w );
  /* s_w angle to ground */
  //float a1 = atan2( wrist_y, wrist_z );
  float a1 = atan2( wrist_z, wrist_y );
  /* s_w angle to humerus */
  float a2 = acos((( hum_sq - uln_sq ) + s_w ) / ( 2 * HUMERUS * s_w_sqrt ));
  /* shoulder angle */
  float shl_angle_r = a1 + a2;
  float shl_angle_d = degrees( shl_angle_r );
  /* elbow angle */
  float elb_angle_r = acos(( hum_sq + uln_sq - s_w ) / ( 2 * HUMERUS * ULNA ));
  float elb_angle_d = degrees( elb_angle_r );
  float elb_angle_dn = -( 180.0 - elb_angle_d );
  /* wrist angle */
  float wri_angle_d = ( grip_angle_d - elb_angle_dn ) - shl_angle_d;
 
  /* Servo pulses */
//  float bas_servopulse = 1500.0 - (( degrees( bas_angle_r )) * 11.11 );
//  float shl_servopulse = 1500.0 + (( shl_angle_d - 90.0 ) * 6.6 );
//  float elb_servopulse = 1500.0 -  (( elb_angle_d - 90.0 ) * 6.6 );
//  float wri_servopulse = 1500 + ( wri_angle_d  * 11.1 );
 
  /* Set servos */
//  servos.setposition( servo1, ftl( bas_servopulse ));
//  servos.setposition( servo4, ftl( wri_servopulse ));
//  servos.setposition( servo2, ftl( shl_servopulse ));
//  servos.setposition( servo3, ftl( elb_servopulse ));


 servo1.write( degrees( bas_angle_r ));
 servo2.write( ( shl_angle_d  ));
 servo3.write( (elb_angle_d  ));

 Serial.println(degrees(bas_angle_r));
 Serial.println(shl_angle_d );
 Serial.println(elb_angle_d-90);
 Serial.println(wri_angle_d );
}

int moveServo(Servo servo, int PrePos, int Pos){
  if (PrePos > Pos) {
        for ( int j = PrePos; j >= Pos; j--) {   // Run servo down
          servo.write(j);
          //Serial.write(j);
          delay(30);    // defines the speed at which the servo rotates
        }

        
      }
      // If previous position is smaller then current position
      if (PrePos < Pos) {
        for ( int j = PrePos; j <= Pos; j++) {   // Run servo up
          servo.write(j);
          //Serial.write(j);
          delay(30);
        }

      }
      return Pos;
}

void IK(float x, float y, float z){
  Serial.println(x);
  Serial.println(y);
  Serial.println(z);
  float bas_angle_r = atan2( y , x );
  float A = sqrt(x*x + y*y);
  float B = z - BASE_HGT;
  float C = (A*A + B*B + HUMERUS*HUMERUS - (ULNA+GRIPPER)*(ULNA+GRIPPER))/(2*HUMERUS);
  float E = (sqrt((A*A)+(B*B)-(C*C)));
  float D = (B+E)/(A+C);
//  Serial.println(A);
//  Serial.println(B);
//  Serial.println(C);
//  Serial.println(D);
//  Serial.println(E);
  float shl_angle_d = degrees(2*(atan2(D,1)));
  float elb_angle_d = degrees(asin(((z - BASE_HGT - (HUMERUS * (sin(radians(shl_angle_d)))))/(ULNA+GRIPPER)))) - shl_angle_d;
  
  moveServo(servo1,servo1PPos,(int)degrees(bas_angle_r));
  moveServo(servo2,servo2PPos,(int)shl_angle_d);
  moveServo(servo3,servo3PPos,(int)(elb_angle_d+90));
  
//  servo1.write(degrees(bas_angle_r));
//  servo2.write(shl_angle_d);
//  servo3.write(elb_angle_d+90);
 Serial.println(servo1PPos);
 Serial.println(servo2PPos );
 Serial.println(servo3PPos);
 Serial.println();
 Serial.println(degrees(bas_angle_r));
 Serial.println(shl_angle_d );
 Serial.println(elb_angle_d+90);
 Serial.println();

}

void line(int i){
  if(i==0) {IK(26.5,0,20.5);}
  if(i==1) {IK(39,0,8);}
 Serial.println(servo1PPos);
 Serial.println(servo2PPos);
 Serial.println(servo3PPos);
  
//int PPos = 0;
//  PPos = moveServo(servo1,servo1PPos,0);
//  servo1PPos = PPos;
//  PPos = moveServo(servo2,servo2PPos,90);
//  servo2PPos = PPos;
//  PPos = moveServo(servo3,servo3PPos,0);
//  servo3PPos = PPos;
//
//  
//  PPos = moveServo(servo1,servo1PPos,0);
//  servo1PPos = PPos;
//  PPos = moveServo(servo2,servo2PPos,10);
//  servo2PPos = PPos;
//  PPos = moveServo(servo3,servo3PPos,80);
//  servo3PPos = PPos;
}

void circle()
{
  #define RADIUS 8.0
  //float angle = 0;
  float zaxis,yaxis;
  for( float angle = 0.0; angle < 360.0; angle += 1.0 ) {
      yaxis = RADIUS * sin( radians( angle )) + 25;
      zaxis = RADIUS * cos( radians( angle )) + 25;
      Serial.println(yaxis);
      Serial.println(zaxis);
      //IK( 25, yaxis, zaxis );
      delay( 1 );
  }
}
