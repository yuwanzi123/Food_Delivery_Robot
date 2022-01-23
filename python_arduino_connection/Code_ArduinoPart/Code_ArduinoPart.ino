
#include <Servo.h>
#include <SoftwareSerial.h>
#include <NewPing.h>
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 3 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
void ObtainAngle();
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
double dYaw,PastYaw,CriteriaYaw,sumYaw;
bool Calibrate;
int  Calibratecount;

//Servo motor
Servo Leftservo;
Servo Rightservo;
Servo Gripper;
void WheelControl(int Left,int Right);
void GripperControl(int Grip);
//Distance seosor
const int FPin=6; int BPin=4;
float invcmCosnt = (2*1000000)/(100*344.8); 
float rawTime1, cmDist1; 
float Dis[4];

#define TRIGGER_PIN  6
#define ECHO_PIN     6
#define MAX_DISTANCE 500
#define TRIGGER_PIN2  7
#define ECHO_PIN2     7
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
NewPing rear(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);
void Distance();
void RearDistance();

//Xbee 
void XbeeFunction();
const int TxPin=5;
const int RxPin=4;
char transData,receiveData,rawData;
SoftwareSerial Xbee(RxPin,TxPin);//Rx,TX

// Raspberrypi
unsigned int incomingByte,Byte;
char incoming;

//main control
int mode,countCase2,CarryCounter;
int dammycounter;
bool rotateFlag,straightFlag;
int waitingcounter;
long waitTime;
 unsigned long Time;
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}




void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(115200);
    
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

   

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

 

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);//this is reason why we use digitalPIN2
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(3, OUTPUT);
    Leftservo.attach(12);
    Rightservo.attach(13);
    Gripper.attach(11);  
    Gripper.write(10); 
    // initial setting
    dYaw=0;PastYaw=0;sumYaw=0;
    Calibrate=false;rotateFlag=false;straightFlag=false;
    Calibratecount=0;countCase2=0;
    transData='N',receiveData='N';
    mode=1;CarryCounter=0;
    incomingByte = 1;
    dammycounter=0;waitTime=0;
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

Serial.print("mpuInterrupt= ");
Serial.print(mpuInterrupt);
Serial.print("fifoCount= ");
Serial.print(fifoCount);
Serial.print("packet= ");
Serial.println(packetSize);
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt) {
      if(mpuInterrupt==true) break;
  
//************** KOTA add for main control part from this line****************************************
//*****Receive image information from Raspberry pi3 *****************************:
  if(Serial.available()>0){
  Byte= (Serial.read()-'0');
   if(Byte<=6 && Byte>=0){
    incomingByte=Byte;
    }
  }

    Serial.print("image is ");
    Serial.println(incomingByte);
//*************By this line********************


//***********Select transData for Xbee*************************
switch(mode){
  case 7:
  if(incomingByte==5){// input the information fcrom rasberry pi 
   transData='F';//will infoprm the partner to carry the object front directin to you
   }
  else {transData='R'; //carry for back direction to you
  }
  break;
  default:
  ;
  }
  


//Motion choice based on mode       
 switch(mode){
  case 1://Calibration is still going on
       Serial.println("mode1");
       digitalWrite(3, HIGH);
       WheelControl(1500,1500);
    if(dYaw<0.01 && dYaw>-0.01){
            Calibratecount++;
            Serial.print(" counter=");
            Serial.println(Calibratecount);
            if(Calibratecount>=100){
                mode=3;//first movement is straight
                digitalWrite(3, LOW);
               CriteriaYaw=ypr[0] * 180/M_PI;
                sumYaw=0;
            }
       }
  break;
  case 2://main control start
        // case 2 is turn left 
       Serial.println("turn left");
       digitalWrite(3, LOW);
       WheelControl(1480,1480);
      Serial.print("CriteriaYaw=");
       Serial.println(CriteriaYaw);
       Serial.print("sumYaw=");
       Serial.println(sumYaw);
  if(abs(sumYaw)>90){
    if(incomingByte==5 || incomingByte==6){
    mode=6;//go to the objet
    sumYaw=0;
    } 
    else {
    mode=9;//after riotating, the robot must go straight
    sumYaw=0;
      }

        }
  break;
  case 3://straight pattern
       digitalWrite(3, HIGH);
       Serial.println("straight1");
       Distance();
       if(cmDist1<20){
        sumYaw=0;
        WheelControl(1500,1500); 
        mode=9;//stop
        }
        else{
       WheelControl(1523,1470); 
       } 
  break;
  case 4://turn Right
    Serial.println("turn right");
  digitalWrite(3, LOW);
  WheelControl(1520,1520);
  Serial.print("CriteriaYaw=");
  Serial.println(CriteriaYaw);
  Serial.print("sumYaw=");
  Serial.println(sumYaw);
    if(abs(sumYaw)>90){
      if(incomingByte==5 || incomingByte==6){
    //information from raspberry pi. Found object
    mode=6;//go to the objet
    sumYaw=0;
    } 
    else{
     mode=9;
     sumYaw=0;
    }
    }
  break;

  case 6:// approach to the object
  Serial.println("found object");
  MeasureDistance(FPin);
  if(cmDist1<5){
    WheelControl(1500,1500);
    GripperControl(100);//just grasp not lift up
    mode=7;//go to wait and send messeage
    }
    else{
     WheelControl(1523,1470); 
    }
  break;
  case 7://finish grasping the object
  Xbee.begin(9600);
  Xbee.listen();
  Serial.println("finish grasping object");
  digitalWrite(3, HIGH);
  WheelControl(1500,1500); 
  XbeeFunction();
  if(receiveData=='F' || receiveData=='R'){
    mode=8;
    }
  break;
  case 8://carry the object to the desired direction
  GripperControl(170);
 XbeeFunction();
  if(receiveData=='F'){// partner robot will go to forward direcction
    RearDistance(); //the robot must measure distance 
    if(cmDist1>5){
      Serial.println("Progressing");
    WheelControl(1470,1523);//go to back
    transData='R';
    }
    else{
      Serial.println("STOP");
       transData='S';
        XbeeFunction();
        Xbee.end();
       mode=10;
      }
    }
    else if(receiveData=='R'){//partner is back, leader is forward
      Serial.println("Progressing");
      WheelControl(1523,1470);//go to forward(for leader)
      transData='F';
      }
     else if(receiveData=='S'){ 
      Serial.println("STOP");
       WheelControl(1500,1500);//stop
       GripperControl(10);
       transData='S';
       XbeeFunction();
       mode=10;
      }
     else{}//unvalid Xbee data
  //}

  break;
  case 9:
 if(waitingcounter<10){//because of time delay
  Serial.println("doing Image processing" );
  Serial.print("Waiting is ");
  Serial.println (waitingcounter);
  WheelControl(1500,1500);
  waitingcounter++;
  }
  else{
    waitingcounter=0;
  if(incomingByte==1){
  mode=3;//straight
  }
  else if(incomingByte==3){
    mode=2;//turn left
    }
   else if(incomingByte==4){
    mode=4;// turn right
    }
   else if(incomingByte==5 || incomingByte==6){
    mode=6;//found object
    }
   else{// incomingByte=2 or0
    mode=9;// do nothing
    }
  }
   break;
  case 10:
   WheelControl(1500,1500);
   delay(2000);
  GripperControl(10);
  break;
  
  default:
  Serial.println("Fail");
  WheelControl(1500,1500);
  }

//************************:By this line (KOTA)**************************::
        
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    //if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    if ((mpuIntStatus & 0x10) || fifoCount == 1024000) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } 
    else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        

        fifoCount -= packetSize;
            // display Euler angles in degrees
 
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
         
  //***************** Kota add fron this line************************************
          
           dYaw=-PastYaw+ypr[0] * 180/M_PI;
           Serial.print("RawdYaw\t");
            Serial.print(dYaw);          
           if(dYaw>100){// when Yaw angle changes from -180~0side to +180~0, dYaw will become very large
            dYaw=360-dYaw;
            }
            else if(dYaw<-100){
              dYaw=360-PastYaw+ypr[0] * 180/M_PI;
              }
            PastYaw=ypr[0] * 180/M_PI;
           sumYaw=sumYaw+dYaw;
          
            Serial.print("Mod-dYaw\t");
            Serial.println(dYaw);
  //****************by this line************************************

    }
}



void WheelControl(int Left,int Right){
  Leftservo.writeMicroseconds(Left);
  Rightservo.writeMicroseconds(Right);
  
  }
void GripperControl(int Grip){
  Gripper.write(Grip); 
  }
  
void Distance(){
  delay(5);
  Serial.print("front: ");
  cmDist1 =sonar.ping_cm();
  Serial.print(cmDist1);
  Serial.println("cm");
  }
void RearDistance(){
  delay(5);
  Serial.print("Rear: ");
  cmDist1 =rear.ping_cm();
  Serial.print(cmDist1);
  Serial.println("cm"); 
  }
  
void XbeeFunction(){
if(Xbee.available()){
  Xbee.write(transData);
  Serial.println("OK");
  rawData=Xbee.read();
  if(rawData=='F' || rawData=='R'||rawData=='S'||rawData=='N'){
    receiveData=rawData;
    Serial.print("receiving is");
    Serial.println(receiveData);
  }
  else{
       Serial.print("receiving is");
    Serial.println(receiveData);
    }//when unvalid data is reveived, the data is ignored
  Xbee.flush();
  delay(5);
 }
 else{
  Xbee.write(transData);
  Xbee.flush();
  Serial.println("Keep sending");
 // delay(5);
  }
  }
