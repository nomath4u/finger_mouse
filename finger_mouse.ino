#include <Mouse.h>
#include "I2Cdev.h"
//#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
/* IDEAS
 *  Use the IRQ on the capacitive sensor in order to wake up the arduino.
 *  Make sure the bluetooth connection stays alive.
 *  project the filter onto the plane with gravity so roll doesn't matter
 */


/* with default values the accelerometer range is  +- 2g and += 250 deg/second
 *  leaving us with 
 *  8192 per g
 *  131 deg/s
 */
#define VAL_TO_G 8192
#define VAL_TO_DEG 131
#define INDEX_PAD 1 //Not sure what this is going to be yet
#define TRUE 1

struct accel_data{
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
};

struct mouse_pair{
  int x;
  int y;
};

struct accel_data read_acc(void);
struct mouse_pair calc_xy(struct accel_data);
void send_mouse_event(struct mouse_pair);
bool on_tick(void);
bool pinched(void);

MPU6050 mpu;


bool dmp_ready;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
uint16_t packetSize;
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
VectorInt16 gyro;

void setup() {
  uint8_t dev_status;
  dmp_ready = 0; //Assume failure
  Serial.begin(9600);
  
  //pinMode(INDEX_PAD, INPUT);
  Serial.println("Setup complete");
  /* Setup for MPU6050, hopefully it doesn't interfere with the capacitive sensor */
  Wire.begin();
  /* Calculate and set gyro/acceleromter offsets */
  mpu.initialize(); // Sets gyro to GYRO_FS_250 and accel to ACCEL_FS_2 (250deg/s and 2g)
  dev_status = mpu.dmpInitialize(); //Begin dmp programming
  /* Supply gyro and accel offsets once I have them */
  if(dev_status == 0){
    //Can enable dmp now
    mpu.setDMPEnabled(TRUE);
    //Setup interupt once I have it
    dmp_ready = TRUE; //So we can actually use it in the main loop
    packetSize = mpu.dmpGetFIFOPacketSize();
     
  } else {
    //Bad stuff
    Serial.print("DMP ERROR: "); Serial.println(dev_status);
  }
}

void loop() {
  int x;
  int y;
  struct accel_data adat;
  struct mouse_pair mp;
  if(pinched() && dmp_ready){
        adat = read_acc();
        mp = calc_xy(adat);
        send_mouse_event(mp);
  }
  //delay(100);
}

/* accel is in gs and gyro is in degrees/sec */
struct accel_data read_acc(){
  struct accel_data dat;
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;

  fifoCount = mpu.getFIFOCount();
  if(fifoCount == 1024 ){
    mpu.resetFIFO();
    Serial.println("FIFO overflow!");
  } else {
    while( fifoCount < packetSize){fifoCount = mpu.getFIFOCount();} // Wait for correct data length
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    mpu.dmpGetGyro(&gyro, fifoBuffer);
    //Serial.print("A REAL: ");
    //Serial.print(aaWorld.x / (float)VAL_TO_G);
    //Serial.print("\t");
    //Serial.print(aaWorld.y / VAL_TO_G);
    //Serial.print("\t");
    //Serial.println(aaWorld.z/ VAL_TO_G);
    dat.ax = aaWorld.x / (float)VAL_TO_G;
    dat.ay = aaWorld.y / (float)VAL_TO_G;
    dat.az = aaWorld.z / (float)VAL_TO_G;
    /* Gyro has not been tested to be correct */
    dat.gx = gyro.x / (float)VAL_TO_DEG;
    dat.gy = gyro.y / (float)VAL_TO_DEG;
    dat.gz = gyro.z / (float)VAL_TO_DEG;
  }
  //mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  /* Convert raw to meaningful */
  //dat.ax = ax;
  //dat.ay = ay;
  //dat.az = az;
  //dat.gx = gx;
  //dat.gy = gy;
  //dat.gz = gz;
  //Serial.println(dat.az);
  return dat;
}

struct mouse_pair calc_xy(struct accel_data dat){
  //https://forum.arduino.cc/index.php?topic=432979.0
  struct mouse_pair mp;
  mp.x = (int) dat.ax * 10;
  mp.y = (int) dat.az * 10;;
  return mp;
}

// This function assumes th/at the state of the mouse begin and end are handled already
void send_mouse_event(struct mouse_pair mp){
  Mouse.move(mp.x,mp.y);
  return;
}

//bool on_tick(void){

  //return TRUE; //Always be on tick until we get a feel for movement and battery consumption

//}

/* Only returns true if the index finger is pinched */
bool first_pinched(uint8_t fingers){
  #define FIRST_FINGER ( 1 << 0 )
  //Serial.println(fingers & FIRST_FINGER);
  return ( (fingers & FIRST_FINGER) == FIRST_FINGER );
}

uint8_t cap_sensor_read(){
  return (1<<0); //Faked until can actually read the capacitative sensor
}

bool pinched(){
  uint8_t fingers = cap_sensor_read();
  //Serial.print("Fingers: "); Serial.println(fingers);
  return first_pinched(fingers); //&& second_pinched(fingers)
}
