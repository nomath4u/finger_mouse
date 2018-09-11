#include <Kalman.h>

#include <Mouse.h>
#include "I2Cdev.h"
//#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "Adafruit_MPR121.h"
/* IDEAS
 *  Use the IRQ on the capacitive sensor in order to wake up the arduino.
 *  Make sure the bluetooth connection stays alive.
 *  project the filter onto the plane with gravity so roll doesn't matter
 */


/* with default values the accelerometer range is  +- 2g and += 250 deg/second
 *  leaving us with 
 *  16384 per g
 *  131 deg/s
 */
#define VAL_TO_G 16384
#define VAL_TO_DEG 131
#define G_TO_MSS 9.8
#define US_TO_S 1000000
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
void set_offsets(void);
void set_starting_angles(void);
void get_angles(struct accel_data, double*, double*, double);
MPU6050 mpu;
Adafruit_MPR121 cap = Adafruit_MPR121();
Kalman kalmanX;
Kalman kalmanY;

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
bool been_pinched;
uint32_t timer;
void setup() {
  uint8_t dev_status;
  dmp_ready = 0; //Assume failure
  been_pinched = 0;
  Serial.begin(9600);
  
  //pinMode(INDEX_PAD, INPUT);
  cap.begin(0x5A);
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
    set_offsets();
    //Setup interupt once I have it
    dmp_ready = TRUE; //So we can actually use it in the main loop
    packetSize = mpu.dmpGetFIFOPacketSize();
    set_starting_angles();
     
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
  adat = read_acc();
  if(pinched() && dmp_ready){
        //adat = read_acc();
        mp = calc_xy(adat);
        send_mouse_event(mp);
        been_pinched = 1;
  } else { been_pinched = 0; }
  //delay(10);
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

  //int16_t a,b,c,d,e,f;
  #define NUM_LOGS 100
  static struct accel_data log[NUM_LOGS];
                                      
                                      
                                 

  
    /* shift things down */
  for(int i = NUM_LOGS; i >0; i--){
    log[i] = log[i-1];
  }
  
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
    log[0].ax = aaWorld.x / (float)VAL_TO_G;
    log[0].ay = aaWorld.y / (float)VAL_TO_G;
    log[0].az = aaWorld.z / (float)VAL_TO_G;
    /* Gyro has not been tested to be correct */
    log[0].gx = gyro.x / (float)VAL_TO_DEG;
    log[0].gy = gyro.y / (float)VAL_TO_DEG;
    log[0].gz = gyro.z / (float)VAL_TO_DEG;
  }


  
//  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz
//  );
//  /* Convert raw to meaningful */
//  log[0].ax = ax / (float)VAL_TO_G;
//  log[0].ay = ay / (float)VAL_TO_G;
//  log[0].az = az / (float)VAL_TO_G;
//  log[0].gx = gx / (float)VAL_TO_DEG;;
//  log[0].gy = gy / (float)VAL_TO_DEG;
//  log[0].gz = gz / (float)VAL_TO_DEG;
  for(int i = 0; i < NUM_LOGS; i++){
    dat.ax += log[i].ax;
    dat.ay += log[i].ay;
  }
  dat.ax = dat.ax / 5;
  dat.ay = dat.ay / 5;
  return dat;
}

/* This currently only takes int account the previous 2 data sets. This can get better
 *  by creating a FIFO of multiple of these.  
 *  This also assumes that the dmp is filtering for us so we dont need a kalman filter
 */

/*flucuating around alot, try and scale it out*/
struct mouse_pair calc_xy(struct accel_data dat){
  //https://forum.arduino.cc/index.php?topic=432979.0
  https://stackoverflow.com/questions/26476467/calculating-displacement-using-accelerometer-and-gyroscope-mpu6050
  struct mouse_pair mp = {0,0};
 
  int flip = 1;
  static unsigned long t = 0; //We can probably get rid of some of these floats if we are crafty
  double cur_t = 0;
  //float vx = 0;
  unsigned long cur_micros = 0;
  static double u;
  double v;
  double s;
  static double s_cum;
  double x_projection;
  double y_projection;
  double x_projected;
  #define DEAD_SPACE 0
  #define SCALE 30000
  if(!been_pinched){
    /*reset all statics*/
    t = micros();
    u = 0;
    s_cum = 0;
  }
  
  
  cur_t = (double)( micros() - t ) / (US_TO_S); 
  get_angles(dat, &y_projection, &x_projection, cur_t); //pitch seems to be X? So reverse here
  //x_projected = cos(x_projection) * dat.ax;
  v = ((G_TO_MSS )* dat.ax * cur_t) + u;
  s = (v * cur_t) + (0.5 * (G_TO_MSS) * dat.ax * pow(cur_t,2));

  s_cum += s;
    Serial.println(dat.ax,8);
  /* Screen is 1366 x 768 */
  /* x is in meters */
  /* screen is 25.4cm x 16.83cm */
  #define SCALEX 53779 // (1366 / 0.0254)
  #define DEAD_SPACE 0.001
  #define S_ONE_PIXELX ( (float) ( 1 / SCALEX ) )
//  if(abs(s_cum) > S_ONE_PIXELX){
//    if(s_cum < 0){
//      mp.x = -1;
//      s_cum += S_ONE_PIXELX;
//    } else {
//      mp.x = 1;
//      s_cum -= S_ONE_PIXELX;
//    }
//  }

  /*Setup for next time*/
  t = micros(); //Yes there will be some skew because of this
  u = v;

  return mp;
}

// This function assumes th/at the state of the mouse begin and end are handled already
void send_mouse_event(struct mouse_pair mp){
  //Serial.println(mp.x);
  Mouse.move(mp.x,mp.y);
  return;
}

/* Only returns true if the index finger is pinched */
bool first_pinched(uint8_t fingers){
  #define FIRST_FINGER ( 1 << 1 )
  //Serial.println(fingers & FIRST_FINGER);
  return ( (fingers & FIRST_FINGER) == FIRST_FINGER );
}

uint8_t cap_sensor_read(){
  //return (1<<0); //Faked until can actually read the capacitative sensor
  //Serial.println(cap.touched());
  return cap.touched();
}

bool pinched(){
  uint8_t fingers = cap_sensor_read();
  //Serial.print("Fingers: "); Serial.println(fingers);
  return first_pinched(fingers); //&& second_pinched(fingers)
}

/* Set offsets gotten from running calibration MPU6050 sketch */
void set_offsets(){
  mpu.setXAccelOffset(-1357);
  mpu.setYAccelOffset(-3639);
  mpu.setZAccelOffset(1019);
  mpu.setXGyroOffset(-45);
  mpu.setYGyroOffset(62);
  mpu.setZGyroOffset(-47);
}

void ComplementaryFilter(struct accel_data d, double* filt_x, double* filt_y, double dt)
{   

    #define M_PI 3.14159265359
    float xAcc;
    float yAcc;
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    //*filt_x += (d.gx * dt); // Angle around the X-axis
    //*filt_y -= (d.gy * dt);    // Angle around the Y-axis
 
    // Compensate for drift with accelerometer data if !bullshit
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192

  // Turning around the X axis results in a vector on the Y-axis
        //xAcc = atan2f(d.ay, d.az) * 180 / M_PI;
        *filt_x = *filt_x * 0.98 + d.ax * 0.02;
 
  // Turning around the Y axis results in a vector on the X-axis
        //yAcc = atan2f(d.ax, d.az) * 180 / M_PI;
        *filt_y = *filt_y * 0.98 + d.ay * 0.02;
} 

void set_starting_angles(){
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  double d_ax, d_ay, d_az, d_gx, d_gy, d_gz;


  d_ax = (double)ax;
  d_ay = (double)ay;
  d_az = (double)az;
  d_gx = (double)gx;
  d_gy = (double)gy;
  d_gz = (double)gz;

    #ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(d_ay, d_az) * RAD_TO_DEG;
  double pitch = atan(-d_ax / sqrt(d_ay * d_ay + d_az * d_az) * RAD_TO_DEG);
#else // Eq. 28 and 29
  double roll  = atan(d_ay / sqrt(d_ax * d_ax + d_az * d_az) * RAD_TO_DEG);
  double pitch = atan2(-d_ax, d_az) * RAD_TO_DEG;
#endif
  //Serial.println(roll);
  kalmanX.setAngle(roll);
  kalmanY.setAngle(pitch);
}

void get_angles( struct accel_data dat, double* froll, double* fpitch, double dt ){
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
  double d_ax, d_ay, d_az, d_gx, d_gy, d_gz;
  static double prev_froll = 0;
  static double prev_fpitch = 0;


  d_ax = dat.ax;
  d_ay = dat.ay;
  d_az = dat.az;
  d_gx = dat.gx;
  d_gy = dat.gy;
  d_gz = dat.gz;

// Eq. 28 and 29
  double roll  = atan(d_ay / sqrt(d_ax * d_ax + d_az * d_az)) * RAD_TO_DEG;
  double pitch = atan2(-d_ax, d_az) * RAD_TO_DEG;

  double gyroXrate = d_gx / 131.0; // Convert to deg/s
  double gyroYrate = d_gy / 131.0; // Convert to deg/s

  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && prev_fpitch > 90) || (pitch > 90 && prev_fpitch < -90)) {
    kalmanY.setAngle(pitch);
    *fpitch = pitch;
  } else
    *fpitch = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(*fpitch) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  *froll = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  prev_froll = *froll;
  prev_fpitch = *fpitch;
}

