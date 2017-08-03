#include <Keypad.h>
#include <Servo.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <Wire.h>
#include <stdlib.h>
#include <LiquidCrystal_I2C.h>




//this defines a bunch of variables for the MPU
#define AK8963_ADDRESS   0x0C
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L   0x03  // data
#define AK8963_XOUT_H  0x04
#define AK8963_YOUT_L  0x05
#define AK8963_YOUT_H  0x06
#define AK8963_ZOUT_L  0x07
#define AK8963_ZOUT_H  0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02
#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E
#define WOM_THR          0x1F

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

#define ADO 1
#if ADO
#define MPU9250_ADDRESS 0x68  // Device address when ADO = 1
#else
#define MPU9250_ADDRESS 0x68  // Device address when ADO = 0
#define AK8963_ADDRESS 0x0C   //  Address of magnetometer
#endif

#define AHRS true         // set to false for basic data read
#define SerialDebug false   // set to true to get Serial output for debugging




enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x02;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed = 13; // Set up pin 13 led for toggling

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer
int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float   SelfTest[6];    // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0, sumCount = 0; // used to control display output rate
//float pitch, yaw, roll;
float deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

float magBias[3], magScale[3];
float pitch, yaw, roll;


















//making all of the objects that I'll need later
TinyGPSPlus gps;
TinyGPSDate dat;
TinyGPSTime tim;
TinyGPSLocation loc;
Servo bpan;
Servo tpan;
Servo tilt;
LiquidCrystal_I2C lcd(0x3F,16,2);


//all of my global variables
double latitude, longitude, Time, hours, minutes, seconds;
int Day, Month, Year;
int printLatLong = 10; // set this to less than five to print the coordinates
double daynow, d;
int planet = -1;
byte printStuf = 0; //set to one in order to print planet stuff
int counter;
double rlpitch, rlroll, rp, rr;
double pitchOffset, rollOffset;

//Keypad Stuff
const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

byte rowPins[ROWS] = {3, 4, 5, 6}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {7, 8, 9, 10}; //connect to the column pinouts of the keypad

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );


byte mpuOrNo; //checks if there's an MPU signal

//variables needed for password stuff
char newkey[3];
byte currentlength;
char codeEnterred[3];
byte validcode; // this is zero, unless a valid code is enterred, then it becomes one
byte codelen;

//time password stuff
   char nkey[6];
   char ce[6];
   byte vcode;
   byte clen;

//same but for date  
   char nkeyy[8];
   char cee[8];
   byte vcodee;
   byte clenn; 

 //planets
    char nke[1];
   char c[1];
   byte vcod;
   byte cle;   
   
int ff = 0; //one is fastforward
byte stabilize = 1; //set to one in order to activate stabilization
byte curTime = 1; //set to one if using current time
byte curDate = 1; //set to one if using current date;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(9600);
  bpan.attach(11);
  tpan.attach(12);
  tilt.attach(13);
  lcd.init();                      // initialize the lcd 
  // Print a message to the LCD.
  lcd.clear();
  lcd.backlight();
  lcd.print("Initializing");
  delay(1000);
  currentlength = 1;
  clen = 1;
  pinMode(43, OUTPUT);
  pinMode(53, OUTPUT);
  pinMode(52, OUTPUT);


if (gps.time.isValid()) {
    hours = gps.time.hour();
    minutes = gps.time.minute();
    seconds = gps.time.second();
    Time = hours + (minutes / 60) + (seconds / 3600);
    digitalWrite(52, HIGH);
  }
  else {
    //Serial.println("TIME INVAlID");
    hours = 0;
    minutes = 0;
    seconds = 0;
    digitalWrite(52, LOW);
  }
  if (gps.date.isValid()) {
    Day = gps.date.day();
    Month = gps.date.month();
    Year = gps.date.year();
  }
  else {
    Day = 2;
    Month = 8;
    Year = 2017;
  }

  Wire.begin();
  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    mpuOrNo = 1;
    Serial.println("MPU9250 is online...");

    MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values

    calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers

    initMPU9250();

    byte d = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    // Read WHO_AM_I register for AK8963
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
    delay(1000);
    initAK8963(magCalibration);
    ; Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer
    getMres();
    // magcalMPU9250(magBias, magScale);


  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    mpuOrNo = 0;
    //while (1) ; // Loop forever if communication doesn't happen
  }


  smartDelay(1);
  counter = 1;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Please make");
  lcd.setCursor(0,1);
  lcd.print("a selection");
}

void loop() {
    // put your main code here, to run repeatedly:
  int cp = planet;
    //potentiometer stuff
  
  //smartDelay(1000);
  MPUloop();
  if (gps.location.isValid()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
  }
  else {
    latitude = 40.8721;
    longitude = -73.9694;
  }
 if(curTime == 1){
  if (gps.time.isValid()) {
    hours = gps.time.hour();
    minutes = gps.time.minute();
    seconds = gps.time.second();
    Time = hours + (minutes / 60) + (seconds / 3600);
  }
  else {
    //Serial.println("TIME INVAlID");
    hours = 16;
    minutes = 15;
  }}
  if(curDate == 1){
  if (gps.date.isValid()) {
    Day = gps.date.day();
    Month = gps.date.month();
    Year = gps.date.year();
  }
  else {
    Day = 27;
    Month = 7;
    Year = 2017;
  }}
 
  if (printLatLong < 5)
  {
    printLatLong ++;
    Serial.println(longitude, 17);
    Serial.println(Day);
    Serial.println(Month);
    Serial.println(Year);
  }

  getCode(); //select the planet from the enterred code
  daynow = dayNow(Year, Month, Day);
  d = daynow - 4975.5;
  if(cp != planet){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Now pointing at:");
  lcd.setCursor(0, 1);
    switch (planet) {
      case 1:
        lcd.print("Mercury");
        break;
      case 2:
        lcd.print("Venus");
        break;
      case 3:
        lcd.println("Earth. Look down");
        tilt.write(0);
        break;
      case 4:
        lcd.print("Mars");
        break;
      case 5:
        lcd.print("Jupiter");
        break;
      case 6:
        lcd.print("Saturn");
        break;
      case 7:
        lcd.print("Uranus");
        break;
      case 8:
        lcd.print("Neptune");
        break;
      case 9:
        lcd.print("Pluto");
        break;
      case 10:
        lcd.print("Andromeda");
        break;


    }
  }

//average pitch and roll every five readings

  double pitchAr[5], rollAr[5];
  pitchAr[counter-1] = pitch;
  rollAr[counter-1] = roll;
  if(counter == 5){
    counter = 1;
    double pitchsum, rollsum;
    for(int xx = 0; xx<5; xx++){
      pitchsum+=pitchAr[xx];
      rollsum+=rollAr[xx];      
      }
    rlpitch = 0-(pitchsum/5);
    rlroll = 0-(rollsum/5);
    rlpitch += pitchOffset;
    rlroll += rollOffset;
    
    }
   rp = rlpitch;
   rr = rlroll; 
  //get alt/az values and write them to the servos
  if (planet != 3) {
    int ALT = Alt(planet);
    int AZ = Az(planet);
    if(stabilize == 0){
    altServ(ALT);
    azServ(AZ);}
    else if (stabilize == 1){
    adjustServos(ALT, AZ, rlpitch, rlroll);
    Serial.print("pitch");
    Serial.println(rlpitch);
    Serial.println(rlroll);
    }
  }
  else {
    tilt.write(0);

  }

 if(ff == 1){
  Time += 0.1;
  
  }
counter++;

 smartDelay(10);
}
//to fix the radians issue
double pi = 4 * atan(1);
double rad = pi / 180;
/*Below are the orbital elements for all of the planets.
  In order, mercury is 1, venus is 2, the Earth-Moon Barycenter is 3, Mars is 4, Jupiter is 5, Saturn is 6, Uranus is 7, Neptune is 8, and Pluto is 9.  */
double i[10] = {0.0, 7.0052, 3.3949, 0.0, 1.8496, 1.3033, 2.4869, 0.7728, 1.7692, 17.1695}; //inclination
double o[10] = {0.0, 48.493, 76.804, 0.0, 49.668, 100.629, 113.732, 73.989, 131.946, 110.469};//long of ascending node
double p[10] = {0.0, 77.669, 131.99, 103.147, 336.322, 14.556, 91.500,  169.602, 6.152, 223.486};//long of perihelion
double a[10] = {0.0, 0.387098, 0.723327, 1.0000, 1.523762, 5.20245, 9.52450, 19.1882, 29.9987, 39.2766};//mean distance in AU
double n[10] = {0.0, 4.09235, 1.60215, 0.985611, 0.523998, 0.083099, 0.033551, 0.011733, 0.006002, 0.004006}; // daily motion
double e[10] = {0.0, 0.205645 , 0.006769, 0.016679, 0.093346, 0.048892, 0.055724, 0.047874, 0.009816, 0.246211}; // eccentricity of orbit
double L[10] = {0.0, 93.8725, 233.5729, 324.5489, 82.9625, 87.9728, 216.6279, 11.9756, 335.0233, 258.8717}; //mean longitude
//these are the empty arrays, we'll calculate values for them as we go.
//double M[10], v[10], r[10], x[10], y[10], z[10], Xi[10], Yi[10], Zi[10], Xq[10], Yq[10], Zq[10], ra[10], dec[10];
/*for the date and location, we're going to use may 15 2017, at my house (40.8721, -73.9694), at 0:00 UT as our temporary placeholder.
  Once the GPS is connected, we'll add code that allows this to work for any date, time, and location as given by the GPS*/

//list of values for the constellations. 
double constRa[11] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8076667};
double constDec[11] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 37.431833333};


//calculates right ascension

double RA(double index) {
  double alpha;
  if(index <= 9) {
  //now we calculate the Mean Anomaly
  double M = MeAn(index);
  //true anomaly
  double v = TrAn(index);
  //radius vector
  double r = RaVe(index);
  //heliocentric coordinates
  double X = Xhel(index);
  double Y = Yhel(index);
  double Z = Zhel(index);
  //converting these to geocentric coordinates
  double Xg = X - (RaVe(3) * cos(TrAn(3) + rad * p[3]));
  double Yg = Y - (RaVe(3) * sin(TrAn(3) + rad * p[3]));
  double Zg = Z;
  //now turn these into geocentric equatorial coordinates
  double Xeq = Xg;
  double Yeq = Yg * cos(rad * 23.439292) - Zg * sin(rad * 23.439292);
  double Zeq = Yg * sin(rad * 23.439292) + Zg * cos(rad * 23.439292);
  //convert these into RA
  /*Serial.print(Xeq);
    Serial.print("\n");
    Serial.print(Yeq);
    Serial.print("\n");
    Serial.print(Zeq);
    Serial.print("\n");*/
  alpha = atan(Yeq / Xeq) / rad;
  if (Xeq < 0) {
    alpha += 180;
  }
  else if (Yeq < 0) {
    alpha += 360;
  }
  alpha = alpha / 15;}
  else if(index > 9) {
    int in = index;
    alpha = constRa[in];
    
    }

  
  return alpha;
  //find the string of the planet from the index and print accordingly
  int pnum = index;
  char printOrNo = 0;
  if (printOrNo == 1) {
    switch (pnum) {
      case 1:
        Serial.print("\n");
        Serial.print("The right ascension of Mercury is ");
        Serial.println(alpha, 10);
        break;
      case 2:
        Serial.print("\n");
        Serial.print("The right ascension of Venus is ");
        Serial.println(alpha, 10);
        break;
      case 4:
        Serial.print("\n");
        Serial.print("The right ascension of Mars is ");
        Serial.println(alpha, 10);
        break;
      case 5:
        Serial.print("\n");
        Serial.print("The right ascension of Jupiter is ");
        Serial.println(alpha, 10);
        break;
      case 6:
        Serial.print("\n");
        Serial.print("The right ascension of Saturn is ");
        Serial.println(alpha, 10);
        break;
      case 7:
        Serial.print("\n");
        Serial.print("The right ascension of Uranus is " );
        Serial.println(alpha, 10);
        break;
      case 8:
        Serial.print("\n");
        Serial.print("The right ascension of Neptune is ");
        Serial.println(alpha, 10);
        break;
      case 9:
        Serial.print("\n");
        Serial.print("The right ascension of Pluto is ");
        Serial.println(alpha, 10);
        break;
      default :
        Serial.print("\n");
        Serial.print("look down, moron");
        Serial.print("\n");
    }
  }
}

//calculates declination, the same as the RA calculations except at the end
double Dec(int index) {
  double delta;
  if(index <= 9){
  //now we calculate the Mean Anomaly
  double M = MeAn(index);
  //true anomaly
  double v = TrAn(index);
  //radius vector
  double r = RaVe(index);
  //heliocentric coordinates
  double X = Xhel(index);
  double Y = Yhel(index);
  double Z = Zhel(index);
  //converting these to geocentric coordinates
  double Xg = X - (RaVe(3) * cos(TrAn(3) + rad * p[3]));
  double Yg = Y - (RaVe(3) * sin(TrAn(3) + rad * p[3]));
  double Zg = Z;
  //now turn these into geocentric equatorial coordinates
  double Xeq = Xg;
  double Yeq = Yg * cos(rad * 23.439292) - Zg * sin(rad * 23.439292);
  double Zeq = Yg * sin(rad * 23.439292) + Zg * cos(rad * 23.439292);
  //use the above values to find declination
  double thing = Xeq * Xeq + Yeq * Yeq;
  double denom = sqrt(thing);
  double inp = Zeq / denom;
  delta = arctangent(inp);
  }
  else if(index > 9) {
    int in = index;
    delta = constDec[in];
    
    }
  return delta;
  //find the string of the planet from the index and print accordingly
  int pnum = index;
  //make the below variable one if you want to print
  char printOrNo = 0;
  if (printOrNo == 1) {
    switch (pnum) {
      case 1:
        Serial.print("\n");
        Serial.print("The declination of Mercury is ");
        Serial.println(delta, 10);
        break;
      case 2:
        Serial.print("\n");
        Serial.print("The declination of Venus is ");
        Serial.println(delta, 10);
        break;
      case 4:
        Serial.print("\n");
        Serial.print("The declination of Mars is ");
        Serial.println(delta, 10);
        break;
      case 5:
        Serial.print("\n");
        Serial.print("The declination of Jupiter is ");
        Serial.println(delta, 10);
        break;
      case 6:
        Serial.print("\n");
        Serial.print("The declination of Saturn is ");
        Serial.println(delta, 10);
        break;
      case 7:
        Serial.print("\n");
        Serial.print("The declination of Uranus is ");
        Serial.println(delta, 10);
        break;
      case 8:
        Serial.print("\n");
        Serial.print("The declination of Neptune is ");
        Serial.println(delta, 10);
        break;
      case 9:
        Serial.print("\n");
        Serial.print("The declination of Pluto is ");
        Serial.println(delta, 10);
        break;
      default :
        Serial.print("\n");
        Serial.print("look down, moron");
        Serial.print("\n");
    }
  }
}

//calculates altitude from RA and DEC
double Alt(int index) {
  double ha = HA(index);
  //  Serial.print("ha is");
  //  Serial.println(ha);
  double dec = Dec(index);
  //  Serial.println(dec);
  double x, y, z, xhor, yhor, zhor;
  x = cos(ha * rad) * cos(dec * rad);
  y = sin(ha * rad) * cos(dec * rad);
  z = sin(dec * rad);
  xhor = x * cos((90 - latitude) * rad) - z * sin((90 - latitude) * rad);
  yhor = y;
  zhor = x * sin((90 - latitude) * rad) + z * cos((90 - latitude) * rad);
  double alt = asin(zhor) * (180 / pi);
  if (printStuf == 1) {
    Serial.print("  the altitude is ");
    Serial.print(alt, 7);
    Serial.print("   ");
  }
  return alt;
}

//Azimuth
double Az(int index) {
  double ha = HA(index);
  //  Serial.print("ha is");
  //  Serial.println(ha);
  double dec = Dec(index);
  //  Serial.println(dec);
  double x, y, z, xhor, yhor, zhor;
  x = cos(ha * rad) * cos(dec * rad);
  y = sin(ha * rad) * cos(dec * rad);
  z = sin(dec * rad);
  xhor = x * cos((90 - latitude) * rad) - z * sin((90 - latitude) * rad);
  yhor = y;
  zhor = x * sin((90 - latitude) * rad) + z * cos((90 - latitude) * rad);
  double az = atan2(yhor, xhor) * (180 / pi) + 180;
  if (printStuf == 1) {
    Serial.print("the azimuth is ");
    Serial.println(az, 7);
  }
  return az;
}




//Accounts for rotation and adjusts servos accordingly
void adjustServos(int altAngle, int azAngle, int pitch, int roll) {
  double Xi = calcX(altAngle, azAngle);
  double Yi = calcY(altAngle, azAngle);
  double Zint = calcZ(altAngle, azAngle);
  double Xint = (Xi * cos(rad * pitch)) - (Yi * sin(rad * pitch));
  double Yint = (Xi * sin(rad * pitch)) + (Yi * cos(rad * pitch));
  double thetAltInter = asin(Yint) / rad;
  double thet = asin(abs((Xint)/(cos(thetAltInter*rad)))) / rad;
  double thetAzInter;
  if(Xi > 0 && Zint >0){
    thetAzInter = 90 - thet;
    }
  if(Xi < 0 && Zint >0){
    thetAzInter = 90 + thet;
    }
  if(Xi > 0 && Zint <0){
    thetAzInter = 270 + thet;
    }
  if(Xi < 0 && Zint <0){
    thetAzInter = 270 - thet;
    }      
  double Zfin = (Zint * cos(rad * roll)) - (Yint * sin(rad * roll));
  double Yfin = (Zint * sin(rad * roll)) + (Yint * cos(rad * roll));
  double thetAltF = asin(Yfin) / rad;
  double thetAzF;
  double theter = acos(abs(Zfin / cos(rad * thetAltF))) / rad;
  if(Xint > 0 && Zfin >0){
    thetAzF = 90 - theter;
    }
  if(Xint < 0 && Zfin >0){
    thetAzF = 90 + theter;
    }
  if(Xint > 0 && Zfin <0){
    thetAzF = 270 + theter;
    }
  if(Xint < 0 && Zfin <0){
    thetAzF = 270 - theter;
    } 
  if (thetAzF < 0) {
    thetAzF = 360 + thetAzF;
  }
  Serial.print("Alt ");
  Serial.println( thetAltF, 4);
  Serial.print("Az ");
  Serial.println( thetAzF, 4);
//    Serial.print("                                  DONE         DONE         DONE");
//    Serial.println(secs);
altServ(thetAltF);
azServ(thetAzF);
if(abs(pitch) > 5 or abs(roll)>5){
  digitalWrite(53, HIGH);
  }
else{
  digitalWrite(53, LOW);  
}

}

double calcX(int altAngle, int azAngle) {
  double X = cos(rad * altAngle) * cos(rad * azAngle);
  return X;
}

double calcY(int altAngle, int azAngle) {
  double Y = sin(rad * altAngle);
  return Y;

}

double calcZ(int altAngle, int azAngle) {
  double Z = cos(rad * altAngle) * sin(rad * azAngle);
  return Z;
}







//mean anomaly
double MeAn(int index) {
  double M = n[index] * d + L[index] - p[index];
  M = rad * M;
  //Serial.print("\n");
  //Serial.print(M);
  return M;
}
//true anomaly
double TrAn(int index) {
  double v = MeAn(index) + (2 * e[index] - 0.25 * pow(e[index], 3) + 5 / 96 * pow(e[index], 5)) * sin(MeAn(index)) +
             (1.25 * pow(e[index], 2) - 11 / 24 * pow(e[index], 4)) * sin(2 * MeAn(index)) +
             (13 / 12 * pow(e[index], 3) - 43 / 64 * pow(e[index], 5)) * sin(3 * MeAn(index)) +
             103 / 96 * pow(e[index], 4) * sin(4 * MeAn(index)) + 1097 / 960 * pow(e[index], 5) * sin(5 * MeAn(index));
  //Serial.print(v);
  //Serial.print("\n");
  //Serial.print(e[index], DEC);
  return v;
}
//radius vector
double RaVe(int index) {
  double r = a[index] * (1 - (e[index] * e[index])) / (1 + e[index] * cos(TrAn(index)));
  /*Serial.print("\n");
    Serial.print(r);
    Serial.print("\n");
    Serial.print(e[index]);
    Serial.print("\n");
    Serial.print(TrAn(index));*/
  return r;
}
//heliocentric coordinates
double Xhel(int index) {
  double X = RaVe(index) * (cos(rad * o[index]) * cos(TrAn(index) + rad * p[index] - rad * o[index]) - sin(rad * o[index]) * sin(TrAn(index) + rad * p[index] - rad * o[index]) *
                            cos(rad * i[index]));
  return X;
}
double Yhel(int index) {
  double Y = RaVe(index) * (sin(rad * o[index]) * cos(TrAn(index) + rad * p[index] - rad * o[index]) + cos(rad * o[index]) * sin(TrAn(index) + rad * p[index] - rad * o[index]) *
                            cos(rad * i[index]));
  return Y;
}
double Zhel(int index) {
  double Z = RaVe(index) * (sin(TrAn(index) + rad * p[index] - rad * o[index]) * sin(rad * i[index]));
  return Z;
}

//local standard time
double LST(int index) {
  double Lst = mod((100.46 + (0.985647 * daynow) + longitude + 15 * Time), 360.0);
  //  double number = 100.46 + (0.985647 * daynow) + longitude + 15*Time;
  //  Serial.println(longitude);
  //  Serial.println(number);
  //  Serial.print("LST is ");
  //  Serial.println(Lst);
  return Lst;
}
//hour angle
double HA(int index) {
  double ha = LST(index) - 15 * RA(index);
  if (ha < 0) {
    ha += 360;
  }
  if (ha > 360) {
    ha -= 360;
  }
  return ha;
}
//The given arctan function is being a pain in the butt so I'm going to write my own
double arctangent( double input) {
  double sum = 0;
  for (int n = 0; n < 20; n++) {
    long double nextTerm = (pow(-1, n)) * (pow(input, 2 * n + 1) / (2 * n + 1));
    sum = sum + nextTerm;
  }
  sum = sum / rad;
  //Serial.print(sum);
  //Serial.print("\n");
  return sum;
}

//my own modulo, returns x%y
double mod(double x, double y) {
  double m = (x / y) - floor(x / y);
  double modulo = m * y;
  return modulo;
}


double dayNow(int Year, int Month, int Day) {
  //first, take the days up to this year from J2000 and the days of all the previous months in the year
  double daysToYear;
  int daysToMonth;
  //ADD MORE YEARS HERE IN THE FUTURE, FOR NOW THIS'LL ONLY GO UP UNTIL 2020
  switch (Year) {
    case 2017:
      daysToYear = 6208.5;
      break;
    case 2018:
      daysToYear = 6573.5;
      break;
    case 2019:
      daysToYear = 6938.5;
      break;
    case 2020:
      daysToYear = 7303.5;
      break;

  }
  switch (Month) {
    case 1:
      daysToMonth = 0 ;
      break;
    case 2:
      daysToMonth = 31 ;
      break;
    case 3:
      daysToMonth =  59;
      break;
    case 4:
      daysToMonth = 90 ;
      break;
    case 5:
      daysToMonth = 120 ;
      break;
    case 6:
      daysToMonth = 151 ;
      break;
    case 7:
      daysToMonth = 181 ;
      break;
    case 8:
      daysToMonth = 212 ;
      break;
    case 9:
      daysToMonth = 243 ;
      break;
    case 10:
      daysToMonth = 273 ;
      break;
    case 11:
      daysToMonth = 304 ;
      break;
    case 12:
      daysToMonth = 334 ;
      break;

  }
  if (mod(Year, 4) == 0 && Month > 2) {
    daysToMonth += 1;

  }
  double totDays = daysToYear + daysToMonth + Day;
  //Serial.println(totDays);
  return totDays;
}
//take pot value and return which planet
//I GOT RID OF THE POTENTIOMETER SO THIS CODE IS NOT USED
int planetSelection(int pv) {
  int p;
  if (pv < 20 ) {
    p = 1;
  }
  if (pv > 20 && pv < 100) {
    p = 2;
  }
  if (pv > 101 && pv < 240) {
    p = 3;
  }
  if (pv > 241 && pv < 390) {
    p = 4;
  }
  if (pv > 391 && pv < 550) {
    p = 5;
  }
  if (pv > 551 && pv < 735) {
    p = 6;
  }
  if (pv > 736 && pv < 900) {
    p = 7;
  }
  if (pv > 901 && pv < 1000) {
    p = 8;
  }
  if (pv > 1000) {
    p = 9;
  }
  return p;


}
//control the altitude servo
void altServ(int alti) {
  int altVal = 90 + alti;
  tilt.write(altVal);
}
//azimuth servo
int azServ(int azim) {
  if(azim< 0){
    azim = 360 - azim;
    }
  if(azim > 360){
    azim = azim - 360;
    }  
  if (azim <= 180) {
    int t = 180 - azim;
    tpan.write(t);
    bpan.write(0);
  }
  if (azim > 180) {
    tpan.write(0);
    int b = azim - 180;
    bpan.write(b);
  }
  
}
//Gets the GPS data
static void smartDelay(unsigned long ms)
{
  //Serial.println("C");
  unsigned long start = millis();
  do
  {
    while (Serial2.available())
      gps.encode(Serial2.read());
  } while (millis() - start < ms);
  //Serial.println("D");
}


//keypad stuff
void getCode(){
  char key = keypad.getKey();
   if(key){
    if(key == '*'){
      currentlength = 1;
      codelen = 3;
    }
    if(key == 'A'){
      currentlength = 1;
      codelen = 1;
    }
     if(key == 'C'){
        clen = 1;
        codelen = 6;
      }
     if(key == 'D'){
      clenn = 1;
      codelen = 8; 
     }

     if(codelen == 1){
      if(key!= 'A'){
      nke[cle-1] = {key};
      if(cle == 1){
        strcpy(c,nke);
        vcod = 1;
        cle = 0;   
        }
      else{
        vcod = 0;
        }  
      cle+=1;
      }
    }
    if(codelen == 3){
    if(key!= '*'){
    newkey[currentlength-1] = {key};
    if(currentlength == 3){
      strcpy(codeEnterred, newkey);
      validcode = 1;
      currentlength = 0;   
      }
    else{
      validcode = 0;
      }  
    currentlength+=1;
    }}

    if(codelen == 6){
      if(key!= 'C'){
      nkey[clen-1] = {key};
      if(clen == 6){
        strcpy(ce,nkey);
        vcode = 1;
        clen = 0;   
        }
      else{
        vcode = 0;
        }  
      clen+=1;
      }
    }

    if(codelen ==8){
      if(key!= 'D'){
      nkeyy[clenn-1] = {key};
      if(clenn == 8){
        strcpy(cee,nkeyy);
        vcodee = 1;
        clenn = 0;   
        }
      else{
        vcodee = 0;
        }  
      clenn+=1;
      }
    }
   }
   if(validcode == 1){
    validcode =0;
   int code = atoi(codeEnterred);
    switch(code){
      case 101:
        planet = 1;
        break;
      case 102:
        planet = 2;
        break;
      case 103:
        planet = 3;
        break; 
      case 104:
        planet = 4;
        break;
      case 105:
        planet = 5;
        break;
      case 106:
        planet = 6;
        break; 
      case 107:
        planet = 7;
        break;
      case 108:
        planet = 8;
        break;
      case 109:
        planet = 9;
        break;
      case 201:
        planet = 10;
        break;
      case 001:
        stabilize = 1;
        lcd.clear();
        lcd.print("Stabilization:");
        lcd.setCursor(0,1);
        lcd.print("on");
        break;
      case 000:
        stabilize = 0;
        lcd.clear();
        lcd.print("Stabilization:");
        lcd.setCursor(0,1);
        lcd.print("off");
        break;
      case 002:
        curTime = 0;
        lcd.clear();
        lcd.print("Custom Time ");
       lcd.setCursor(12,0);
       lcd.print(Time);
        lcd.setCursor(0,1);
        lcd.print("Enter C + HHMMSS");
        break;
      case 003:
        curTime = 1;
        break;
      case 004:
        curDate = 0;
        lcd.clear();
        lcd.print("CustDate");
        lcd.setCursor(8,0);
        lcd.print(Month);
        lcd.print(Day);
        lcd.print(Year);
        lcd.setCursor(0,1);
        lcd.print("Enter D+MMDDYYYY");
        break;
      case 005:
        curDate = 1;
        break;
      case 006:
        dispDT();
        break;
      case 007:
        whatsUp();
        break;
      case 901:
        lcd.clear();
        lcd.print("Place on a");
        lcd.setCursor(0,1);
        lcd.print("level surface");
        delay(1000);
        calibPitchRoll();
        lcd.clear();
        lcd.print("Calibrating...");
        delay(500);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Please make");
        lcd.setCursor(0,1);
        lcd.print("a selection");
        
        
        break;   
      
      }}
   if(vcod == 1){
    vcod = 0;
     int code = atoi(c);
     switch(code){
      case 1:
        planet = 1;
        break;
      case 2:
        planet = 2;
        break;
      case 3:
        planet = 3;
        break; 
      case 4:
        planet = 4;
        break;
      case 5:
        planet = 5;
        break;
      case 6:
        planet = 6;
        break; 
      case 7:
        planet = 7;
        break;
      case 8:
        planet = 8;
        break;
      case 9:
        planet = 9;
        break;
    }}
   if(vcode == 1){
    vcode = 0;
    float code = atof(ce);
    changeTime(code);
    }
   if(vcodee == 1){
    vcodee = 0;
    float code = atof(cee);
    changeDate(code); 
   }
  
  
  }


void changeTime(float code){
  double s = code - 100*floor(code/100);
  double h = floor(code/10000);
  double m = floor(code/100) - 100*h;
  seconds = s;
  minutes = m;
  hours = h;
  Time = h + (m/60) + (s/3600);
  //Serial.print("code:  ");
  //Serial.println(code);
  Serial.println(Time);
}
void changeDate(float code){
  Year = code - 10000*floor(code/10000);
  Month = floor(code/1000000);
  Day = floor(code/10000) - 100*Month;
  Serial.println(Year);
  Serial.println(Month);
  Serial.println(Day);
}

void dispDT(){
  int h = hours - 4;
    if(h<0){
    h+=24;
  }
  int m = minutes;
  int s = seconds;
  lcd.clear();
  lcd.print(Month);
  lcd.print("/");
  lcd.print(Day);
  lcd.print("/");
  lcd.print(Year);
  lcd.setCursor(0, 1);
  if( h != 0){lcd.print(h);}
  else{lcd.print("00");}
  lcd.print(":");
  if( m != 0){lcd.print(m);}
  else{lcd.print("00");}
  lcd.print(":");
  if( s != 0){lcd.print(s);}
  else{lcd.print("00");}
  
  }

void whatsUp(){
  int c = 0;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("What's in the");
  lcd.setCursor(0,1);
  lcd.print("sky right now");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0,0);
  for(int ind = 0; ind <=9; ind++){
     if(Alt(ind) > 0){
      printPlanet(ind);
      lcd.print(", ");
      c++;
      }
      if(c >=3){
        lcd.setCursor(0,1);
        }  
  }
  
  }  

void printPlanet(int num){
  switch(num){
      case 1:
        lcd.print("Mer");
        break;
      case 2:
        lcd.print("Ven");
        break;
      case 3:
        lcd.print("");
        break;
      case 4:
        lcd.print("Mars");
        break;
      case 5:
        lcd.print("Jup");
        break;
      case 6:
        lcd.print("Sat");
        break;
      case 7:
        lcd.print("Ura");
        break;
      case 8:
        lcd.print("Nep");
        break;
      case 9:
        lcd.print("Plu");
        break;
  
  }}

void laser( int t){

        digitalWrite(43, HIGH);
        delay(t);
        digitalWrite(43, LOW);
}


void calibPitchRoll(){
  pitchOffset = -rlpitch;
  rollOffset = -rlroll;
  
  }





















//MPU STUFF

void MPUloop()
{
  //Serial.println("A");
  // If intPin goes high, all data registers have new data
  if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
    readAccelData(accelCount);  // Read the x/y/z adc values
    getAres();

    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0] * aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1] * aRes; // - accelBias[1];
    az = (float)accelCount[2] * aRes; // - accelBias[2];

    readGyroData(gyroCount);  // Read the x/y/z adc values
    getGres();

    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0] * gRes; // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1] * gRes;
    gz = (float)gyroCount[2] * gRes;

    readMagData(magCount);  // Read the x/y/z adc values
    getMres();
    //    magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
    //    magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
    //    magbias[2] = +125.;  // User environmental x-axis correction in milliGauss

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    //           mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
    //           my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];
    //           mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];
    mx = (float)magCount[0] * mRes * magCalibration[0] + 451.95; // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1] * mRes * magCalibration[1] + 550.21;
    mz = (float)magCount[2] * mRes * magCalibration[2] + 451.95;
  }

  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
  // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!
  // Pass gyro rate as rad/s
  MadgwickQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f,  my,  mx, mz);
  //MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);


  if (!AHRS) {
    delt_t = millis() - count;
    if (delt_t > 500) {

      if (SerialDebug) {
        // Print acceleration values in milligs!
        //    Serial.print("X-acceleration: "); Serial.print(1000*ax); Serial.print(" mg ");
        //    Serial.print("Y-acceleration: "); Serial.print(1000*ay); Serial.print(" mg ");
        //    Serial.print("Z-acceleration: "); Serial.print(1000*az); Serial.println(" mg ");
        //
        //    // Print gyro values in degree/sec
        //    Serial.print("X-gyro rate: "); Serial.print(gx, 3); Serial.print(" degrees/sec ");
        //    Serial.print("Y-gyro rate: "); Serial.print(gy, 3); Serial.print(" degrees/sec ");
        //    Serial.print("Z-gyro rate: "); Serial.print(gz, 3); Serial.println(" degrees/sec");
        //
        //    // Print mag values in degree/sec
        //    Serial.print("X-mag field: "); Serial.print(mx); Serial.print(" mG ");
        //    Serial.print("Y-mag field: "); Serial.print(my); Serial.print(" mG ");
        //    Serial.print("Z-mag field: "); Serial.print(mz); Serial.println(" mG");

        tempCount = readTempData();  // Read the adc values
        temperature = ((float) tempCount) / 333.87 + 21.0; // Temperature in degrees Centigrade
        // Print temperature in degrees Centigrade
        Serial.print("Temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
      }

      //    display.clearDisplay();
      //    display.setCursor(0, 0); Serial.println("MPU9250/AK8963");
      //    display.setCursor(0, 8); Serial.println(" x   y   z  ");
      //
      //    display.setCursor(0,  16); Serial.println((int)(1000*ax));
      //    display.setCursor(24, 16); Serial.println((int)(1000*ay));
      //    display.setCursor(48, 16); Serial.println((int)(1000*az));
      //    display.setCursor(72, 16); Serial.println("mg");
      //
      //    display.setCursor(0,  24); Serial.println((int)(gx));
      //    display.setCursor(24, 24); Serial.println((int)(gy));
      //    display.setCursor(48, 24); Serial.println((int)(gz));
      //    display.setCursor(66, 24); Serial.println("o/s");
      //
      //    display.setCursor(0,  32); Serial.println((int)(mx));
      //    display.setCursor(24, 32); Serial.println((int)(my));
      //    display.setCursor(48, 32); Serial.println((int)(mz));
      //    display.setCursor(72, 32); Serial.println("mG");
      //
      //    display.setCursor(0,  40); Serial.println("Gyro T ");
      //    display.setCursor(50,  40); Serial.println(temperature, 1); Serial.println(" C");
      //    display.display();
      //
      count = millis();
      digitalWrite(myLed, !digitalRead(myLed));  // toggle led
    }
  }
  else {

    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > 50) { // update LCD once per half-second independent of read rate

      if (SerialDebug) {
        //    Serial.print("ax = "); Serial.print((int)1000*ax);
        //    Serial.print(" ay = "); Serial.print((int)1000*ay);
        //    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
        //    Serial.print("gx = "); Serial.print( gx, 2);
        //    Serial.print(" gy = "); Serial.print( gy, 2);
        //    Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
        //    Serial.print("mx = "); Serial.print( (int)mx );
        //    Serial.print(" my = "); Serial.print( (int)my );
        //    Serial.print(" mz = "); Serial.print( (int)mz ); Serial.println(" mG");
        //
        //    Serial.print("q0 = "); Serial.print(q[0]);
        //    Serial.print(" qx = "); Serial.print(q[1]);
        //    Serial.print(" qy = "); Serial.print(q[2]);
        //    Serial.print(" qz = "); Serial.println(q[3]);
      }

      // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
      // In this coordinate system, the positive z-axis is down toward Earth.
      // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
      // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
      // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
      // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
      // applied in the correct order which for this configuration is yaw, pitch, and then roll.
      // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
      yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
      pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
      roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
      pitch *= 180.0f / PI;
      yaw   *= 180.0f / PI;
      yaw   += 12.97; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
      roll  *= 180.0f / PI;

      yaw = yaw + 180;
      //Serial.print("Yaw, Pitch, Roll: ");
      //Serial.print(yaw, 2);


      //        yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
      //    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
      //    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
      //    pitch *= 180.0f / PI;
      //    yaw   *= 180.0f / PI;
      //    yaw   = 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
      //    roll  *= 180.0f / PI;

      if (SerialDebug) {
        Serial.print("Yaw, Pitch, Roll: ");
        Serial.print(yaw, 2);
        Serial.print(", ");
        Serial.print(pitch, 2);
        Serial.print(", ");
        Serial.println(roll, 2);
      }
      //
      ////  Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
      //    }

      //    display.clearDisplay();
      //
      //    display.setCursor(0, 0); Serial.println(" x   y   z  ");
      //
      //    display.setCursor(0,  8); Serial.println((int)(1000*ax));
      //    display.setCursor(24, 8); Serial.println((int)(1000*ay));
      //    display.setCursor(48, 8); Serial.println((int)(1000*az));
      //    display.setCursor(72, 8); Serial.println("mg");
      //
      //    display.setCursor(0,  16); Serial.println((int)(gx));
      //    display.setCursor(24, 16); Serial.println((int)(gy));
      //    display.setCursor(48, 16); Serial.println((int)(gz));
      //    display.setCursor(66, 16); Serial.println("o/s");
      //
      //    display.setCursor(0,  24); Serial.println((int)(mx));
      //    display.setCursor(24, 24); Serial.println((int)(my));
      //    display.setCursor(48, 24); Serial.println((int)(mz));
      //    display.setCursor(72, 24); Serial.println("mG");
      //
      //    display.setCursor(0,  32); Serial.println((int)(yaw));
      //    display.setCursor(24, 32); Serial.println((int)(pitch));
      //    display.setCursor(48, 32); Serial.println((int)(roll));
      //    display.setCursor(66, 32); Serial.println("ypr");
      //
      // With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and
      // >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
      // The filter update rate is determined mostly by the mathematical steps in the respective algorithms,
      // the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
      // an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
      // filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively.
      // This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
      // This filter update rate should be fast enough to maintain accurate platform orientation for
      // stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
      // produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
      // The 3.3 V 8 MHz Pro Mini is doing pretty well!
      //    display.setCursor(0, 40);
      //    Serial.println("rt: "); Serial.println((float) sumCount / sum, 2); Serial.println(" Hz");
      //    display.display();

      count = millis();
      sumCount = 0;
      sum = 0;
    }
  }

}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

void getMres() {
  switch (Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
      mRes = 10.*4912. / 8190.; // Proper scale to return milliGauss
      break;
    case MFS_16BITS:
      mRes = 10.*4912. / 32760.0; // Proper scale to return milliGauss
      break;
  }
}

void getGres() {
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
      gRes = 250.0 / 32768.0;
      break;
    case GFS_500DPS:
      gRes = 500.0 / 32768.0;
      break;
    case GFS_1000DPS:
      gRes = 1000.0 / 32768.0;
      break;
    case GFS_2000DPS:
      gRes = 2000.0 / 32768.0;
      break;
  }
}

void getAres() {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
      aRes = 2.0 / 32768.0;
      break;
    case AFS_4G:
      aRes = 4.0 / 32768.0;
      break;
    case AFS_8G:
      aRes = 8.0 / 32768.0;
      break;
    case AFS_16G:
      aRes = 16.0 / 32768.0;
      break;
  }
}


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void readMagData(int16_t * destination)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
    readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
    uint8_t c = rawData[6]; // End data read by reading ST2 register
    if (!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
      destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
      destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
      destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
    }
  }
}

int16_t readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

void initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128) / 256. + 1.;
  destination[2] =  (float)(rawData[2] - 128) / 256. + 1.;
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}


void initMPU9250()
{
  // wake up device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  delay(100); // Wait for all registers to reset

  // get stable time source
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200);

  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
  // be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
  // determined inset in CONFIG above

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG);
  //  writeRegister(GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x02); // Clear Fchoice bits [1:0]
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro
  // writeRegister(GYRO_CONFIG, c | 0x00); // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG

  // Set accelerometer full-scale range configuration
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG);
  //  writeRegister(ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c & ~0x0F); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c | 0x03); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
  delay(100);
}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9250(float * dest1, float * dest2)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  // reset device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);

  // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
  // else use the internal oscillator, bits 2:0 = 001
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
  writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
  delay(200);

  // Configure device for bias calculation
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];

  }
  accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;

  if (accel_bias[2] > 0L) {
    accel_bias[2] -= (int32_t) accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
  }
  else {
    accel_bias[2] += (int32_t) accelsensitivity;
  }

  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1] / 4)       & 0xFF;
  data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2] / 4)       & 0xFF;

  // Push gyro biases to hardware registers
  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

  // Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity;
  dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for (ii = 0; ii < 3; ii++) {
    if ((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1] / 8);
  accel_bias_reg[2] -= (accel_bias[2] / 8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  // Apparently this is not working for the acceleration biases in the MPU-9250
  // Are we handling the temperature correction bit properly?
  // Push accelerometer biases to hardware registers
  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

  // Output scaled accelerometer biases for display in the main program
  dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
  dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
  dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;


}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t selfTest[6];
  int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
  float factoryTrim[6];
  uint8_t FS = 0;

  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 1 << FS); // Set full scale range for the gyro to 250 dps
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 1 << FS); // Set full scale range for the accelerometer to 2 g

  for ( int ii = 0; ii < 200; ii++) { // get average current values of gyro and acclerometer

    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average current readings
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }

  // Configure the accelerometer for self-test
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  delay(25);  // Delay a while to let the device stabilize

  for ( int ii = 0; ii < 200; ii++) { // get average self-test values of gyro and acclerometer

    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average self-test readings
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }

  // Configure the gyro and accelerometer for normal operation
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);
  delay(25);  // Delay a while to let the device stabilize

  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
  selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
  selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
  selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
  selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
  selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
  factoryTrim[0] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
  factoryTrim[1] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
  factoryTrim[2] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
  factoryTrim[3] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
  factoryTrim[4] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
  factoryTrim[5] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  // To get percent, must multiply by 100
  for (int i = 0; i < 3; i++) {
    destination[i]   = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i]; // Report percent differences
    destination[i + 3] = 100.0 * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3]; // Report percent differences
  }

}


// Wire.h read and write protocols
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address
  while (Wire.available()) {
    dest[i++] = Wire.read();
  }         // Put read results in the Rx buffer
}

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
  norm = 1.0f / norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;

}



void magcalMPU9250(float * dest1, float * dest2)
{
  Serial.println("B");
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {0x8000, 0x8000, 0x8000}, mag_min[3] = {0x7FFF, 0x7FFF, 0x7FFF}, mag_temp[3] = {0, 0, 0};

  Serial.println("Mag Calibration: Wave device in a figure eight until done!");

  sample_count = 128;
  for (ii = 0; ii < sample_count; ii++) {
    readMagData(mag_temp);  // Read the mag data
    for (int jj = 0; jj < 3; jj++) {
      if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
  }

  // Get hard iron correction
  mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

  dest1[0] = (float) mag_bias[0] * mRes * magCalibration[0]; // save mag biases in G for main program
  dest1[1] = (float) mag_bias[1] * mRes * magCalibration[1];
  dest1[2] = (float) mag_bias[2] * mRes * magCalibration[2];

  // Get soft iron correction estimate
  mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2; // get average x axis max chord length in counts
  mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2; // get average y axis max chord length in counts
  mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2; // get average z axis max chord length in counts

  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0;

  dest2[0] = avg_rad / ((float)mag_scale[0]);
  dest2[1] = avg_rad / ((float)mag_scale[1]);
  dest2[2] = avg_rad / ((float)mag_scale[2]);

  Serial.println("Mag Calibration done!");
}

void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrt((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

  // Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f)
  {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
  }
  else
  {
    eInt[0] = 0.0f;     // prevent integral wind up
    eInt[1] = 0.0f;
    eInt[2] = 0.0f;
  }

  // Apply feedback terms
  gx = gx + Kp * ex + Ki * eInt[0];
  gy = gy + Kp * ey + Ki * eInt[1];
  gz = gz + Kp * ez + Ki * eInt[2];

  // Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;

}
