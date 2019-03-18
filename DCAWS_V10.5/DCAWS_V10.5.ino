// Alex Barker and Adam Hall
// 3/04/2019
// Code to run autonomous water sampler DCAWS
// Changes from previous version: verified safety check codes, variable overhaul.

// include libraries
#include <Adafruit_GPS.h>
#include <Servo.h>
#include <SD.h>
#include <SPI.h>

// declare serial constants
#define GPS_Serial Serial3
#define radio Serial2

// declare GPS
Adafruit_GPS GPS(&GPS_Serial);

// declare servo
Servo servo;

// declare pin constants
#define ESC_RC_PW 2
#define CURR_SENSOR 14
#define TEMPERATURE_SENSOR 15
#define LEAK_SENSOR 16
#define INT_PRESS_SENSOR 17
#define IMU_SDA 18
#define IMU_SCL 19
#define SOLENOID_1 20
#define SOLENOID_2 21
#define SOLENOID_3 22
#define DEPTH_SENSOR 23
#define BATTERY_SENSOR_EN 31
#define BATTERY_CELL_READ 32
#define THIRD_CELL_READ 33
#define SECOND_CELL_READ 34
#define TOP_CELL_READ 35
#define chipSelect BUILTIN_SDCARD

//declare constants
#define STOP_SIGNAL 1500                  //PWM signal that stops thruster
#define SETUP_DELAY 1000                  // time delay for setup signals
#define MAX_GPS_TIME 120000               // 2min cutoff for GPS to recieve new signal
#define RES 1024.0                        // 10 bit resolution for adc conversion
#define V_TEENSY 3.0                      // 3 volt reference into the teensy
#define RHO 1025.0                        // density of seawater (kg/m^3)
#define g 9.81                            // (gravity m/s^2)
//#define MIN_SAFE_CURR                     // Minimum safe current before abort is neccesary
#define MAX_SAFE_TEMP 85.0               // Maximum operating temp before abort needed
#define V_SUPPLY 5.0                     // 5 volt supply voltage
//#define MAX_SAFE_PRESS                    // Maximum safe pressure before aborting
# define MIN_SAFE_LEAK_V 3.0             // Minimum voltage of 3V n leak sensor any less means leak
#define WINSZ_Z 10                       // window size used for depth of 10
#define DATA_HZ 100                      //update frequency of 10 HZ for log (and PID)
#define GPS_DRIFT_TIME 60000             // desired duration of drift state in milliseconds
#define WINSZ_GPS 100                    // window size used for GPS of 100
#define MIN_GPS_NUM  10                  // Minimum number of good gps readings taken during GPS_DRIFT to consider successful
#define PID_RANGE -2
#define HOLD_TOL .2                      // declare tolerance on hold
#define KP 2.6
#define KI .225
#define KD 10
#define FBPOS 5                          // positive buoyancy of system
#define A .09                            // Area of attack of system (m^2)
#define CD .38                           //the drag coefficient of system from solidworks
#define M 10.5                           // Mass of system (kg)
#define T_GAIN 17.98561151
#define HOLD_TIME  10000                 //hold time of ten seconds for depth and sample
#define SURF_NUM 10                      //number of times for pressure sensor to read surface value before claiming surface
#define GPS_SEND_FREQ 50000
int fuck;
int doublefuck = 1;
double leakVoltage;
//declare variables
File DCAWS_Depth;                        // Create file on the sd card to log depth
File DCAWS_GPS;                          // Create file on the sd card to log GPS
bool missionReady = true;                // Create bool to track mission status through diagnostics
bool initCheckGPS = true;
bool newGPS = false;
int goodGPSCount = 0;
double depth;                            // variable to hold depth
double current;                          // variable for internal current
double temperature;                      // variable for internal temp
double pressure;                         // variable for internal pressure
double avgDepth;                         // variable for avg Depth after going through moving average
bool avgInitZ = true;                    // declare initial mvavg flag for depth true
double pidDepth;                         // will use if want avgDepth updated at 10hz
bool avgInitGPS = true;
bool initGPS = true;
double avgLat;
double avgLon;
int targetCount = 1;
int targetDepth;
int targetDepth1 = 10;
int targetDepth2 = 15;
int targetDepth3 = 20;
float pidError = 0;
float Fb = (M*g) + FBPOS;               // calculate total buoyancy force of system
float thrust;
float prevError = 0;
bool initDepthHold = true;
bool sampleTaken = false;
bool initSample = true;
float prevDepth = 0;
int surfaceCount = 0;

// declare state machine variables
enum {GPS_DRIFT, ASCENT, SAMPLE_MISSION, GPS_FINAL, ABORT} state;

// declare timer
IntervalTimer timerDepth;

//declare elapsed timer to use for gpsTimeout
elapsedMillis gpsTimeout;


void setup()
{
  char go = 'n';
  //initialize components
  setupRadio();
  setupThruster();
  //delay(1000);
  setupGPS();
  //delay(1000);
  setupSD();
  //delay(1000);
  setupSolenoids();
  delay(1000);
  //prior to deployment conduct system Diagnosis and wait for user command to continue
  while(go != 'y')
  { 
    //run a system diagnostic 
    systemDiagnosis();
    radio.println(F("Send 'y' (yes) to begin mission "));
    radio.println(F("or enter any other key to rerun diagnostic."));
    while(!radio.available())
      {
        
      } 
    go = radio.read(); 
    //radio.println(go); 
  }
  digitalWrite(SOLENOID_1, LOW);
  digitalWrite(SOLENOID_2, LOW);
  digitalWrite(SOLENOID_3, LOW);
  state = SAMPLE_MISSION;
  timerDepth.begin(getDepth, 10000);
}

//create timer for mission
elapsedMillis sinceStart;
//create timer for PID control
elapsedMillis sincePrev;
//create timer for data renewal
elapsedMillis sinceDataFreq;
elapsedMillis sinceGPS;
elapsedMillis sinceErrorInTol;
elapsedMillis sinceTrigger;

void loop()
{
  //pause interrupts and make copy of current depth reading
  noInterrupts();
  double depthCopy = depth;
  interrupts();
  // average pressure (not sure if moving avg right)
  //mvavgDepth(depthCopy, avgInitZ);
  //avgInitZ = false;
  // update depth log (and set depth for pid) at rate of 10Hz
  if (sinceDataFreq >= DATA_HZ)
  {
    logDepth();
    pidDepth = depthCopy;
    checkSafetySensors();
    sinceDataFreq = 0;
  }
  switch (state)
  {
    case SAMPLE_MISSION:
//    //radio.println(F("entering sample mission state"));
//    //radio.println(F(" pid depth reading"));
//    //radio.println(pidDepth);
//    if(pidDepth < 0)
//    {
//        if (targetCount ==1)
//        {
//          radio.println(F("taking sample 1"));
//          takeSample(1);
//          if(sampleTaken)
//          {
//            initSample = true;
//          }
//        }
//        if(targetCount==2)
//        {
//          //radio.println(F("taking sample 2"));
//          takeSample(2);
//          if(sampleTaken)
//          {
//            initSample = true;
//          }
//        }
//        if(targetCount==3)
//        {
//          radio.println(F("taking sample 3"));
//          takeSample(3);
//          if(sampleTaken)
//          {
//            initSample = true;
//          }
//        }
//    }
    if(pidDepth > 2)
    {     
        if(initSample)
        {
          radio.println("open1");
          digitalWrite(SOLENOID_1, HIGH);
          sinceTrigger = 0;
          initSample = false;
          fuck = 1;
        }
        if(sinceTrigger > 10000 && fuck && doublefuck)
        {
          digitalWrite(SOLENOID_1, LOW);
          radio.println("close1");
          digitalWrite(SOLENOID_2, HIGH);
          radio.println("open2");
          fuck = 0;
        }
        if(sinceTrigger > 20000 && !fuck && doublefuck)
        {
          digitalWrite(SOLENOID_2, LOW);
          radio.println("close2");
          digitalWrite(SOLENOID_3, HIGH);
          radio.println("open3");
          fuck = 1;
          doublefuck = 0;
        }
        if(sinceTrigger > 30000 && fuck)
        {
          digitalWrite(SOLENOID_3, LOW);
          radio.println("close3");
          targetCount = 4;
          fuck = 0;
          doublefuck = 0;
        }      
    }
    if (targetCount > 3)
        state = ASCENT;
    break;

    case ASCENT:
      radio.println(F("entering ascent state"));
      ascend();
      if (depthCopy <= 2 && prevDepth <= 2)
      {
        surfaceCount++;
        if (surfaceCount > SURF_NUM)
        {
          servo.writeMicroseconds(STOP_SIGNAL);
          state = GPS_FINAL;
          initGPS = true;
        }
      }
    break;

    case GPS_FINAL:
      radio.println(F("made it to GPS Final State"));
    break;
  }
}


