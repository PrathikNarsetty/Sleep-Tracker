// Prathik Narsetty

/*
  Hardware Connections (Breakoutboard to Arduino):
  -5V = 5V (3.3V is allowed)
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected
 
  The MAX30105 Breakout can handle 5V or 3.3V I2C logic. We recommend powering the board with 5V
  but it will also run at 3.3V.
*/

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"
#include <MPU6050.h>
#include <Arduino.h>
MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

#define THRESHOLD 15000

// timer code 

hw_timer_t * timer = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool  sampleReady = false;

// This is your “ISR” – runs in interrupt context
void IRAM_ATTR onTimer() {
  // must be very fast!
  portENTER_CRITICAL_ISR(&timerMux);
  sampleReady = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}

//////////
// MPU setup
MPU6050 mpu;

const int32_t SPIKE_THRESHOLD = 5000;  

int16_t ax, ay, az, gx, gy, gz;
int16_t last_ax = 0, last_ay = 0, last_az = 0;

uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read

long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

const uint64_t SLEEP_TIME_US = 30;

void setup()
{
  Serial.begin(115200); // initialize serial communication at 115200 bits per second:
    while (!Serial);
  Serial.println("Initializing...");

  Wire.begin(21, 22); // SDA = GPIO21, SCL = GPIO22
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);
    mpu.initialize();
      if (!mpu.testConnection()) {
    Serial.println(F("ERROR: MPU6050 not found. Check wiring/power!"));
    while (1) { delay(10); }   // halt here
  }
  Serial.println(F("MPU6050 initialized successfully"));

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  while (Serial.available() == 0) ; //wait until user presses a key
  Serial.read();

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings


  // setup timers
// esp_sleep_enable_timer_wakeup(SLEEP_TIME_US); // 30 seconds
// esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
//  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

  timer = timerBegin(0, 80, true);

  // attach onTimer() to our timer
  timerAttachInterrupt(timer, &onTimer, true);

  // set alarm to fire every 3 s (3 000 000 µs), auto-reload = true
  timerAlarmWrite(timer, 3000000, true);

  // enable the alarm
  timerAlarmEnable(timer);
  
}

void loop()
{
  // setup

 
    // 2) go into light sleep
//   Serial.flush();
//      esp_sleep_enable_timer_wakeup(100); // 30 seconds
//   Serial.printf("Sleeping for %llu seconds...\n", SLEEP_TIME_US / 1000000);

//   esp_light_sleep_start();

//   // 3) resumes here after timer interrupt
//   Serial.println("Woke up from timer!");

  // 4) re-initialize I2C
  if (sampleReady) {
      particleSensor.wakeUp();
    mpu.setSleepEnabled(false);
    portENTER_CRITICAL(&timerMux);
    sampleReady = false;
    portEXIT_CRITICAL(&timerMux);
  Wire.begin(21, 22);   
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps
  // motion logic 
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.println(ax);
  Serial.println(ay);
  Serial.println(az);
  Serial.println(gx);
  Serial.println(gy);
  Serial.println(gz);

  int diff = abs(ax- last_ax) + abs(ay - last_ay) + abs(az-last_az);

  if ( diff> THRESHOLD) {
    Serial.print("Motion Spike detected!!!");
    Serial.print(diff);
  }
  last_ax = ax;   
  last_ay = ay;
  last_az = az;
  // delay(2500);


  //read the first 100 samples, and determine the signal range



  // O2 level
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    // Serial.print(F("red="));
    // Serial.print(redBuffer[i], DEC);
    // Serial.print(F(", ir="));
    // Serial.println(irBuffer[i], DEC);
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  Serial.print("O2 Level: ");
  Serial.println(spo2);


  // heart rate
bool beat = false;

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second  
  // add feature for which if it doesnt get a good reading in 15 seconds to ditch 
  uint32_t pTime = millis();
  while (!beat) {
    if (millis()-pTime> 15000) {
      break;
    }
   long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);   // converts the delta ms into a BPS

    if (beatsPerMinute < 185 && beatsPerMinute > 35)
    {
      // its a valid heartrate so store it 
      // store the beatsPerMinute into a ROM buffer capable of storing 15KB of data here ( each location can be 1 byte( truncate the decimals))
  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.println();
  beat = true;

  
    }
    Serial.print("BAD");
  }
  }
//   esp_light_sleep_start();
  particleSensor.shutDown();
  mpu.setSleepEnabled(true);
}
}


// use 2 interrupts | one for motion (turn this off when not in use)
// one for O2 level 

// int counter = 0;
// const int ledPin = 2;           // GPIO pin for onboard LED
// uint64_t sleepTime = 1000;  // Sleep duration in microseconds (10 seconds)

// void setup() {
//     Serial.begin(115200);
//     pinMode(ledPin, OUTPUT);

//     // Enable wake-up by timer
//     esp_err_t result = esp_sleep_enable_timer_wakeup(sleepTime);

//     if (result == ESP_OK) {
//         Serial.println("Timer Wake-Up set successfully as wake-up source.");
//     } else {
//         Serial.println("Failed to set Timer Wake-Up as wake-up source.");
//     }
// }

// void loop() {
//     Serial.printf("Counter: %d\n", counter);
//     counter++;

//     digitalWrite(ledPin, HIGH);  // LED on to indicate wake-up
//     delay(2000);
//     digitalWrite(ledPin, LOW);   // Turn off LED before going to sleep

//     Serial.println("Going into light sleep mode");
//     delay(500);
//     esp_light_sleep_start();     // Enter light sleep
//     Serial.println("Returning from light sleep");
// }
