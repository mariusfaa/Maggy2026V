#include "konfig.h"
#include "magsensor.h"
#include "memory.h"
#include "pid.h"
#include "solenoid.h"


// Timing parameters
constexpr float sensorFrequency = 3000.0;
constexpr int sensorInterval = static_cast<int>(1e6 / sensorFrequency + 0.5f);
constexpr float controlFrequency = 1000.0;
constexpr int controlInterval = static_cast<int>(1e6 / controlFrequency + 0.5f);

constexpr float Kp = 0.25;
constexpr float Kd = 0.0025;

#define X_POS 0
#define X_NEG 1
#define Y_POS 2
#define Y_NEG 3

Solenoid solenoids[4] = {
    Solenoid(MD2_IN1, MD2_IN2, CURRENT_X_POS), // X+
    Solenoid(MD3_IN1, MD3_IN2, CURRENT_X_NEG), // X-
    Solenoid(MD4_IN1, MD4_IN2, CURRENT_Y_POS), // Y+
    Solenoid(MD1_IN1, MD1_IN2, CURRENT_Y_NEG)  // Y-
};

// Sensor objects - one for each physical sensor
TLx493D_A1B6 _mag = TLx493D_A1B6(Wire, TLx493D_IIC_ADDR_A0_e);
MagSensor mag(&_mag);

uint32_t controlTimer, measurementTimer, logTimer;

pidc_t pid_x = {0};
pidc_t pid_y = {0};

double mx = 0;
double my = 0;
double mz = 0;

constexpr double mag_alpha = 0.5;

void setup() {

  analogWriteResolution(PWM_BIT_RANGE);
  analogReadResolution(ANALOG_READ_BIT_RANGE);

  // i2c_recover();

  Serial.begin(115200);
  // while (!Serial);
  // ASSERT(SD.begin(BUILTIN_SDCARD), "Failed to initialize SD library.");
  // if(SD.open("/")){
  //   Serial.println("SD card mounted successfully.");
  // }
  // else{
  //   Serial.println("Failed to mount SD card.");
  // }

  Serial.println("Starting solenoids");
  uint32_t tic = micros();
  uint32_t toc;
  for (auto &sol : solenoids) {
    sol.initialize();
    sol.calibrate();
  }
  toc = micros();
  Serial.printf("Solenoids initialized in %.2f ms\n", (toc - tic) / 1000.0f);


  // Initialize I2C and mag sensor
  Serial.println("I2C init...");
  // Clear the I2C bus first for reliability
  pinMode(SDA, OUTPUT);
  pinMode(SCL, OUTPUT);

  reset_i2c();

  // Initialize I2C communication
  Wire.begin();
  Wire.setClock(200000);

  tic = micros();
  mag.begin();
  // Serial.println("Calibrating mag...");
  mag.calibrate(solenoids);
  toc = micros();
  Serial.printf("Mag initialized in %.2f ms\n", (toc - tic) / 1000.0f);

  // Setup built-in LED
  pinMode(LED_BUILTIN, OUTPUT);

  memInfo();

  pid_init(&pid_x, Kp, 0.0, Kd);
  pid_init(&pid_y, Kp, 0.0, Kd);

  double minmax = 0.24;
  pid_limits(&pid_x, -minmax, minmax);
  pid_limits(&pid_y, -minmax, minmax);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("setup complete");

  Serial.println("starting measurement_loop");
  // Start measurement loop timer
  // measurementTimer.begin(measurement_loop, sensorInterval);

  // run_signals_routine();
  // digitalWrite(LED_BUILTIN, HIGH);
  measurementTimer = micros();
  controlTimer = micros();
  logTimer = millis();

  // uint32_t t0 = micros();

  // // perform step response on i1
  // uint32_t step_time = 0;
  // uint32_t step_data[10000];
  // uint32_t time_data[10000];
  // for(int i = 0; i < 10000; i++)
  // {
  //   if(i == 100)
  //   {
  //     solenoids[X_POS].set_current(0.24);
  //     step_time = micros();
  //   }
  //   step_data[i] = solenoids[X_POS].get_raw_current();
  //   time_data[i] = micros();
  //   // if(!mag.getMeasurements(&mx, &my, &mz, solenoids))
  //   // {
  //   //   mx = 9; my = 9; mz = 9;
  //   // }

  //   // char buffer[160];
  //   // int l = snprintf(buffer, sizeof(buffer),
  //   "t:%ld,bx:%6.2f,by:%6.2f,bz:%6.2f,ux:%6.3f,i1:%6.3f", micros(), mx, my,
  //   mz, solenoids[X_POS].get_commanded_current(), i1);
  //   // if(l > 0 && l < static_cast<int>(sizeof(buffer)))
  //   // {
  //   //   Serial.println(buffer);
  //   // }
  // }
  // solenoids[X_POS].set_current(0);

  // // Print data
  // for (int i = 0; i < 1000; i++)
  // {
  //   float ux = (time_data[i] >= step_time) ? 0.24 : 0.0;
  //   uint32_t i1 = step_data[i];
  //   char buffer[160];
  //   int l = snprintf(buffer, sizeof(buffer), "t:%ld,ux:%.2f,i1:%ld",
  //   micros(), ux, i1); if(l > 0 && l < static_cast<int>(sizeof(buffer)))
  //   {
  //     Serial.println(buffer);
  //   }
  // }

  // while(true)
  // {
  //   (void)0;
  // }
}

void loop() {

  if (run_every_us(&measurementTimer, sensorInterval)) {
    double x, y, z;
    if (mag.getMeasurements(&x, &y, &z, solenoids)) {
      mx = iir_filter(mx, x, mag_alpha);
      my = iir_filter(my, y, mag_alpha);
      mz = z;
    } else {

      mx = 0;
      my = 0;
      reset_i2c();
      digitalToggle(LED_BUILTIN);
      Serial.println("Error reading mag field");
      delay(100);
    }
  }

  if (run_every_us(&controlTimer, controlInterval)) {
    double ux;
    double uy;

    if (fabs(mz) > 5) {
      ux = pid_compute(&pid_x, -mx+0.00);
      uy = pid_compute(&pid_y, -my);
    } else {
      ux = 0;
      uy = 0;
    }

    solenoids[X_POS].set_current(ux);
    solenoids[X_NEG].set_current(-ux);
    solenoids[Y_POS].set_current(uy);
    solenoids[Y_NEG].set_current(-uy);

    double i1, i2, i3, i4;
    i1 = solenoids[X_POS].get_current();
    i2 = solenoids[X_NEG].get_current();
    i3 = solenoids[Y_POS].get_current();
    i4 = solenoids[Y_NEG].get_current();

    if (run_every_ms(&logTimer, 10)) {
      char buffer[160];
      int l = snprintf(buffer, sizeof(buffer),
                       "t:%ld,bx:%6.2f,by:%6.2f,bz:%6.2f,ux:%6.3f,uy:%6.3f,i1:%"
                       "6.3f,i2:%6.3f,i3:%6.3f,i4:%6.3f",
                       micros(), mx, my, mz, ux, uy, i1, i2, i3, i4);
      if (l > 0 && l < static_cast<int>(sizeof(buffer))) {
        Serial.println(buffer);
      }
    }
  }
}