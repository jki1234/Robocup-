#include "Collection.h"
#include "Control.h"
#include "MotorControl.h"
#include "NavControl.h"
#include "SDCard.h"
#include "Sensors.h"
#include "Utils.h"

// Lets you use GDB to debug the device
// Go to tools->USB Type->Dual Serial to enable this
// Debug with:
//   > C:\Apps\Arduino15\packages\teensy\tools\teensy-compile\5.4.1\arm\bin\arm-none-eabi-gdb.exe C:\Users\csu59\AppData\Local\Temp\arduino\sketches\<hash>\robocup_arduino.ino.elf
// Select the target with:
//   target remote \\.\com13
#include "src/TeensyDebug/TeensyDebug.h"

// #define LOGGING


const int proportional = 0.3;

void setup() {
  // This comes first, so we can see debug printfs from everything else.
  serialInit();
  revolver_init();

#if defined(HAS_DUAL_SERIAL)
  debug.begin(SerialUSB1);
  // halt_cpu();
#endif

  motor_init();
  // DebugSearchSensors();

  SetupSensors();

  sdInit();
#ifdef LOGGING
  loggerInit();
  loggerWrite("t,x,y,target_x,target_y,tdist,thdr,yaw\n");
#endif

  serialPrintf("Ready to start!");
}

void loop() {
  UpdateSensors(); // TODO make this faster so it doesn't muck up the motor PID control
  motorUpdate();

  navUpdate();

  updateMasterFSM();


#ifdef LOGGING
  // static unsigned long lastLog = 0;
  // unsigned long now = micros();
  // if (now - lastLog > 5'000) {
  //   lastLog = now;
  //   loggerWrite("%f,%d,%d,%d,%d,%f\n", micros() / 1.0e6f, leftMotor.getEncoderPos(), rightMotor.getEncoderPos(), leftMotor.getEncoderSpeed(), rightMotor.getEncoderSpeed(), getIMUYaw() / M_PI *
  //   180);
  // }
  loggerPeriodicFlush(1000);
#endif

  // float heading = radians(micros() / (1'000'000 * 4) * 90);
  // motorSelectHeading(heading, 3500);
}
