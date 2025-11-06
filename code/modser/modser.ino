#include <SimpleFOC.h>

// Motor instance
BLDCMotor motor = BLDCMotor(7);           // 7 = pole pairs (adjust to match your motor)
BLDCDriver3PWM driver = BLDCDriver3PWM(D0, D1, D2, D3); // A, B, C, enable

// Magnetic sensor
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// Current limiting is highly recommended on Mini
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();
  sensor.init();
  motor.linkSensor(&sensor);

  // Setup driver
  driver.voltage_power_supply = 12;       // Set to your supply voltage
  driver.init();
  motor.linkDriver(&driver);

  // Motor configuration
  // motor.controller = MotionControlType::torque;
  // motor.foc_modulation = FOCModulationType::SinePWM;




    // set control loop to be used
  motor.controller = MotionControlType::angle;
    // controller configuration based on the control type 
  // velocity PI controller parameters
  // default P=0.5 I = 10
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;




    // angle P controller 
  // default P=20
  motor.P_angle.P = 5;
  //  maximal velocity of the position control
  // default 20
  motor.velocity_limit = 4;

  // Initialize motor
  motor.init();
  motor.initFOC();

  Serial.println("Motor ready.");
}

void loop() {
  motor.loopFOC();

  // // Apply torque — change this for testing!
  // motor.move(0.5); // 0.2 = light torque, can go from -1.0 to 1.0 depending on supply
  motor.move(0);
}
