#include <Arduino.h>
#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 12);

TwoWire I2Cone = TwoWire(0);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

float target_velocity = 5;
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

void setup() {
  Serial.begin(115200);
  delay(1000);

  I2Cone.begin(19, 18, 400000UL);
  sensor.init(&I2Cone);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 15.2;
  driver.init();
  motor.linkDriver(&driver);

  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::voltage;

  motor.voltage_sensor_align = 2;   // [V] voltaje de calibración
  motor.voltage_limit = 5;          // [V] sube a 4 si hace falta
  motor.velocity_limit = 30;        // [rad/s]

  // Punto de partida del PI de velocidad (ajustar después)
  motor.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 1.0;
  motor.LPF_velocity.Tf = 0.02;

  motor.useMonitoring(Serial);
  motor.init();
  motor.initFOC();                  // calibra: el motor se moverá un poco

  command.add('T', doTarget, "target velocity");
  Serial.println("Listo. Usa T5, T10, T-5...");
}

void loop() {
  motor.loopFOC();
  motor.move(target_velocity);
  command.run();
}