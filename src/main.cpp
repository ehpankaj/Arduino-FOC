#include <Arduino.h>
#include "SimpleFOC.h"

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 25, 33);

// Initialise the commander
Commander command = Commander(Serial);
void doTarget(char *cmd) { command.scalar(&motor.target, cmd); }
void doLimit(char *cmd) { command.scalar(&motor.voltage_limit, cmd); }
void doMotor(char *cmd) { command.motor(&motor, cmd); }

void setup()
{
    // Initialise the Serial
    Serial.begin(115200);
    Wire.begin();
    motor.useMonitoring(Serial);

    // Initialise and link the sensor
    sensor.init();
    motor.linkSensor(&sensor);
    Serial.println("Sensor ready");

    // Initialise and link the driver
    if (!driver.init())
    {
        Serial.println("Driver init failed!");
        return;
    }
    driver.voltage_power_supply = 8;
    driver.voltage_limit = 6;
    driver.pwm_frequency = 25000;
    motor.linkDriver(&driver);
    Serial.println("Driver ready!");

    // Initialise motor and the FOC algorithm
    motor.voltage_limit = 3; // [V]
    motor.velocity_limit = 12.56;
    motor.KV_rating = 80;
    motor.torque_controller = TorqueControlType::voltage;
    motor.controller = MotionControlType::angle;
    motor.init();
    if (!motor.initFOC())
    {
        Serial.println("FOC init failed!");
        return;
    }
    Serial.println("Motor ready!");

    // Set initial motor target
    motor.target = 3;

    // Add commander commands
    command.add('m', doMotor, "Motor");
    command.add('t', doTarget, "target velocity");
    command.add('l', doLimit, "voltage limit");
    _delay(500);
};

void loop()
{
    motor.loopFOC();
    motor.move();
    command.run();
};