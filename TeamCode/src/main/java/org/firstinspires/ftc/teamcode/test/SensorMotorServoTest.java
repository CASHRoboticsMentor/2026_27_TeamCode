package org.firstinspires.ftc.teamcode.test;

import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Test OpMode integrating:
 * - 2 Servos (servo0, servo1)
 * - 1 Touch Sensor (touchSensor)
 * - 1 Distance Sensor (distanceSensor)
 * - 1 Color Sensor (colorSensor)
 * - 1 DC Motor (motor0)
 *
 * Requirements:
 * 1. When touch sensor is pressed, rotate servo 0 one full rotation (position 1.0).
 * 2. If distance sensor sees an object < 0.5 meters, rotate servo 1 half a revolution (position 0.5).
 * 3. If color sensor sees yellow, spin motor0 in 1 direction at half speed (+0.5).
 * 4. If color sensor sees blue, spin motor0 in the other direction at half speed (-0.5).
 * 5. Otherwise (neither color detected), motor0 stands still (0.0).
 */
@Configurable
@TeleOp(name = "Test: Sensors and Actuators", group = "Test")
public class SensorMotorServoTest extends OpMode {

    // Configurable hardware device names
    public static String SERVO_0_NAME = "servo0";
    public static String SERVO_1_NAME = "servo1";
    public static String TOUCH_SENSOR_NAME = "touchSensor";
    public static String DISTANCE_SENSOR_NAME = "distanceSensor";
    public static String COLOR_SENSOR_NAME = "colorSensor";
    public static String MOTOR_NAME = "motor0";

    // Configurable positions and speeds
    public static double SERVO_0_PRESSED_POS = 1.0;
    public static double SERVO_0_DEFAULT_POS = 0.0;
    public static double SERVO_1_HALF_REV_POS = 0.5;
    public static double SERVO_1_DEFAULT_POS = 0.0;

    public static double DISTANCE_THRESHOLD_METERS = 0.5;
    public static double MOTOR_HALF_SPEED = 0.5;

    // Hardware fields
    private Servo servo0;
    private Servo servo1;
    private TouchSensor touchSensor;
    private DistanceSensor distanceSensor;
    private NormalizedColorSensor colorSensor;
    private DcMotor motor0;

    // Array for holding HSV color values
    private final float[] hsvValues = new float[3];

    @Override
    public void init() {
        // Map servos
        servo0 = hardwareMap.get(Servo.class, SERVO_0_NAME);
        servo1 = hardwareMap.get(Servo.class, SERVO_1_NAME);

        servo0.setPosition(SERVO_0_DEFAULT_POS);
        servo1.setPosition(SERVO_1_DEFAULT_POS);

        // Map sensors
        touchSensor = hardwareMap.get(TouchSensor.class, TOUCH_SENSOR_NAME);
        distanceSensor = hardwareMap.get(DistanceSensor.class, DISTANCE_SENSOR_NAME);
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, COLOR_SENSOR_NAME);

        // Map motor
        motor0 = hardwareMap.get(DcMotor.class, MOTOR_NAME);
        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor0.setPower(0);

        telemetry.addLine("Sensors and Actuators Test Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // 1. Touch sensor -> Servo 0
        boolean isTouchPressed = touchSensor.isPressed();
        if (isTouchPressed) {
            servo0.setPosition(SERVO_0_PRESSED_POS);
        } else {
            servo0.setPosition(SERVO_0_DEFAULT_POS);
        }

        // 2. Distance sensor -> Servo 1
        double distanceMeters = distanceSensor.getDistance(DistanceUnit.METER);
        boolean objectDetected = distanceMeters < DISTANCE_THRESHOLD_METERS;
        if (objectDetected) {
            servo1.setPosition(SERVO_1_HALF_REV_POS);
        } else {
            servo1.setPosition(SERVO_1_DEFAULT_POS);
        }

        // 3. Color sensor -> Motor 0
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        float hue = hsvValues[0];
        float saturation = hsvValues[1];
        float value = hsvValues[2];

        // Yellow hue is ~30° to 90°; Blue hue is ~180° to 250°
        boolean isYellow = (saturation > 0.3) && (hue >= 30 && hue <= 90);
        boolean isBlue = (saturation > 0.3) && (hue >= 180 && hue <= 250);

        String detectedColor = "NONE";
        if (isYellow) {
            motor0.setPower(MOTOR_HALF_SPEED);
            detectedColor = "YELLOW";
        } else if (isBlue) {
            motor0.setPower(-MOTOR_HALF_SPEED);
            detectedColor = "BLUE";
        } else {
            motor0.setPower(0.0);
        }

        // Telemetry readout
        telemetry.addData("Touch Pressed", isTouchPressed);
        telemetry.addData("Servo 0 Position", servo0.getPosition());
        telemetry.addData("Distance (m)", "%.3f", distanceMeters);
        telemetry.addData("Servo 1 Position", servo1.getPosition());
        telemetry.addData("Color Detected", detectedColor);
        telemetry.addData("Color Hue/Sat/Val", "H:%.1f S:%.2f V:%.2f", hue, saturation, value);
        telemetry.addData("Motor 0 Power", motor0.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        motor0.setPower(0);
    }
}
