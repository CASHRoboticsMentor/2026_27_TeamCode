package org.firstinspires.ftc.teamcode.test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Standalone bench test for pollen_servo and nector_servo using center-relative degrees (150° center).
 *
 * Gamepad 2 Left Trigger: Rotates pollen_servo relative to center (150°) by POLLEN_SERVO_RELATIVE_DEG in CCW/CW direction.
 * Gamepad 2 Right Trigger: Rotates nector_servo relative to center (150°) by NECTOR_SERVO_RELATIVE_DEG in CW/CCW direction.
 *
 * Released trigger state returns both servos back to center position (150° / command 0.5).
 * All degree offsets and direction flags are live-editable from the Panels web dashboard.
 */
@Configurable
@TeleOp(name = "Test: Pollen & Nector Servos", group = "Test")
public class SensorMotorServoTest extends OpMode {

    // Configurable Servo hardware device names
    public static String POLLEN_SERVO_NAME = "pollen_servo";
    public static String NECTOR_SERVO_NAME = "nector_servo";

    // Center position degrees (command 0.5) and physical range
    public static double SERVO_CENTER_DEG = 150.0;
    public static double SERVO_TOTAL_RANGE_DEG = 300.0;

    // Relative degree offset from center on trigger press
    public static double POLLEN_SERVO_RELATIVE_DEG = 90.0; // degrees from center
    public static double NECTOR_SERVO_RELATIVE_DEG = 90.0; // degrees from center

    // Direction looking at shaft (true = Counter-Clockwise [CCW], false = Clockwise [CW])
    public static boolean POLLEN_SERVO_CCW = true;  // CCW (+offset from 150°)
    public static boolean NECTOR_SERVO_CCW = false; // CW (-offset from 150°)

    private Servo pollenServo;
    private Servo nectorServo;

    @Override
    public void init() {
        try {
            pollenServo = hardwareMap.get(Servo.class, POLLEN_SERVO_NAME);
            pollenServo.setPosition(centerDegreesToCommand());
        } catch (Exception e) {
            telemetry.addData("Warning", "Pollen servo '%s' not found", POLLEN_SERVO_NAME);
        }

        try {
            nectorServo = hardwareMap.get(Servo.class, NECTOR_SERVO_NAME);
            nectorServo.setPosition(centerDegreesToCommand());
        } catch (Exception e) {
            telemetry.addData("Warning", "Nector servo '%s' not found", NECTOR_SERVO_NAME);
        }

        telemetry.addLine("Pollen & Nector Servo Test Initialized (Center-Relative Mode)");
        telemetry.update();
    }

    private double centerDegreesToCommand() {
        return SERVO_CENTER_DEG / SERVO_TOTAL_RANGE_DEG;
    }

    private double centerRelativeDegreesToCommand(double relativeDegrees, boolean isCCW) {
        double absoluteDegrees = isCCW ? (SERVO_CENTER_DEG + relativeDegrees) : (SERVO_CENTER_DEG - relativeDegrees);
        return Math.max(0.0, Math.min(1.0, absoluteDegrees / SERVO_TOTAL_RANGE_DEG));
    }

    @Override
    public void loop() {
        boolean leftTriggerPressed = gamepad2.left_trigger > 0.1 || gamepad1.left_trigger > 0.1;
        boolean rightTriggerPressed = gamepad2.right_trigger > 0.1 || gamepad1.right_trigger > 0.1;

        if (pollenServo != null) {
            double command = leftTriggerPressed ? centerRelativeDegreesToCommand(POLLEN_SERVO_RELATIVE_DEG, POLLEN_SERVO_CCW) : centerDegreesToCommand();
            pollenServo.setPosition(command);
            double currentAngle = command * SERVO_TOTAL_RANGE_DEG;
            telemetry.addData("Pollen Target Angle", "%.1f° (Cmd: %.3f)", currentAngle, command);
        }

        if (nectorServo != null) {
            double command = rightTriggerPressed ? centerRelativeDegreesToCommand(NECTOR_SERVO_RELATIVE_DEG, NECTOR_SERVO_CCW) : centerDegreesToCommand();
            nectorServo.setPosition(command);
            double currentAngle = command * SERVO_TOTAL_RANGE_DEG;
            telemetry.addData("Nector Target Angle", "%.1f° (Cmd: %.3f)", currentAngle, command);
        }

        telemetry.addData("Left Trigger (Pollen)", leftTriggerPressed);
        telemetry.addData("Right Trigger (Nector)", rightTriggerPressed);
        telemetry.update();
    }
}
