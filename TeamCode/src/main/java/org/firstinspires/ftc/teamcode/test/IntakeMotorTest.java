package org.firstinspires.ftc.teamcode.test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Standalone bench test for a closed-loop (PID) velocity-controlled intake motor.
 *
 * Gamepad 2 Left Joystick:
 * - Negative Y (push forward): spins motor in positive direction at INTAKE_MAX_SPEED (slower).
 * - Positive Y (pull backward): spins motor in negative direction at OUTPUT_MAX_SPEED (faster).
 * All public static fields below are live-editable from the Panels dashboard.
 */
@Configurable
@TeleOp(name = "Test: Intake Motor", group = "Test")
public class IntakeMotorTest extends OpMode {

    // Configurable motor name in Panels
    public static String MOTOR_NAME = "intake";

    // PIDF tuning parameters for closed-loop velocity control
    public static double P = 10.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 12.0;

    // Configurable speed limits in Panels (Intake is slower than Output)
    public static double INTAKE_MAX_SPEED = 1500; // ticks/sec for intake (negative Y stick -> positive motor direction)
    public static double OUTPUT_MAX_SPEED = 2800; // ticks/sec for output/eject (positive Y stick -> negative motor direction)
    public static double VELOCITY_STEP = 100;

    public static boolean running = false;
    public static boolean reverseDirection = false;

    // Optional direct power mode (for open-loop testing without encoders)
    public static boolean useDirectPower = false;
    public static double directPowerIntake = 0.5;  // slower intake power limit
    public static double directPowerOutput = 0.85; // faster output power limit

    private DcMotorEx intakeMotor;

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Intake Motor Test initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean dpadUp = gamepad2.dpadUpWasPressed() || gamepad1.dpadUpWasPressed();
        boolean dpadDown = gamepad2.dpadDownWasPressed() || gamepad1.dpadDownWasPressed();

        if (dpadUp) {
            INTAKE_MAX_SPEED += VELOCITY_STEP;
        }
        if (dpadDown) {
            INTAKE_MAX_SPEED = Math.max(0, INTAKE_MAX_SPEED - VELOCITY_STEP);
        }

        // Controller 2 Left stick (-1.0 is forward/negative Y, +1.0 is backward/positive Y)
        double stickY = Math.abs(gamepad2.left_stick_y) >= 0.05 ? gamepad2.left_stick_y : gamepad1.left_stick_y;

        intakeMotor.setDirection(reverseDirection ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        if (useDirectPower) {
            double powerOutput;
            if (Math.abs(stickY) >= 0.05) {
                // Negative Y -> positive motor direction at directPowerIntake (slower)
                // Positive Y -> negative motor direction at directPowerOutput (faster)
                double maxPower = stickY < 0 ? directPowerIntake : directPowerOutput;
                powerOutput = -stickY * maxPower;
            } else if (running) {
                powerOutput = directPowerIntake;
            } else {
                powerOutput = 0;
            }
            intakeMotor.setPower(powerOutput);
        } else {
            // Re-applied every loop so PIDF edits made in Panels take effect immediately
            intakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));

            double currentTargetVelocity;
            if (Math.abs(stickY) >= 0.05) {
                // Negative Y stick -> Positive motor speed (slower INTAKE_MAX_SPEED)
                // Positive Y stick -> Negative motor speed (faster OUTPUT_MAX_SPEED)
                double maxSpeed = stickY < 0 ? INTAKE_MAX_SPEED : OUTPUT_MAX_SPEED;
                currentTargetVelocity = -stickY * maxSpeed;
            } else if (running) {
                currentTargetVelocity = INTAKE_MAX_SPEED;
            } else {
                currentTargetVelocity = 0;
            }

            intakeMotor.setVelocity(currentTargetVelocity);
        }

        telemetry.addData("Stick Y (Gamepad 2)", "%.2f", stickY);
        telemetry.addData("Mode", useDirectPower ? "Direct Power" : "PIDF Velocity");
        telemetry.addData("Intake Max Speed (Slower)", INTAKE_MAX_SPEED);
        telemetry.addData("Output Max Speed (Faster)", OUTPUT_MAX_SPEED);
        telemetry.addData("Actual Motor Velocity (ticks/s)", intakeMotor.getVelocity());
        telemetry.addData("Actual Motor Power", intakeMotor.getPower());
        telemetry.addData("PIDF", "P=%.2f I=%.2f D=%.2f F=%.2f", P, I, D, F);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (intakeMotor != null) {
            intakeMotor.setVelocity(0);
            intakeMotor.setPower(0);
        }
    }
}
