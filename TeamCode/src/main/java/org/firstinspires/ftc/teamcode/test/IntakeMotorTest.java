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
 * Dpad up/down: increment/decrement intake speed. Bumpers: increment/decrement eject speed.
 * A: toggle the motor on/off. X: run forward (intake). B: run reverse (eject).
 * The static fields below are also live-editable from the Panels dashboard.
 */
@Configurable
@TeleOp(name = "Test: Intake Motor", group = "Test")
public class IntakeMotorTest extends OpMode {

    // TODO: match this to the motor's name in the robot configuration.
    private static final String MOTOR_NAME = "intake";

    // TODO: tune these for the actual intake motor.
    public static double P = 10.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 12.0;

    public static double VELOCITY_STEP = 50; // ticks/sec added or removed per button press
    public static double MAX_VELOCITY = 2800; // ticks/sec - adjust for the motor used
    public static double intakeTargetVelocity = 0;
    public static double ejectTargetVelocity = 0;

    private DcMotorEx intakeMotor;
    private boolean running = false;
    private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;

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
        if (gamepad1.dpadUpWasPressed()) {
            intakeTargetVelocity = Math.min(intakeTargetVelocity + VELOCITY_STEP, MAX_VELOCITY);
        }
        if (gamepad1.dpadDownWasPressed()) {
            intakeTargetVelocity = Math.max(intakeTargetVelocity - VELOCITY_STEP, 0);
        }
        if (gamepad1.rightBumperWasPressed()) {
            ejectTargetVelocity = Math.min(ejectTargetVelocity + VELOCITY_STEP, MAX_VELOCITY);
        }
        if (gamepad1.leftBumperWasPressed()) {
            ejectTargetVelocity = Math.max(ejectTargetVelocity - VELOCITY_STEP, 0);
        }
        if (gamepad1.aWasPressed()) {
            running = !running;
        }
        if (gamepad1.xWasPressed()) {
            direction = DcMotorSimple.Direction.FORWARD;
        }
        if (gamepad1.bWasPressed()) {
            direction = DcMotorSimple.Direction.REVERSE;
        }
        intakeMotor.setDirection(direction);

        // Re-applied every loop so PIDF edits made in Panels take effect immediately.
        intakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));

        double targetVelocity = direction == DcMotorSimple.Direction.FORWARD ? intakeTargetVelocity : ejectTargetVelocity;
        intakeMotor.setVelocity(running ? targetVelocity : 0);

        telemetry.addData("Running", running);
        telemetry.addData("Direction", direction);
        telemetry.addData("Intake Speed (ticks/s)", intakeTargetVelocity);
        telemetry.addData("Eject Speed (ticks/s)", ejectTargetVelocity);
        telemetry.addData("Actual Velocity (ticks/s)", intakeMotor.getVelocity());
        telemetry.addData("Power", intakeMotor.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        intakeMotor.setVelocity(0);
    }
}
