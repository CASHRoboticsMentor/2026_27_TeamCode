package org.firstinspires.ftc.teamcode.test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Standalone bench test for a closed-loop (PID) velocity-controlled shooter motor.
 *
 * Dpad up/down: increment/decrement target velocity. A: toggle the motor on/off.
 * The static fields below are also live-editable from the Panels dashboard.
 */
@Configurable
@TeleOp(name = "Test: Shooter Motor", group = "Test")
public class ShooterMotorTest extends OpMode {

    // TODO: match this to the motor's name in the robot configuration.
    private static final String MOTOR_NAME = "shooter";

    // TODO: tune these for the actual shooter motor/flywheel.
    public static double P = 10.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 12.0;

    public static double VELOCITY_STEP = 50; // ticks/sec added or removed per button press
    public static double MAX_VELOCITY = 12800; // ticks/sec - adjust for the motor used
    public static double targetVelocity = 0;

    private DcMotorEx shooterMotor;
    private boolean running = false;

    @Override
    public void init() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Shooter Motor Test initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) {
            targetVelocity = Math.min(targetVelocity + VELOCITY_STEP, MAX_VELOCITY);
        }
        if (gamepad1.dpadDownWasPressed()) {
            targetVelocity = Math.max(targetVelocity - VELOCITY_STEP, -MAX_VELOCITY);
        }
        if (gamepad1.aWasPressed()) {
            running = !running;
        }

        // Re-applied every loop so PIDF edits made in Panels take effect immediately.
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
        shooterMotor.setVelocity(running ? targetVelocity : 0);

        telemetry.addData("Running", running);
        telemetry.addData("Target Velocity (ticks/s)", targetVelocity);
        telemetry.addData("Actual Velocity (ticks/s)", shooterMotor.getVelocity());
        telemetry.addData("Power", shooterMotor.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        shooterMotor.setVelocity(0);
    }
}
