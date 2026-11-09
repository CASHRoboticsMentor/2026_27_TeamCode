package org.firstinspires.ftc.teamcode.test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Standalone bench test for dual shooter motors: pollen_shooter and nector_shooter.
 *
 * Gamepad 2 (or Gamepad 1):
 * - X button: Start both shooter motors.
 * - Y button: Stop both shooter motors.
 *
 * Panels Dashboard Live Controls:
 * - POLLEN_SHOOTER_MAX_SPEED & NECTOR_SHOOTER_MAX_SPEED: Tune max speeds independently.
 * - POLLEN_SHOOTER_REVERSE & NECTOR_SHOOTER_REVERSE: Toggle directions independently.
 */
@Configurable
@TeleOp(name = "Test: Shooter Motor", group = "Test")
public class ShooterMotorTest extends OpMode {

    // Configurable motor names in Panels
    public static String POLLEN_SHOOTER_NAME = "pollen_shooter";
    public static String NECTOR_SHOOTER_NAME = "nector_shooter";

    // Configurable shooter speeds in Panels
    public static double POLLEN_SHOOTER_MAX_SPEED = 2000.0; // ticks/sec - editable in Panels
    public static double NECTOR_SHOOTER_MAX_SPEED = 2000.0; // ticks/sec - editable in Panels

    // Independent direction toggles in Panels
    public static boolean POLLEN_SHOOTER_REVERSE = false;
    public static boolean NECTOR_SHOOTER_REVERSE = false;

    // Configurable state
    public static boolean shootersRunning = false;

    // PIDF tuning parameters for velocity control
    public static double P = 10.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 12.0;

    private DcMotorEx pollenShooter;
    private DcMotorEx nectorShooter;

    @Override
    public void init() {
        try {
            pollenShooter = hardwareMap.get(DcMotorEx.class, POLLEN_SHOOTER_NAME);
            pollenShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            pollenShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pollenShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            telemetry.addData("Warning", "Pollen shooter motor '%s' not found", POLLEN_SHOOTER_NAME);
        }

        try {
            nectorShooter = hardwareMap.get(DcMotorEx.class, NECTOR_SHOOTER_NAME);
            nectorShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            nectorShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            nectorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            telemetry.addData("Warning", "Nector shooter motor '%s' not found", NECTOR_SHOOTER_NAME);
        }

        telemetry.addLine("Dual Shooter Motor Test initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Controls: Gamepad 2 or Gamepad 1 (X = Start, Y = Stop)
        if (gamepad2.xWasPressed() || gamepad1.xWasPressed()) {
            shootersRunning = true;
        }
        if (gamepad2.yWasPressed() || gamepad1.yWasPressed()) {
            shootersRunning = false;
        }

        if (pollenShooter != null) {
            pollenShooter.setDirection(POLLEN_SHOOTER_REVERSE ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            pollenShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
            pollenShooter.setVelocity(shootersRunning ? POLLEN_SHOOTER_MAX_SPEED : 0);
            telemetry.addData("Pollen Actual Speed", pollenShooter.getVelocity());
        }

        if (nectorShooter != null) {
            nectorShooter.setDirection(NECTOR_SHOOTER_REVERSE ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            nectorShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
            nectorShooter.setVelocity(shootersRunning ? NECTOR_SHOOTER_MAX_SPEED : 0);
            telemetry.addData("Nector Actual Speed", nectorShooter.getVelocity());
        }

        telemetry.addData("Shooters Running (X=Start, Y=Stop)", shootersRunning);
        telemetry.addData("Pollen Target Speed", POLLEN_SHOOTER_MAX_SPEED);
        telemetry.addData("Nector Target Speed", NECTOR_SHOOTER_MAX_SPEED);
        telemetry.addData("Pollen Reverse", POLLEN_SHOOTER_REVERSE);
        telemetry.addData("Nector Reverse", NECTOR_SHOOTER_REVERSE);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (pollenShooter != null) pollenShooter.setVelocity(0);
        if (nectorShooter != null) nectorShooter.setVelocity(0);
    }
}
