package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Standard robot-centric driving TeleOp with intake, dual shooter, and center-relative dual servo control.
 *
 * Gamepad 1:
 *   - Left stick: drive (forward/strafe)
 *   - Right stick X: turn
 *   - Left bumper: toggle slow mode
 *
 * Gamepad 2:
 *   - Left stick: Intake motor control (Negative Y -> Intake [Slower], Positive Y -> Output [Faster]).
 *   - X button: Start both shooter motors (pollen_shooter & nector_shooter).
 *   - Y button: Stop both shooter motors.
 *   - Left Trigger: Rotate pollen_servo relative to center (150°) in CCW/CW direction by POLLEN_SERVO_RELATIVE_DEG.
 *   - Right Trigger: Rotate nector_servo relative to center (150°) in CW/CCW direction by NECTOR_SERVO_RELATIVE_DEG.
 *
 * All public static fields below are live-editable from the Panels web dashboard.
 */
@Configurable
@TeleOp(name = "TeleOp: Robot Centric", group = "Competition")
public class RobotCentricTeleOp extends OpMode {

    // =========================================================================
    // 0) DRIVETRAIN PARAMETERS
    // =========================================================================
    public static double NORMAL_SPEED = 1.0;
    public static double SLOW_SPEED = 0.4;
    public static boolean slowMode = false;

    // =========================================================================
    // 1) NECTOR / POLLEN SHOOTER
    // =========================================================================

    // a) PID Controller Parameters
    public static double SHOOTER_P = 10.0;
    public static double SHOOTER_I = 0.0;
    public static double SHOOTER_D = 0.0;
    public static double SHOOTER_F = 12.0;

    // b) Servo Parameters
    public static String POLLEN_SERVO_NAME = "pollen_servo";
    public static String NECTOR_SERVO_NAME = "nector_servo";
    public static double SERVO_CENTER_DEG = 150.0;       // Center of 300° range (command 0.5)
    public static double SERVO_TOTAL_RANGE_DEG = 300.0;  // Total physical range
    public static double POLLEN_SERVO_RELATIVE_DEG = 90.0; // degrees from center for pollen_servo
    public static double NECTOR_SERVO_RELATIVE_DEG = 90.0; // degrees from center for nector_servo
    public static boolean POLLEN_SERVO_CCW = true;  // true = CCW (+offset from 150°)
    public static boolean NECTOR_SERVO_CCW = false; // false = CW (-offset from 150°)

    // c) Shooter Motor Parameters
    public static String POLLEN_SHOOTER_NAME = "pollen_shooter";
    public static String NECTOR_SHOOTER_NAME = "nector_shooter";
    public static double POLLEN_SHOOTER_MAX_SPEED = 2000.0; // ticks/sec - editable in Panels
    public static double NECTOR_SHOOTER_MAX_SPEED = 500.0;  // ticks/sec - editable in Panels
    public static boolean POLLEN_SHOOTER_REVERSE = true;    // independent direction toggle in Panels
    public static boolean NECTOR_SHOOTER_REVERSE = false;   // independent direction toggle in Panels
    public static boolean shootersRunning = false;

    // =========================================================================
    // 2) INTAKE PARAMETERS
    // =========================================================================
    public static String INTAKE_MOTOR_NAME = "intake";
    public static double INTAKE_MAX_SPEED = 1500.0; // ticks/sec (slower intake speed)
    public static double OUTPUT_MAX_SPEED = 2800.0; // ticks/sec (faster output speed)
    public static boolean reverseIntakeDirection = false;
    public static double INTAKE_P = 10.0;
    public static double INTAKE_I = 0.0;
    public static double INTAKE_D = 0.0;
    public static double INTAKE_F = 12.0;

    private Follower follower;
    private DcMotorEx intakeMotor;
    private DcMotorEx pollenShooter;
    private DcMotorEx nectorShooter;
    private Servo pollenServo;
    private Servo nectorServo;

    @Override
    public void init() {
        try {
            follower = Constants.create(hardwareMap);
        } catch (Exception e) {
            telemetry.addData("Warning", "Follower initialization skipped until Constants.create() is set up");
        }

        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR_NAME);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            telemetry.addData("Warning", "Intake motor '%s' not found", INTAKE_MOTOR_NAME);
        }

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

        telemetry.addLine("Robot Centric TeleOp initialized with Panels support");
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
    public void start() {
        if (follower != null) {
            follower.setPose(OpModeStorage.autonomousEndPose);
            follower.update();
        }
    }

    @Override
    public void loop() {
        // --- Drivetrain Control (Gamepad 1) ---
        if (gamepad1.leftBumperWasPressed()) {
            slowMode = !slowMode;
        }
        double speed = slowMode ? SLOW_SPEED : NORMAL_SPEED;

        double forward = -gamepad1.left_stick_y * speed;
        double lateral = -gamepad1.left_stick_x * speed;
        double turn = -gamepad1.right_stick_x * speed;

        if (follower != null) {
            follower.manual(forward, lateral, turn);
            follower.update();
        }

        // --- Intake Motor Control (Gamepad 2 Left Stick) ---
        if (intakeMotor != null) {
            double stickY = gamepad2.left_stick_y;
            intakeMotor.setDirection(reverseIntakeDirection ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            intakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(INTAKE_P, INTAKE_I, INTAKE_D, INTAKE_F));

            double targetVelocity;
            if (Math.abs(stickY) >= 0.05) {
                double maxSpeed = stickY < 0 ? INTAKE_MAX_SPEED : OUTPUT_MAX_SPEED;
                targetVelocity = -stickY * maxSpeed;
            } else {
                targetVelocity = 0;
            }
            intakeMotor.setVelocity(targetVelocity);
            telemetry.addData("Intake Target Velocity", targetVelocity);
            telemetry.addData("Intake Actual Velocity", intakeMotor.getVelocity());
        }

        // --- Shooter Motors Control (Gamepad 2 X = Start, Y = Stop) ---
        if (gamepad2.xWasPressed()) {
            shootersRunning = true;
        }
        if (gamepad2.yWasPressed()) {
            shootersRunning = false;
        }

        if (pollenShooter != null) {
            pollenShooter.setDirection(POLLEN_SHOOTER_REVERSE ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            pollenShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F));
            pollenShooter.setVelocity(shootersRunning ? POLLEN_SHOOTER_MAX_SPEED : 0);
            telemetry.addData("Pollen Shooter Speed", pollenShooter.getVelocity());
        }

        if (nectorShooter != null) {
            nectorShooter.setDirection(NECTOR_SHOOTER_REVERSE ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            nectorShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F));
            nectorShooter.setVelocity(shootersRunning ? NECTOR_SHOOTER_MAX_SPEED : 0);
            telemetry.addData("Nector Shooter Speed", nectorShooter.getVelocity());
        }

        // --- Dual Servo Controls (Gamepad 2 Triggers - Center-Relative 150°) ---
        // Left Trigger -> pollen_servo rotates from center (150°) in chosen CCW/CW direction by POLLEN_SERVO_RELATIVE_DEG
        if (pollenServo != null) {
            boolean pressed = gamepad2.left_trigger > 0.1;
            double command = pressed ? centerRelativeDegreesToCommand(POLLEN_SERVO_RELATIVE_DEG, POLLEN_SERVO_CCW) : centerDegreesToCommand();
            pollenServo.setPosition(command);
            double currentAngle = command * SERVO_TOTAL_RANGE_DEG;
            telemetry.addData("Pollen Servo Angle", "%.1f° (Cmd: %.3f)", currentAngle, command);
        }

        // Right Trigger -> nector_servo rotates from center (150°) in chosen CW/CCW direction by NECTOR_SERVO_RELATIVE_DEG
        if (nectorServo != null) {
            boolean pressed = gamepad2.right_trigger > 0.1;
            double command = pressed ? centerRelativeDegreesToCommand(NECTOR_SERVO_RELATIVE_DEG, NECTOR_SERVO_CCW) : centerDegreesToCommand();
            nectorServo.setPosition(command);
            double currentAngle = command * SERVO_TOTAL_RANGE_DEG;
            telemetry.addData("Nector Servo Angle", "%.1f° (Cmd: %.3f)", currentAngle, command);
        }

        Pose robotPose = follower != null ? follower.pose() : new Pose(0, 0, 0);
        telemetry.addData("Shooters Running (G2 X=Start, Y=Stop)", shootersRunning);
        telemetry.addData("Slow Mode", slowMode ? "ON" : "off");
        telemetry.addData("X", robotPose.x());
        telemetry.addData("Y", robotPose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(robotPose.heading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        if (intakeMotor != null) intakeMotor.setVelocity(0);
        if (pollenShooter != null) pollenShooter.setVelocity(0);
        if (nectorShooter != null) nectorShooter.setVelocity(0);
    }
}
