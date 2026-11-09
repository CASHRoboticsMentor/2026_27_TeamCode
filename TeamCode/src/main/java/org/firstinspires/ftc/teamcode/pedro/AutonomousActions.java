package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Common mechanism actions for autonomous routines (Shooter, Servos, Intake).
 * Reusable helper that manages non-blocking timers so routines can execute actions cleanly between path steps.
 */
public class AutonomousActions {

    // Default Tuning Constants
    public static double POLLEN_SHOOTER_MAX_SPEED = 2000.0;
    public static double NECTOR_SHOOTER_MAX_SPEED = 500.0;
    public static double INTAKE_MAX_SPEED = 1500.0;
    public static double OUTPUT_MAX_SPEED = 2800.0;

    public static boolean POLLEN_SHOOTER_REVERSE = true;
    public static boolean NECTOR_SHOOTER_REVERSE = false;
    public static boolean REVERSE_INTAKE_DIRECTION = false;

    public static double SERVO_CENTER_DEG = 150.0;
    public static double SERVO_TOTAL_RANGE_DEG = 300.0;
    public static double POLLEN_SERVO_RELATIVE_DEG = 90.0;
    public static double NECTOR_SERVO_RELATIVE_DEG = 90.0;
    public static boolean POLLEN_SERVO_CCW = true;
    public static boolean NECTOR_SERVO_CCW = false;

    public static double INTAKE_P = 10.0, INTAKE_I = 0.0, INTAKE_D = 0.0, INTAKE_F = 12.0;
    public static double SHOOTER_P = 10.0, SHOOTER_I = 0.0, SHOOTER_D = 0.0, SHOOTER_F = 12.0;

    private DcMotorEx intakeMotor;
    private DcMotorEx pollenShooter;
    private DcMotorEx nectorShooter;
    private Servo pollenServo;
    private Servo nectorServo;

    private double actionTimerEndTime = 0;
    private boolean actionInProgress = false;

    public void init(DcMotorEx intake, DcMotorEx pollenShooter, DcMotorEx nectorShooter, Servo pollenServo, Servo nectorServo) {
        this.intakeMotor = intake;
        this.pollenShooter = pollenShooter;
        this.nectorShooter = nectorShooter;
        this.pollenServo = pollenServo;
        this.nectorServo = nectorServo;
        this.actionInProgress = false;
        resetServos();
    }

    private double centerDegreesToCommand() {
        return SERVO_CENTER_DEG / SERVO_TOTAL_RANGE_DEG;
    }

    private double centerRelativeDegreesToCommand(double relativeDegrees, boolean isCCW) {
        double absoluteDegrees = isCCW ? (SERVO_CENTER_DEG + relativeDegrees) : (SERVO_CENTER_DEG - relativeDegrees);
        return Math.max(0.0, Math.min(1.0, absoluteDegrees / SERVO_TOTAL_RANGE_DEG));
    }

    public void resetServos() {
        if (pollenServo != null) pollenServo.setPosition(centerDegreesToCommand());
        if (nectorServo != null) nectorServo.setPosition(centerDegreesToCommand());
    }

    // =========================================================================
    // COMMON AUTONOMOUS MECHANISM FUNCTIONS
    // =========================================================================

    /**
     * Starts nector and pollen shooter motors and waits for spinupTimeSec.
     * Returns true when the spinup timer has completed.
     */
    public boolean startShooterMotors(double spinupTimeSec, double runtime) {
        if (!actionInProgress) {
            if (pollenShooter != null) {
                pollenShooter.setDirection(POLLEN_SHOOTER_REVERSE ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
                pollenShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F));
                pollenShooter.setVelocity(POLLEN_SHOOTER_MAX_SPEED);
            }
            if (nectorShooter != null) {
                nectorShooter.setDirection(NECTOR_SHOOTER_REVERSE ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
                nectorShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F));
                nectorShooter.setVelocity(NECTOR_SHOOTER_MAX_SPEED);
            }
            actionTimerEndTime = runtime + spinupTimeSec;
            actionInProgress = true;
        }

        if (runtime >= actionTimerEndTime) {
            actionInProgress = false;
            return true;
        }
        return false;
    }

    /** Stops pollen and nector shooter motors. */
    public void stopShooterMotors() {
        if (pollenShooter != null) pollenShooter.setVelocity(0);
        if (nectorShooter != null) nectorShooter.setVelocity(0);
    }

    /**
     * Activates pollen servo for activeTimeSec. Returns true and resets servo to center when timer expires.
     */
    public boolean firePollen(double activeTimeSec, double runtime) {
        if (!actionInProgress) {
            if (pollenServo != null) {
                pollenServo.setPosition(centerRelativeDegreesToCommand(POLLEN_SERVO_RELATIVE_DEG, POLLEN_SERVO_CCW));
            }
            actionTimerEndTime = runtime + activeTimeSec;
            actionInProgress = true;
        }

        if (runtime >= actionTimerEndTime) {
            if (pollenServo != null) {
                pollenServo.setPosition(centerDegreesToCommand());
            }
            actionInProgress = false;
            return true;
        }
        return false;
    }

    /**
     * Activates nector servo for activeTimeSec. Returns true and resets servo to center when timer expires.
     */
    public boolean fireNector(double activeTimeSec, double runtime) {
        if (!actionInProgress) {
            if (nectorServo != null) {
                nectorServo.setPosition(centerRelativeDegreesToCommand(NECTOR_SERVO_RELATIVE_DEG, NECTOR_SERVO_CCW));
            }
            actionTimerEndTime = runtime + activeTimeSec;
            actionInProgress = true;
        }

        if (runtime >= actionTimerEndTime) {
            if (nectorServo != null) {
                nectorServo.setPosition(centerDegreesToCommand());
            }
            actionInProgress = false;
            return true;
        }
        return false;
    }

    /**
     * Activates both pollen and nector servos for activeTimeSec. Returns true and resets servos when timer expires.
     */
    public boolean fireBothServos(double activeTimeSec, double runtime) {
        if (!actionInProgress) {
            if (pollenServo != null) {
                pollenServo.setPosition(centerRelativeDegreesToCommand(POLLEN_SERVO_RELATIVE_DEG, POLLEN_SERVO_CCW));
            }
            if (nectorServo != null) {
                nectorServo.setPosition(centerRelativeDegreesToCommand(NECTOR_SERVO_RELATIVE_DEG, NECTOR_SERVO_CCW));
            }
            actionTimerEndTime = runtime + activeTimeSec;
            actionInProgress = true;
        }

        if (runtime >= actionTimerEndTime) {
            resetServos();
            actionInProgress = false;
            return true;
        }
        return false;
    }

    /** Starts intake motor to bring elements in. */
    public void captureElements() {
        if (intakeMotor != null) {
            intakeMotor.setDirection(REVERSE_INTAKE_DIRECTION ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            intakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(INTAKE_P, INTAKE_I, INTAKE_D, INTAKE_F));
            intakeMotor.setVelocity(INTAKE_MAX_SPEED);
        }
    }

    /** Starts intake motor in reverse to push out elements. */
    public void spitOutElements() {
        if (intakeMotor != null) {
            intakeMotor.setDirection(REVERSE_INTAKE_DIRECTION ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
            intakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(INTAKE_P, INTAKE_I, INTAKE_D, INTAKE_F));
            intakeMotor.setVelocity(OUTPUT_MAX_SPEED);
        }
    }

    /** Turns off intake motor. */
    public void stopIntake() {
        if (intakeMotor != null) {
            intakeMotor.setVelocity(0);
        }
    }

    /** Pauses/waits for delaySec. Returns true when delay timer completes. */
    public boolean waitDelay(double delaySec, double runtime) {
        if (!actionInProgress) {
            actionTimerEndTime = runtime + delaySec;
            actionInProgress = true;
        }

        if (runtime >= actionTimerEndTime) {
            actionInProgress = false;
            return true;
        }
        return false;
    }
}
