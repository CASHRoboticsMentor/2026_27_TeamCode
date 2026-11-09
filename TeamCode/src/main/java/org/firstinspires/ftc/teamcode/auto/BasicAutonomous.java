package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.*;

/**
 * Single Autonomous OpMode with interactive pre-match configuration and multi-path routine selection.
 *
 * Select between:
 * - HIVE_UP_GARDEN
 * - HIVE_DOWN_GARDEN
 * - HIVE_UP_PARKING
 * - HIVE_DOWN_PARKING
 */
@Configurable
@Autonomous(name = "Auto: Multi-Path Selector", group = "Competition")
public class BasicAutonomous extends OpMode {

    public enum Routine {
        HIVE_UP_GARDEN,
        HIVE_DOWN_GARDEN,
        HIVE_UP_PARKING,
        HIVE_DOWN_PARKING
    }

    public static Routine selectedRoutine = Routine.HIVE_UP_GARDEN;
    public static int startDelaySec = 0; // 0s, 1s, 2s, 3s, 5s delay

    // Configurable Hardware Names in Panels
    public static String INTAKE_MOTOR_NAME = "intake";
    public static String POLLEN_SHOOTER_NAME = "pollen_shooter";
    public static String NECTOR_SHOOTER_NAME = "nector_shooter";
    public static String POLLEN_SERVO_NAME = "pollen_servo";
    public static String NECTOR_SERVO_NAME = "nector_servo";

    private Follower follower;
    private DcMotorEx intakeMotor;
    private DcMotorEx pollenShooter;
    private DcMotorEx nectorShooter;
    private Servo pollenServo;
    private Servo nectorServo;

    private AutonomousRoutine activeRoutine;
    private int menuIndex = 0; // 0 = Routine, 1 = Delay

    private enum Step {
        PREMATCH_DELAY,
        RUNNING_ROUTINE,
        DONE
    }

    private Step currentStep;
    private double timerEndTime = 0;

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
            pollenServo.setPosition(150.0 / 300.0);
        } catch (Exception e) {
            telemetry.addData("Warning", "Pollen servo '%s' not found", POLLEN_SERVO_NAME);
        }

        try {
            nectorServo = hardwareMap.get(Servo.class, NECTOR_SERVO_NAME);
            nectorServo.setPosition(150.0 / 300.0);
        } catch (Exception e) {
            telemetry.addData("Warning", "Nector servo '%s' not found", NECTOR_SERVO_NAME);
        }

        instantiateSelectedRoutine();

        telemetry.addLine("=== AUTONOMOUS MENU ===");
        telemetry.addLine("Use Gamepad D-PAD Up/Down to navigate, Left/Right to change");
        telemetry.update();
    }

    private void instantiateSelectedRoutine() {
        switch (selectedRoutine) {
            case HIVE_DOWN_GARDEN:
                activeRoutine = new Hive_Down_Garden();
                break;
            case HIVE_UP_PARKING:
                activeRoutine = new Hive_Up_Parking();
                break;
            case HIVE_DOWN_PARKING:
                activeRoutine = new Hive_Down_Parking();
                break;
            case HIVE_UP_GARDEN:
            default:
                activeRoutine = new Hive_Up_Garden();
                break;
        }

        if (activeRoutine != null) {
            activeRoutine.init(follower, intakeMotor, pollenShooter, nectorShooter, pollenServo, nectorServo);
            if (follower != null) {
                follower.setPose(activeRoutine.getStartPose());
            }
        }
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
            menuIndex = (menuIndex - 1 + 2) % 2;
        }
        if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
            menuIndex = (menuIndex + 1) % 2;
        }

        if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed()) {
            changeMenuValue(1);
        }
        if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) {
            changeMenuValue(-1);
        }

        instantiateSelectedRoutine();

        telemetry.addLine("=== PRE-MATCH AUTONOMOUS CONFIG ===");
        telemetry.addLine((menuIndex == 0 ? "> " : "  ") + "Routine: " + selectedRoutine);
        telemetry.addLine((menuIndex == 1 ? "> " : "  ") + "Delay:   " + startDelaySec + " sec");
        telemetry.addLine("===================================");
        telemetry.addLine("Controls: Dpad UP/DOWN = Select, LEFT/RIGHT = Change");
        telemetry.addLine("Press START when ready!");
        telemetry.update();
    }

    private void changeMenuValue(int dir) {
        if (menuIndex == 0) {
            Routine[] routines = Routine.values();
            int nextIdx = (selectedRoutine.ordinal() + dir + routines.length) % routines.length;
            selectedRoutine = routines[nextIdx];
        } else if (menuIndex == 1) {
            startDelaySec = Math.max(0, Math.min(10, startDelaySec + dir));
        }
    }

    @Override
    public void start() {
        timerEndTime = getRuntime() + startDelaySec;
        if (startDelaySec > 0) {
            currentStep = Step.PREMATCH_DELAY;
        } else {
            startRoutine();
        }
    }

    private void startRoutine() {
        currentStep = Step.RUNNING_ROUTINE;
        if (activeRoutine != null) {
            activeRoutine.start();
        }
    }

    @Override
    public void loop() {
        if (follower != null) {
            follower.update();
        }

        switch (currentStep) {
            case PREMATCH_DELAY:
                if (getRuntime() >= timerEndTime) {
                    startRoutine();
                }
                break;

            case RUNNING_ROUTINE:
                if (activeRoutine != null) {
                    boolean finished = activeRoutine.update(telemetry, getRuntime());
                    if (finished) {
                        currentStep = Step.DONE;
                    }
                } else {
                    currentStep = Step.DONE;
                }
                break;

            case DONE:
                break;
        }

        Pose currentPose = follower != null ? follower.pose() : new Pose(0, 0, 0);
        telemetry.addData("Selected Routine", selectedRoutine);
        telemetry.addData("State", currentStep);
        telemetry.addData("X", currentPose.x());
        telemetry.addData("Y", currentPose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(currentPose.heading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            OpModeStorage.autonomousEndPose = follower.pose();
        }
        if (intakeMotor != null) intakeMotor.setVelocity(0);
        if (pollenShooter != null) pollenShooter.setVelocity(0);
        if (nectorShooter != null) nectorShooter.setVelocity(0);
    }
}
