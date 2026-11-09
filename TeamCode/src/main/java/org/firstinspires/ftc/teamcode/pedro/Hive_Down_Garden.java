package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Autonomous route and mechanism action sequence for Hive Down, Garden strategy.
 * Uses AutonomousActions helper for all common mechanism functions.
 */
public class Hive_Down_Garden implements AutonomousRoutine {

    private static final PoseFactory poseFactory = PoseFactory.degrees();

    // Field Poses (Hive Down variant)
    private final Pose start = poseFactory.of(56, 8, 270);
    private final Pose downStart2Shoot = poseFactory.of(58.1815, 25.385, 270);
    private final Pose downShoot2CornerStart = poseFactory.of(58.1815, 25.385, 270);
    private final Pose downShoot2Corner = poseFactory.of(17.7714, 8.0872, 180);
    private final Pose downShoot2CornerControl1 = poseFactory.of(43.9413, 28.2979, 0);
    private final Pose downCorner2EdgeStart = poseFactory.of(17.7714, 8.0872, 180);
    private final Pose downCorner2Edge = poseFactory.of(8.6965, 8.0476, 180);
    private final Pose downEdge2Shoot2Start = poseFactory.of(8.6965, 8.0476, 180);
    private final Pose downEdge2Shoot2 = poseFactory.of(57.9633, 24.8876, 270);
    private final Pose downShoot2TeleStartStart = poseFactory.of(57.9633, 24.8876, 270);
    private final Pose downShoot2TeleStart = poseFactory.of(13.8567, 90.8422, 270);
    private final Pose downShoot2TeleStartControl1 = poseFactory.of(12.8588, 28.9825, 0);

    // Path Generators
    public Path downStart2Shoot() {
        return Paths.line(start, downStart2Shoot).linear(start, downStart2Shoot);
    }

    public Path downShoot2Corner() {
        return Paths.curve(downShoot2CornerStart, downShoot2CornerControl1, downShoot2Corner).linear(downShoot2CornerStart, downShoot2Corner);
    }

    public Path downCorner2Edge() {
        return Paths.line(downCorner2EdgeStart, downCorner2Edge).linear(downCorner2EdgeStart, downCorner2Edge);
    }

    public Path downEdge2Shoot2() {
        return Paths.line(downEdge2Shoot2Start, downEdge2Shoot2).linear(downEdge2Shoot2Start, downEdge2Shoot2);
    }

    public Path downShoot2TeleStart() {
        return Paths.curve(downShoot2TeleStartStart, downShoot2TeleStartControl1, downShoot2TeleStart).linear(downShoot2TeleStartStart, downShoot2TeleStart);
    }

    // Hardware & Common Mechanism Actions Helper
    private Follower follower;
    private final AutonomousActions actions = new AutonomousActions();

    // Routine State Machine Steps
    private enum Step {
        DRIVE_START_2_SHOOT,
        ACTION_SHOOTER_SPINUP,
        ACTION_FIRE_SERVOS,
        DRIVE_SHOOT_2_CORNER,
        DRIVE_CORNER_2_EDGE,
        ACTION_EDGE_DELAY,
        DRIVE_EDGE_2_SHOOT2,
        ACTION_SHOOTER_SPINUP2,
        ACTION_FIRE_SERVOS2,
        DRIVE_SHOOT2_2_TELE,
        DONE
    }

    private Step currentStep = Step.DRIVE_START_2_SHOOT;

    @Override
    public Pose getStartPose() {
        return start;
    }

    @Override
    public void init(Follower follower, DcMotorEx intake, DcMotorEx pollenShooter, DcMotorEx nectorShooter, Servo pollenServo, Servo nectorServo) {
        this.follower = follower;
        this.actions.init(intake, pollenShooter, nectorShooter, pollenServo, nectorServo);
        this.currentStep = Step.DRIVE_START_2_SHOOT;
    }

    @Override
    public void start() {
        currentStep = Step.DRIVE_START_2_SHOOT;
        if (follower != null) {
            follower.follow(downStart2Shoot());
        }
    }

    @Override
    public boolean update(Telemetry telemetry, double runtime) {
        switch (currentStep) {
            case DRIVE_START_2_SHOOT:
                if (follower != null && !follower.isBusy()) {
                    currentStep = Step.ACTION_SHOOTER_SPINUP;
                }
                break;

            case ACTION_SHOOTER_SPINUP:
                // Start shooter motors with 2.0s spinup time
                if (actions.startShooterMotors(2.0, runtime)) {
                    currentStep = Step.ACTION_FIRE_SERVOS;
                }
                break;

            case ACTION_FIRE_SERVOS:
                // Activate both servos for 2.0s
                if (actions.fireBothServos(2.0, runtime)) {
                    actions.stopShooterMotors();
                    actions.captureElements(); // Start intake motor

                    if (follower != null) follower.follow(downShoot2Corner());
                    currentStep = Step.DRIVE_SHOOT_2_CORNER;
                }
                break;

            case DRIVE_SHOOT_2_CORNER:
                if (follower != null && !follower.isBusy()) {
                    if (follower != null) follower.follow(downCorner2Edge());
                    currentStep = Step.DRIVE_CORNER_2_EDGE;
                }
                break;

            case DRIVE_CORNER_2_EDGE:
                if (follower != null && !follower.isBusy()) {
                    currentStep = Step.ACTION_EDGE_DELAY;
                }
                break;

            case ACTION_EDGE_DELAY:
                // 4.0 second delay
                if (actions.waitDelay(4.0, runtime)) {
                    if (follower != null) follower.follow(downEdge2Shoot2());
                    currentStep = Step.DRIVE_EDGE_2_SHOOT2;
                }
                break;

            case DRIVE_EDGE_2_SHOOT2:
                if (follower != null && !follower.isBusy()) {
                    actions.stopIntake();
                    currentStep = Step.ACTION_SHOOTER_SPINUP2;
                }
                break;

            case ACTION_SHOOTER_SPINUP2:
                // Start shooter motors with 2.0s spinup time
                if (actions.startShooterMotors(2.0, runtime)) {
                    currentStep = Step.ACTION_FIRE_SERVOS2;
                }
                break;

            case ACTION_FIRE_SERVOS2:
                // Activate both servos for 2.0s
                if (actions.fireBothServos(2.0, runtime)) {
                    actions.stopShooterMotors();

                    if (follower != null) follower.follow(downShoot2TeleStart());
                    currentStep = Step.DRIVE_SHOOT2_2_TELE;
                }
                break;

            case DRIVE_SHOOT2_2_TELE:
                if (follower != null && !follower.isBusy()) {
                    currentStep = Step.DONE;
                }
                break;

            case DONE:
                break;
        }

        telemetry.addData("Routine Step", currentStep);
        return currentStep == Step.DONE;
    }
}
