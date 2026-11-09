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
 * Autonomous route and mechanism action sequence for Hive Down, Parking strategy.
 */
public class Hive_Down_Parking implements AutonomousRoutine {

    private static final PoseFactory poseFactory = PoseFactory.degrees();

    // Field Poses
    private final Pose start = poseFactory.of(56, 8, 270);
    private final Pose downParkTarget = poseFactory.of(13.8567, 90.8422, 270);
    private final Pose downParkControl1 = poseFactory.of(35.0, 48.0, 0);

    public Path downParkPath() {
        return Paths.curve(start, downParkControl1, downParkTarget).linear(start, downParkTarget);
    }

    private Follower follower;
    private final AutonomousActions actions = new AutonomousActions();

    private enum Step {
        DRIVE_TO_PARK,
        DONE
    }

    private Step currentStep = Step.DRIVE_TO_PARK;

    @Override
    public Pose getStartPose() {
        return start;
    }

    @Override
    public void init(Follower follower, DcMotorEx intake, DcMotorEx pollenShooter, DcMotorEx nectorShooter, Servo pollenServo, Servo nectorServo) {
        this.follower = follower;
        this.actions.init(intake, pollenShooter, nectorShooter, pollenServo, nectorServo);
        this.currentStep = Step.DRIVE_TO_PARK;
    }

    @Override
    public void start() {
        currentStep = Step.DRIVE_TO_PARK;
        if (follower != null) {
            follower.follow(downParkPath());
        }
    }

    @Override
    public boolean update(Telemetry telemetry, double runtime) {
        switch (currentStep) {
            case DRIVE_TO_PARK:
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
