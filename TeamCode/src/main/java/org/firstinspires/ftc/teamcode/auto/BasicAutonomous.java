package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.PoseFactory;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Minimal Pedro Pathing autonomous template: drive to a scoring spot, then park.
 * Poses below are placeholders - replace them once field positions for this
 * season's game (and the robot's start position) are known.
 *
 * NOTE: This will not run correctly until Constants.create() is filled in
 * after tuning the drivetrain/localizer (see TeamCode/.../pedro/Constants.java).
 */
@Autonomous(name = "Auto: Basic Path Template", group = "Competition")
public class BasicAutonomous extends OpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    // TODO: replace with real start/score/park poses for this season's field.
    private final Pose startPose = poseFactory.of(0, 0, 0);
    private final Pose scorePose = poseFactory.of(24, 24, 0);
    private final Pose parkPose = poseFactory.of(24, 48, 0);

    private enum Step { DRIVE_TO_SCORE, SCORE, DRIVE_TO_PARK, DONE }
    private Step currentStep;

    private Path startToScore() {
        return line(startPose, scorePose).linear(startPose, scorePose);
    }

    private Path scoreToPark() {
        return line(scorePose, parkPose).linear(scorePose, parkPose);
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        currentStep = Step.DRIVE_TO_SCORE;
    }

    @Override
    public void start() {
        follower.follow(startToScore());
    }

    @Override
    public void loop() {
        follower.update();

        switch (currentStep) {
            case DRIVE_TO_SCORE:
                if (!follower.isBusy()) {
                    // TODO: trigger the scoring mechanism here once it exists.
                    currentStep = Step.SCORE;
                }
                break;
            case SCORE:
                // TODO: replace with a real "scoring finished" condition/timer once a mechanism exists.
                currentStep = Step.DRIVE_TO_PARK;
                follower.follow(scoreToPark());
                break;
            case DRIVE_TO_PARK:
                if (!follower.isBusy()) {
                    currentStep = Step.DONE;
                }
                break;
            case DONE:
                break;
        }

        telemetry.addData("Step", currentStep);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        // Hand off our ending position so TeleOp can continue tracking from here.
        OpModeStorage.autonomousEndPose = follower.pose();
    }
}
