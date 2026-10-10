package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Standard robot-centric driving TeleOp.
 *
 * Left stick: drive (forward/strafe). Right stick X: turn.
 * Left bumper: toggles slow mode for precision driving.
 *
 * NOTE: This will not run correctly until Constants.create() is filled in
 * after tuning the drivetrain/localizer (see TeamCode/.../pedro/Constants.java).
 */
@TeleOp(name = "TeleOp: Robot Centric", group = "Competition")
public class RobotCentricTeleOp extends OpMode {

    private Follower follower;

    private static final double NORMAL_SPEED = 1.0;
    private static final double SLOW_SPEED = 0.4;
    private boolean slowMode = false;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        telemetry.addLine("Robot Centric TeleOp initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        // Pick up where autonomous left off, if it ran first.
        follower.setPose(OpModeStorage.autonomousEndPose);
        follower.update();
    }

    @Override
    public void loop() {
        if (gamepad1.leftBumperWasPressed()) {
            slowMode = !slowMode;
        }
        double speed = slowMode ? SLOW_SPEED : NORMAL_SPEED;

        double forward = -gamepad1.left_stick_y * speed;
        double lateral = gamepad1.left_stick_x * speed;
        double turn = gamepad1.right_stick_x * speed;

        follower.manual(forward, lateral, turn);
        follower.update();

        Pose robotPose = follower.pose();
        telemetry.addData("Slow Mode", slowMode ? "ON" : "off");
        telemetry.addData("X", robotPose.x());
        telemetry.addData("Y", robotPose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(robotPose.heading()));
        telemetry.update();
    }
}
