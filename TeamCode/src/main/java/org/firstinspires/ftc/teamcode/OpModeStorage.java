package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

// Carries the robot's last known pose from Autonomous into TeleOp (set in each auto's stop(), read in TeleOp's start()).
public class OpModeStorage {
    public static Pose autonomousEndPose = new Pose(0, 0, 0);
}
