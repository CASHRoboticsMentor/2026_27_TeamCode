package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Interface for all Autonomous Path routines.
 * Encapsulates route paths, state machine, and robot mechanism actions for a specific routine.
 */
public interface AutonomousRoutine {

    /** Returns the starting pose for this routine. */
    Pose getStartPose();

    /** Initializes hardware references for this routine. */
    void init(Follower follower, DcMotorEx intake, DcMotorEx pollenShooter, DcMotorEx nectorShooter, Servo pollenServo, Servo nectorServo);

    /** Called once when START is pressed. */
    void start();

    /** Called every loop iteration to run path steps and mechanism actions. Returns true when done. */
    boolean update(Telemetry telemetry, double runtime);
}
