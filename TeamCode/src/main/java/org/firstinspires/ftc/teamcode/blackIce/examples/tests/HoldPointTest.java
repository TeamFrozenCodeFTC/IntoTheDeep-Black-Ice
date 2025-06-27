package org.firstinspires.ftc.teamcode.blackIce.examples.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.controller.PIDController;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;

@Autonomous(group="Black-Ice Examples")
public class HoldPointTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower follower = new Follower(this, new Pose(0,0,0));

        waitForStart();

        while (follower.isFollowingPath() && opModeIsActive()) {
            follower.update();

            follower.drivetrain.driveTowards(
                follower.motionState.makeRobotRelative(new PIDController(0.5, 0, 0).run(
                    new Vector(0,0),
                    follower.motionState.getPredictedStoppedPosition(),
                    follower.motionState.deltaTime
                )),
                (0 - follower.motionState.heading) * 2
            );
            telemetry.addData("position", follower.motionState.position);
            telemetry.addData("predicted position", follower.motionState.getPredictedStoppedPosition());
            telemetry.addData("displacement", follower.motionState.getPredictedStoppedPosition().minus(follower.motionState.position));
            telemetry.addData("field velocity", follower.motionState.fieldRelativeVelocity);
            telemetry.addData("robot velocity", follower.motionState.robotRelativeVelocity);

            telemetry.update();
        }
    }
}