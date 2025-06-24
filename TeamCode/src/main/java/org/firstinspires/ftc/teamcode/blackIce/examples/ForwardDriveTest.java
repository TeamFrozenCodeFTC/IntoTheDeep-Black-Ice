package org.firstinspires.ftc.teamcode.blackIce.examples;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;
import org.firstinspires.ftc.teamcode.blackIce.follower.FollowerConfig;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.paths.Path;
import org.firstinspires.ftc.teamcode.blackIce.paths.PathSequenceConstructor;
import org.firstinspires.ftc.teamcode.blackIce.robot.wheelPowers.MecanumPowers;

@TeleOp(group="Black-Ice Examples")
public class ForwardDriveTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower follower = new Follower(this, new Pose(0,0,0));
        follower.whileFollowing(follower::telemetryDebug);

        waitForStart();
        
        gamepad1.rumble(1, 0, 500);
        
        while (!isStopRequested()) {
          //  MecanumPowers.turnCounterclockwise(gamepad1.right_stick_x);
            telemetry.addData("heading", follower.getMotionState().heading);
            telemetry.update();
   
            follower.fieldCentricTeleOpDrive(
                gamepad1.left_stick_x/2,
                -gamepad1.left_stick_y/2,
                -gamepad1.right_stick_x/2
            );
        }
        follower.waitUntilOpModeStop();
        
        // TODO path Maker op-mode
        // TODO motor/servo opmodes (just find online)
    }
}