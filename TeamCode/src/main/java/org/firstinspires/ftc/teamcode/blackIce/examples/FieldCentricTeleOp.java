package org.firstinspires.ftc.teamcode.blackIce.examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;

/**
 * Run a basic field-centric tele-op.
 * Sets motor powers based on the controls and brakes at zero power.
 * Will implement teleOp velocity constraints in the future.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class FieldCentricTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower follower = new Follower(this, new Pose(0,0,0));

        waitForStart();
        
        gamepad1.rumble(1, 0, 500);
        
        while (!isStopRequested()) {
            follower.update();
            telemetry.addData("heading", follower.getMotionState().heading);
            telemetry.addData("field direction vector", new Vector(-gamepad1.left_stick_y,
                -gamepad1.left_stick_x));
            telemetry.addData("robot relative vector", follower.motionState.makeRobotRelative(new Vector(-gamepad1.left_stick_y,
                -gamepad1.left_stick_x)));
            telemetry.update();

            follower.fieldCentricTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
            );
        }
        follower.waitUntilOpModeStop();
        
    }
}