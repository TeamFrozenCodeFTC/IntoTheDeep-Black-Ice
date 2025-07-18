package org.firstinspires.ftc.teamcode.blackIce.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;

/**
 * Find the x, y, and heading of a position on the field. Useful for path making.
 * <p>
 * Start the robot in the corner of the field facing away from the side wall.
 */
@TeleOp(group = "Tuning")
public class MeasurePosition extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower follower = new Follower(this, new Pose(0,0,0));

        waitForStart();

        follower.drivetrain.zeroPowerFloatMode();

        while (opModeIsActive()) {
            follower.updateMotionState();
            telemetry.addData("x -> ", follower.getMotionState().position.getX());
            telemetry.addData("y  ^ ", follower.getMotionState().position.getY());
            telemetry.addData("robotVelocityX", follower.getMotionState().robotRelativeVelocity.getX());
            telemetry.addData("robotVelocityY", follower.getMotionState().robotRelativeVelocity.getY());
            telemetry.addData("fieldVelocityX", follower.getMotionState().velocity.getX());
            telemetry.addData("fieldVelocityY", follower.getMotionState().velocity.getY());
            telemetry.addData("heading", follower.getMotionState().heading);

            telemetry.update();
        }
        
        // make buttons put position into telemetry and then auto make paths
    }
}
//sweeperRotator.getController().pwmDisable();
//        for (ServoController controller : hardwareMap.getAll(ServoController.class)) {
//            controller.pwmDisable();
//        }
// plug different servos in different hubs depending on when they need to be disabled
