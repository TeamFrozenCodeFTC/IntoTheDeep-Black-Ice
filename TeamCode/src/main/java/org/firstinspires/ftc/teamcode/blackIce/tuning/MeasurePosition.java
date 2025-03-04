package org.firstinspires.ftc.teamcode.blackIce.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.Drive;

import org.firstinspires.ftc.teamcode.blackIce.Follower;
import org.firstinspires.ftc.teamcode.blackIce.odometry.Odometry;

/**
 * Find the x, y, and heading of a position on the field. Useful for path making.
 * <p>
 * Start the robot in the corner of the field facing away from the side wall.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "Tuning")
public class MeasurePosition extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower.init(this);

        waitForStart();

        //sweeperRotator.getController().pwmDisable();
//        for (ServoController controller : hardwareMap.getAll(ServoController.class)) {
//            controller.pwmDisable();
//        }
        // plug different servos in different hubs depending on when they need to be disabled

        Drive.zeroPowerFloatMode();

        while (opModeIsActive()) {
            Odometry.update();
            telemetry.addData("x -> ", Odometry.x);
            telemetry.addData("y  ^ ", Odometry.y);
            telemetry.addData("heading", Odometry.heading);

            telemetry.update();
        }
        
    }
}