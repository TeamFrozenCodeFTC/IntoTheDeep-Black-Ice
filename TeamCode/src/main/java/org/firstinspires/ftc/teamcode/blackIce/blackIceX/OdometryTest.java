package org.firstinspires.ftc.teamcode.blackIce.blackIceX;


import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class OdometryTest extends RobotMovement {
    @Override
    public void runOpMode() {
        initOdometry();
        initWheels();
        waitForStart();

        odometry.setPosition(-90,0,0);
        targetHeading = -90;
        targetX = 0;
        targetY = 15;

        while (opModeIsActive()) {
            updatePosition();

            double[] powers = normalize(localToGlobal(
                    odometry.heading - 90 ,// + 90, // +90 for starting orientation of hub
                    -(targetX - odometry.x) / LINEAR_INCH_SLOW_DOWN,
                    (targetY - odometry.y) / LINEAR_INCH_SLOW_DOWN
            ));

            telemetry.addData("x", odometry.x);
            telemetry.addData("y", odometry.y);
            telemetry.addData("heading", odometry.heading);

            telemetry.addData("frontLeft", powers[0]);
            telemetry.addData("backLeft", powers[1]);
            telemetry.addData("frontRight", powers[2]);
            telemetry.addData("backRight", powers[3]);

            double[] powers2 = normalize(localToGlobal(
                    odometry.heading - 90,
                    -(targetX - odometry.x) / LINEAR_INCH_SLOW_DOWN,
                    (targetY - odometry.y) / LINEAR_INCH_SLOW_DOWN
            ));

            telemetry.addData("frontLeft2", powers2[0]);
            telemetry.addData("backLeft2", powers2[1]);
            telemetry.addData("frontRight2", powers2[2]);
            telemetry.addData("backRight2", powers2[3]);

//            powerWheels(applyCorrection(normalize(localToGlobal(
//                    odometry.heading + 90, // +90 for starting orientation of hub
//                    -(targetX - odometry.x) / LINEAR_INCH_SLOW_DOWN / 3,
//                    (targetY - odometry.y) / LINEAR_INCH_SLOW_DOWN /3
//            ))));
            //brakeToPosition(-90,0,15);

            telemetry.update();
        }

    }
}