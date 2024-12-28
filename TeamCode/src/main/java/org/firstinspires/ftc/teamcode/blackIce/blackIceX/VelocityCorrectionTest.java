package org.firstinspires.ftc.teamcode.blackIce.blackIceX;


import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class VelocityCorrectionTest extends RobotMovement {
    @Override
    public void runOpMode() {
        initOdometry();
        initWheels();
        waitForStart();

        odometry.setPosition(0,0,0);
        setTarget(-90, 15, 0);

        while (opModeIsActive()) {
            updatePosition();

            double rotation = -odometry.heading;

            double currentXVel = Math.signum(odometry.xVelocity) * (
                    0.00130445 * Math.pow(odometry.xVelocity, 2)
                            + 0.0644448 * Math.abs(odometry.xVelocity) + 0.0179835);
            double currentYVel = Math.signum(odometry.yVelocity) * (
                    0.00130445 * Math.pow(odometry.yVelocity, 2)
                            + 0.0644448 * Math.abs(odometry.yVelocity) + 0.0179835);

            double x1 = (targetX - odometry.x);
            double y1 = (targetY - odometry.y);

            double cos = Math.cos(Math.toRadians(rotation));
            double sin = Math.sin(Math.toRadians(rotation));
            double x = (x1 * cos - y1 * sin) / 5 + currentXVel;
            double y = (x1 * sin + y1 * cos) / 5 + currentYVel;

            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("x1", x1);
            telemetry.addData("y1", y1);
            telemetry.addData("xVel", currentXVel);
            telemetry.addData("yVel", currentYVel);

            telemetry.addData("odo x", odometry.x);
            telemetry.addData("odo y", odometry.y);
            telemetry.addData("heading", odometry.heading);
//
//            telemetry.addData("frontLeft", powers[0]);
//            telemetry.addData("backLeft", powers[1]);
//            telemetry.addData("frontRight", powers[2]);
//            telemetry.addData("backRight", powers[3]);
//
//            telemetry.addData("frontLeft2", powers2[0]);
//            telemetry.addData("backLeft2", powers2[1]);
//            telemetry.addData("frontRight2", powers2[2]);
//            telemetry.addData("backRight2", powers2[3]);

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