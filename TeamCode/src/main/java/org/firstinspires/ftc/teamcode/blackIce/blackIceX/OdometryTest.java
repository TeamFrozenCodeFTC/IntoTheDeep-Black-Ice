package org.firstinspires.ftc.teamcode.blackIce.blackIceX;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class OdometryTest extends RobotMovement {
    @Override
    public void runOpMode() {
        initOdometry();
        initWheels();
        waitForStart();

        odometry.setPosition(0,0,0);
        setTarget(-90, 15, 0);

        while (opModeIsActive()) {
            updatePosition();

            double distanceTraveled = totalDistanceToTarget - distanceToTarget;
            double percentageTraveled = Math.max(0, distanceTraveled / totalDistanceToTarget);

            double degreesToTurn = targetHeading - previousHeading;
            double targetTurn = percentageTraveled * degreesToTurn + previousHeading;

            double power = (targetTurn - odometry.heading) * 0.02;


            telemetry.addData("totalDistanceToTarget", totalDistanceToTarget);
            telemetry.addData("distanceToTarget", distanceToTarget);
            telemetry.addData("distanceTraveled", distanceTraveled);
            telemetry.addData("percent", percentageTraveled);
            telemetry.addData("degrees To turn", degreesToTurn);
            telemetry.addData("targetTurn", targetTurn);
            telemetry.addData("power", power);

            telemetry.addData("x", odometry.x);
            telemetry.addData("y", odometry.y);
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