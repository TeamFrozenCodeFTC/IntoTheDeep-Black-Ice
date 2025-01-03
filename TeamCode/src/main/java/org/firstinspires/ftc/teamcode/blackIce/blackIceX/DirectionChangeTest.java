package org.firstinspires.ftc.teamcode.blackIce.blackIceX;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class DirectionChangeTest extends RobotMovement {
    @Override
    public void runOpMode() {
        initOdometry();
        initWheels();
        waitForStart();

        odometry.setPosition(0,0,0);

        updatePosition();

        moveTo(0, 24, 0);

        setTarget(0, 24, -24);

        while (opModeIsActive() && isNotWithinErrorMargin(wideErrorMargin)) {
            if (distanceToTarget < odometry.brakingDistance) {
                break;
            }
            forceTowardTarget2(0,5);
            updatePosition();
        }
        brakeForSeconds(2);
        holdFor(2);

//        // Move away from submersible
//        accelerate(-90, 4, TILE + EDGE_OF_TILE - 1.5);
//        // Turn and Go towards sample
////        LINEAR_INCH_SLOW_DOWN = 10;
//        accelerate(-180, TILE, TILE + EDGE_OF_TILE - 3);
//
//        // 1st sample
//        directionChange(-180, TILE * 2 - 8, TILE + EDGE_OF_TILE + HALF_OF_ROBOT - 2);
//        quickBrakeTo( 90, TILE * 2 - 5, 3);

//
//        ElapsedTime timer = new ElapsedTime();
//        timer.reset();
//        while (opModeIsActive() && isNotWithinErrorMargin(defaultErrorMargin)) {
//            double stoppingDistance = 0;
//
//            if (distanceToTarget <= stoppingDistance && odometry.velocity > CONTROLLABLE_VELOCITY) {
//                // try braking using negative powers
//                updatePosition();
//                brake();
//                continue;
//            }
//
//            double rotation = odometry.heading - 90;
//            double x1 = -(targetX - odometry.x) / LINEAR_INCH_SLOW_DOWN;
//            double y1 = (targetY - odometry.y) / LINEAR_INCH_SLOW_DOWN;
//
//            double cos = Math.cos(Math.toRadians(rotation));
//            double sin = Math.sin(Math.toRadians(rotation));
//            double x = x1 * cos - y1 * sin;
//            double y = x1 * sin + y1 * cos;
//
//            powerWheels(applyCorrection(normalize(new double[] {y-x, y+x, y+x, y-x})));
//
//            telemetry.addData("velocity", odometry.xVelocity);
//            telemetry.addData("x", odometry.x);
//            telemetry.addData("y", odometry.y);
//            telemetry.addData("delta", timer.milliseconds());
//
//            telemetry.update();
//            updatePosition();
//            timer.reset();
//        }
//        holdFor(3);
    }
}