package org.firstinspires.ftc.teamcode.autonomous.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.Drive;
import org.firstinspires.ftc.teamcode.blackIce.Follower;
import org.firstinspires.ftc.teamcode.blackIce.odometry.Odometry;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
public class EasyPush extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower.initAuto(this);

        waitForStart();

        Odometry.setPosition(0, 0, 0);

        Drive.zeroPowerFloatMode();

//        MovementBuilder.moveThrough(48, 0, 0)
//            .setMaxVelocity(40)
//            .setDriveCorrection(DriveCorrection.velocityConstraints)
//            .build()
//            .waitForMovement();

        double startTime = getRuntime();

//        double previousPower = 0.1;
        double previousXVelocity = 0;
        double previousYVelocity = 0;

        while (opModeIsActive()) {
            Odometry.update();
//            if (Odometry.velocity > 0.1) {
//                double time = System.nanoTime() - startTime;
//                startTime = System.nanoTime();
//                Drive.power(Drive.fieldVectorToLocalWheelPowers(
//                    new double[]{
//                        Odometry.xBrakingDistance - 30*time*1_000_000*Math.signum(Odometry.xVelocity),
//                        Odometry.yBrakingDistance - 30*time*1_000_000*Math.signum(Odometry.yVelocity)
//                    }
//                ));
//            }
            double time = getRuntime() - startTime;
            startTime = getRuntime();
            if (Odometry.velocity > 0.1) {
                Drive.power(Drive.fieldVectorToLocalWheelPowers(
                    new double[]{
                        // * seconds or /seconds?
//                        (Odometry.xVelocity - previousXVelocity) * 0.02,
//                        (Odometry.yVelocity - previousYVelocity) * 0.02,

//                        (Odometry.xVelocity - previousXVelocity) * 0.01,
//                        (Odometry.yVelocity - previousYVelocity) * 0.01

                        Math.min(0.5, (Odometry.xVelocity - previousXVelocity + 30*Math.signum(Odometry.xVelocity)*time*time) * ((double) 1 /60)),
                        Math.min(0.5, (Odometry.yVelocity - previousYVelocity + 30*Math.signum(Odometry.yVelocity)*time*time) * ((double) 1 /60))
                    }
                ));
            }


            //
            // power = previousVelocity * 1/60

            // (velocity - previousVelocity)/0.01
            // 10 - 9.9 = 5power

            // predictedVelocity = previousPower * 60
            // velocity - previousPower * 60
        }
    }} // 20 -

// 15 - 30*0.0009