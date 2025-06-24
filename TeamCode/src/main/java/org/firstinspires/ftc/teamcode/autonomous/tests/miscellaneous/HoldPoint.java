//package org.firstinspires.ftc.teamcode.autonomous.tests.miscellaneous;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.blackIce.robot.Drive;
//import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;
//import org.firstinspires.ftc.teamcode.blackIce.Target;
//import org.firstinspires.ftc.teamcode.blackIce.localization.localizers.PinpointLocalizer;
//
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
//public class HoldPoint extends LinearOpMode {
//    @Override
//    public void runOpMode() {
//        Follower.initAuto(this);
//
//        waitForStart();
//
//        PinpointLocalizer.setPosition(90, 0, 0);
//
//        Drive.zeroPowerFloatMode();
//
////        Movement movement = MovementBuilder.moveThrough(0,0,90)
////            .setDriveCorrection(
////                () -> DrivePowers.scaleToMax(DriveVectors.fieldVectorToLocalWheelPowers(
////                        (Target.xError - Odometry.xBrakingDistance),
////                        (Target.yError - Odometry.yBrakingDistance)
////                ), 1)
////            ).build();
//
//        //movement.start();
//        while (opModeIsActive()) {
//            Target.updatePosition();
//            Follower.telemetry.addData("odometryX", PinpointLocalizer.x);
//            Follower.telemetry.addData("odometryY", PinpointLocalizer.y);
//            Follower.telemetry.addData("targetX", Target.x);
//            Follower.telemetry.addData("targetY", Target.y);
//
//            Follower.telemetry.addData("xPower", (Target.xError - PinpointLocalizer.xBrakingDistance));
//            Follower.telemetry.addData("yPower", (Target.yError - PinpointLocalizer.yBrakingDistance));
//
////            Follower.telemetry.addData("drivePower1",
////                movement.driveCorrection.calculateDrivePowers()[0]);
////            Follower.telemetry.addData("drivePower2",
////                movement.driveCorrection.calculateDrivePowers()[1]);
////            Follower.telemetry.addData("drivePower3",
////                movement.driveCorrection.calculateDrivePowers()[2]);
////            Follower.telemetry.addData("drivePower4",
////                movement.driveCorrection.calculateDrivePowers()[3]);
////
////            Drive.power(movement.driveCorrection.calculateDrivePowers());
//
//            Follower.telemetry.update();
//        }
//    }
//}
