//package org.firstinspires.ftc.teamcode.blackIce.tuning;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.blackIce.Drive;
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.blackIce.DriveCorrections;
//import org.firstinspires.ftc.teamcode.odometry.Odometry;
//
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "Tuning")
//public class VelocityTester extends Robot {
//    @Override
//    public void runOpMode() {
//        initRobot();
//        waitForStart();
//
//        Drive.zeroPowerFloatMode();
//
//        while (opModeIsActive()) {
//            loopUpdate();
//
//            double[] robotVelocity =
//                DriveCorrections.fieldVectorToRobotVector(new double[]{Odometry.xVelocity, Odometry.yVelocity});
//            double[] brakingDistances = DriveCorrections.robotVectorToFieldVector(new double[]{
//                Math.signum(robotVelocity[0]) * 0.00112 * Math.pow(robotVelocity[0], 2) + 0.07316 * robotVelocity[0] + 0.00577,
//                Math.signum(robotVelocity[1]) * 0.00165 * Math.pow(robotVelocity[1], 2) + 0.05054 * robotVelocity[1] + 0.01029}
//            );
//
//            double lateralBrakingDistance = brakingDistances[1];
//            double forwardBrakingDistance = brakingDistances[0];
//
//            telemetry.addData("local forward Velocity", robotVelocity[0]);
//            telemetry.addData("local lateral Velocity", robotVelocity[1]);
//            telemetry.addData("x braking Distance", forwardBrakingDistance);
//            telemetry.addData("y braking distance", lateralBrakingDistance);
//
//            telemetry.update();
//        }
//    }
//}
