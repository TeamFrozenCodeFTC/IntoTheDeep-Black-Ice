//package org.firstinspires.ftc.teamcode.blackIce.tuning;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Robot;
//
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "Tuning")
//public class VelocityTester extends Robot {
//    public double[] fieldVectorToLocalVector(double x, double y) {
//        // positive heading is counterclockwise
//        double headingRadians = Math.toRadians(odometry.heading);
//        double cos = Math.cos(headingRadians);
//        double sin = Math.sin(headingRadians);
//        double localX = (x * cos + y * sin); // clockwise rotation
//        double localY = (-x * sin + y * cos);
//
//        return new double[] {localX, localY};
//    }
//
//    public double[] localVectorToFieldVector(double x, double y) {
//        // positive heading is counterclockwise
//        double headingRadians = Math.toRadians(odometry.heading);
//        double cos = Math.cos(headingRadians);
//        double sin = Math.sin(headingRadians);
//        double fieldX = (-x * cos - y * sin); // counter clockwise rotation
//        double fieldY = (x * sin - y * cos);
//
//        return new double[] {fieldX, fieldY};
//    }
//
//    @Override
//    public void runOpMode() {
//        initRobot();
//        waitForStart();
//
//        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//        double startingVelocity = 5;
//
//        while (opModeIsActive()) {
//            loopUpdate();
//
//            double xVelocity = odometry.xVelocity + startingVelocity;
//            double yVelocity = odometry.yVelocity + startingVelocity;
//
//            telemetry.addData("field forward x Velocity <->",  xVelocity);
//            telemetry.addData("field lateral y Velocity ^-v", yVelocity);
//
//            double fieldXBrakingDistance = 0.00130445 * Math.pow(xVelocity, 2) + 0.0644448 * xVelocity + 0.0179835;
//            double fieldYBrakingDistance = 0.00130445 * Math.pow(yVelocity, 2) + 0.0644448 * yVelocity + 0.0179835;
//
//            telemetry.addData("fieldX braking distance <->", fieldXBrakingDistance);
//            telemetry.addData("fieldY braking distance ^-v", fieldYBrakingDistance);
//
//            double[] localVelocity = fieldVectorToLocalVector(xVelocity, yVelocity);
//            telemetry.addData("local forward local Velocity X", localVelocity[0]);
//            telemetry.addData("local lateral local Velocity Y", localVelocity[1]);
//
//            double forwardBrakingDistance = 0.00130445 * Math.pow(localVelocity[0], 2) + 0.0644448 * localVelocity[0] + 0.0179835;
//            double lateralBrakingDistance = 0.00130445 * Math.pow(localVelocity[1], 2) + 0.0644448 * localVelocity[1] + 0.0179835;
//
//            telemetry.addData("local forward braking distance", forwardBrakingDistance);
//            telemetry.addData("local lateral braking distance", lateralBrakingDistance);
//
//
//            double[] fieldVelocity = localVectorToFieldVector(forwardBrakingDistance, lateralBrakingDistance);
//
//            telemetry.addData("dispersed fieldX braking distance", fieldVelocity[0]);
//            telemetry.addData("dispersed fieldY braking distance", fieldVelocity[1]);
//
//            telemetry.update();
//        }
//    }
//}
