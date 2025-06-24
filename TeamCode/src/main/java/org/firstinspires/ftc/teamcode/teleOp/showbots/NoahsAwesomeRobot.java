//package org.firstinspires.ftc.teamcode.teleOp.showbots;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
//
//
//@TeleOp
//public class NoahsAwesomeRobot extends LinearOpMode {
//    double x;
//    double y;
//    double radians;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//       // Initializing
//        DcMotor leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
//        DcMotor leftBack = hardwareMap.get(DcMotor.class, "backLeft");
//        DcMotor rightFront = hardwareMap.get(DcMotor.class, "frontRight");
//        DcMotor rightBack = hardwareMap.get(DcMotor.class, "backRight");
//
//        BNO055IMU imu = hardwareMap.get(BNO055IMU.class,"imu");
//
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
//
//        imu.initialize(parameters);
//
//        // Reverse the direction ofor motors on onw side of the robot
//        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        waitForStart();
//
//        while(opModeIsActive()) {
//            y = -gamepad1.right_stick_y;
//            double turn = gamepad1.right_stick_x;
//            x = gamepad1.left_stick_x;
//
//            radians = imu.getAngularOrientation(AxesReference.INTRINSIC,
//                AxesOrder.XYZ, AngleUnit.RADIANS).firstAngle;
//
//            DrivePowers x = toRobotVector()
//                .robotVectorToLocalWheelPowers()
//                .add(DrivePowers.turnClockwise(turn))
//                .downscaleMaxTo(0.5);
//
////            leftBack.setPower(forward+turn-slide);
////            rightBack.setPower(forward-turn+slide);
////            rightFront.setPower(forward-turn-slide);
////            leftFront.setPower(forward+turn+slide);
//
//            leftBack.setPower(x.backLeftPower);
//            rightBack.setPower(x.backRightPower);
//            rightFront.setPower(x.frontRightPower);
//            leftFront.setPower(x.frontLeftPower);
//
//        }
//    }
//
//    public Vector toRobotVector() {
//        // positive heading is counterclockwise
//        double localForwards = (x * Math.cos(radians) + y * Math.sin(radians)); // clockwise rotation
//        double localSlide = (x * Math.sin(radians) + y * Math.cos(radians));
//
//        return new Vector(localForwards, localSlide);
//    }
//}
