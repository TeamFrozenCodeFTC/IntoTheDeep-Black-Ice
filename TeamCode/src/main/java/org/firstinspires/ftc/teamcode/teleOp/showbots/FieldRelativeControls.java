////package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//public class FieldRelativeControls {
//
//    public double[] fieldVectorToLocalWheelPowers(double x, double y) {
//        // positive heading is counterclockwise
//        double heading = Math.toRadians(robot.odometry.heading);
//        double cos = Math.cos(heading);
//        double sin = Math.sin(heading);
//        double localForwards = (x * cos + y * sin); // clockwise rotation
//        double localSlide = (-x * sin + y * cos);
//
//        return normalize(new double[]
//                {localForwards-localSlide, localForwards+localSlide,
//                        localForwards+localSlide, localForwards-localSlide}
//        );
//    }
//    public static double[] normalize(double[] a) {
//        double maxPower = 1.0;
//        for (double value : a) {
//            maxPower = Math.max(maxPower, Math.abs(value));
//        }
//
//        return new double[]{
//                a[0] / maxPower,
//                a[1] / maxPower,
//                a[2] / maxPower,
//                a[3] / maxPower
//        };
//    }
//
//    public double[] combine(double[] powers1, double[] powers2) {
//        return normalize(new double[] {
//                powers1[0] + powers2[0], powers1[1] + powers2[1],
//                powers1[2] + powers2[2], powers1[3] + powers2[3]
//        });
//    }
//
//
//    @TeleOp
//    public class NoahsAwesomeRobot extends LinearOpMode {
//        @Override
//        public void runOpMode() throws InterruptedException {
//            // Initializing
//            DcMotor leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
//            DcMotor leftBack = hardwareMap.get(DcMotor.class, "backLeft");
//            DcMotor rightFront = hardwareMap.get(DcMotor.class, "frontRight");
//            DcMotor rightBack = hardwareMap.get(DcMotor.class, "backRight");
//
//            CRServo intake= hardwareMap.get(CRServo.class,"intake");
//
//            // Reverse the direction ofor motors on onw side of the robot
//            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
//            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
//
//            waitForStart();
//
//            while(opModeIsActive()) {
//                double xy = fieldVectorToLocalWheelPowers(gamepad1.right_stick_x, -gamepad1.right_stick_y);
//                double turn = -gamepad1.right_stick_x;
//
//                leftBack.setPower(xy[0]+xy[1]+turn-slide);
//                rightBack.setPower(forward-turn+slide);
//                rightFront.setPower(forward-turn-slide);
//                leftFront.setPower(forward+turn+slide);
//
//
//                if (gamepad1.x){
//                    intake.setPower(1);
//                } else if (gamepad1.a) {
//                    intake.setPower(-1);
//                } else {
//                    intake.setPower(0);
//
//                }
//
//            }
//        }
//    }
//
//}
