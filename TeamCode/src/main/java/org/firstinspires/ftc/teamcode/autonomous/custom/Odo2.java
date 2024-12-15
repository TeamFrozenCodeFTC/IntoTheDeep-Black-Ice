package org.firstinspires.ftc.teamcode.autonomous.custom;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.autonomous.custom.Autonomous;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Odo2 extends Autonomous {

    public void angleLock(
            double frontLeft,
            double backLeft,
            double frontRight,
            double backRight
    ) {
        frontLeftWheel.setPower(frontLeft);
        backLeftWheel.setPower(backLeft);
        frontRightWheel.setPower(frontRight);
        backRightWheel.setPower(backRight);
    }

    @Override
    public void runOpMode() {
        initOdo();
        initWheels();
        waitForStart();

        double x;
        double y;

        double previousAngle = 0;
        double previousX = 0;
        double previousY = 0;
        double sumX = 0;

        double power = 0.3;

        angleLock(power,power,-power,-power);
        sleep(700);
        stopWheels();
        odo.update();
//        odo.setPosition(new Pose2D(DistanceUnit.INCH,0,0,AngleUnit.DEGREES,
//                odo.getPosition().getHeading(AngleUnit.DEGREES)));
        Pose2D pos = odo.getPosition();
        previousX = pos.getX(DistanceUnit.INCH);
        previousY = pos.getY(DistanceUnit.INCH);


        angleLock(power,power,power,power);
        sleep(2000);
        stopWheels();
        // dont normalize lower than max 1

        while (opModeIsActive()) {
            odo.update();
            pos = odo.getPosition();

            double currentAngle = pos.getHeading(AngleUnit.DEGREES);

            telemetry.addData("currentAngle", currentAngle);

            double[] global = convertToGlobal(currentAngle,
                    odo.getEncoderX() * 19.89436789f * 0.0393701 - previousX,
                    odo.getEncoderY() * 19.89436789f * 0.0393701 - previousY);
            x = global[0];
            y = global[1];

            previousX = x;
            previousY = y;

            telemetry.addData("localX", pos.getX(DistanceUnit.INCH));
            telemetry.addData("localY", pos.getY(DistanceUnit.INCH));

            //double imuA = imu.getAngularOrientation().firstAngle;
//            double currentAngle = pos.getHeading(AngleUnit.DEGREES);
//
//            double distanceX = pos.getX(DistanceUnit.INCH) - previousX;
//            double distanceY = pos.getY(DistanceUnit.INCH) - previousY;
//
//            telemetry.addData("distanceX", distanceX);
//            telemetry.addData("distanceY", distanceY);
//            telemetry.addData("previousX", previousX);
//            telemetry.addData("previousY", previousY);
////            double[] add = convertToGlobal1(
////                    -currentAngle, // offset angle
////                    distanceX, distanceY
////            );
//            telemetry.addData("currentAngle", currentAngle);
//            telemetry.addData("imu", imuA);
//            telemetry.addData("addX", add[0]);
//            telemetry.addData("addY", add[1]);
//            telemetry.addData("encoder X", odo.getEncoderX());
//            telemetry.addData("gobildaY", pos.getY(DistanceUnit.INCH));
//            telemetry.addData("gobildaX", pos.getX(DistanceUnit.INCH));
//// rotates the x and y odometry to face real X and Y absolute
//// (odometry needs to be tangent to center of robot) Turning offset (Does not work
//            //x = pos.getX(DistanceUnit.INCH) + degreesTurned * (Math.PI*X_ODOMETRY_RADIUS)/180;
//            //y = pos.getY(DistanceUnit.INCH) - degreesTurned * (Math.PI*Y_ODOMETRY_RADIUS)/180;
//// continuously adds up rotations in the odometry to get an absolute X and Y estimate
//            sumX = previousX + add[0];
//            telemetry.addData("sumX", sumX);
//            if (Math.abs(distanceX) < 1 || Math.abs(distanceY) < 1) {
//                return;
//            }
//            x = previousX + add[0];// + degreesTurned * (Math.PI*X_ODOMETRY_RADIUS)/180;
//            y = previousY + add[1]; //- degreesTurned * (Math.PI*Y_ODOMETRY_RADIUS)/180;
//            previousAngle = currentAngle;
//            previousX = x;
//            previousY = y;

            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.update();
        }

    }

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    public void initOdo() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        odo.setOffsets(-36, 0); //these are tuned for 3110-0002-0001 Product Insight #1

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
    }

    public double[] convertToGlobal(double rotation, double x1, double y1) {
        double cos = Math.cos(Math.toRadians(rotation));
        double sin = Math.sin(Math.toRadians(rotation));
        double xx = x1 * cos - y1 * sin;
        double yy = x1 * sin + y1 * cos;

        return new double[]{xx, yy};
    }

    public double[] normalize(double[] a) {
        double maxPower = Math.max(1.0,
                Math.max(Math.abs(a[0]),
                        Math.max(Math.abs(a[1]),
                                Math.max(Math.abs(a[2]), Math.abs(a[3]))
                        )));
        return new double[]{a[0] / maxPower, a[1] / maxPower, a[2] / maxPower, a[3] / maxPower};
    }

    private double simplifyAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    public void stopWheels() {
        backLeftWheel.setPower(0);
        backRightWheel.setPower(0);
        frontLeftWheel.setPower(0);
        frontRightWheel.setPower(0);
    }
}