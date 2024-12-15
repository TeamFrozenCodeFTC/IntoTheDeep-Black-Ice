package org.firstinspires.ftc.teamcode.autonomous.custom;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.autonomous.custom.Autonomous;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class TurnTesty extends Autonomous {

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

        double power = 0.2;

        angleLock(power,power,-power,-power);
        // dont normalize lower than max 1

        while (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();

            double currentAngle = pos.getHeading(AngleUnit.DEGREES);
            double change = simplifyAngle(currentAngle - previousAngle);

            telemetry.addData("currentAngle", currentAngle);
            double change2 = 0;//(Math.PI*48*0.0393701)/180;
            telemetry.addData("XCHANGE",-change2);

            telemetry.addData("localX", pos.getX(DistanceUnit.INCH) - change2);
            telemetry.addData("localY", pos.getY(DistanceUnit.INCH));
            telemetry.update();
        }
        stopWheels();

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
//
//    private double[] normalize(double[] a) {
//        double maxPower = Math.max(1.0,
//                Math.max(Math.abs(a[0]),
//                        Math.max(Math.abs(a[1]),
//                                Math.max(Math.abs(a[2]), Math.abs(a[3]))
//                        )));
//        return new double[]{a[0] / maxPower, a[1] / maxPower, a[2] / maxPower, a[3] / maxPower};
//    }

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