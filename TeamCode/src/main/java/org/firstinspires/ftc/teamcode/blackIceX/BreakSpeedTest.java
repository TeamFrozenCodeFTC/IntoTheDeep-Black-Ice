package org.firstinspires.ftc.teamcode.blackIceX;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.ThreePointQuadratic;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class BreakSpeedTest extends BlackIce {

    public static int getSignMultiplier(double number) {
        return (number > 0) ? 1 : (number < 0) ? -1 : 0; // Returns 0 if the number is 0
    }

    public double[] getStoppingDistance(double power) {
        frontLeftWheel.setPower(power);
        frontRightWheel.setPower(power);
        backLeftWheel.setPower(power);
        backRightWheel.setPower(power);
        sleep(1250);
        updatePosition();

        double startingX = odometry.getPosition().getX(DistanceUnit.INCH);
        double xVelocity = getSignMultiplier(power) * odometry.getVelocity().getX(DistanceUnit.INCH);

        telemetry.update();
        frontLeftWheel.setPower(0);
        frontRightWheel.setPower(0);
        backLeftWheel.setPower(0);
        backRightWheel.setPower(0);
        sleep(3000);
        updatePosition();

        double newDistance = odometry.getPosition().getX(DistanceUnit.INCH);

        double stoppingDistance = Math.abs(newDistance - startingX);

        return new double[] {xVelocity, stoppingDistance};
    }

    public double estimate(double vel, double[] quad) {
        return quad[0] * Math.pow(vel, 2) - quad[1] * vel + quad[2];
    }

    @Override
    public void runOpMode() {
        initOdo();
        initWheels();
        waitForStart();

        double[] point1 = getStoppingDistance(1);
        double[] point2 = getStoppingDistance(-0.7);
        double[] point3 = getStoppingDistance(0.3);


        double[] quad = ThreePointQuadratic.findQuadratic(new double[][]{point1, point2, point3});

        telemetry.addData("Vel 1", point1[0]);
        telemetry.addData("Stopping Distance 1", point1[1]);
        telemetry.addData("Estimated 1", estimate(point1[0], quad));
        telemetry.addData("Vel 2", point2[0]);
        telemetry.addData("Stopping Distance 2", point2[1]);
        telemetry.addData("Estimated 2", estimate(point2[0], quad));
        telemetry.addData("Vel 3", point3[0]);
        telemetry.addData("Stopping Distance 3", point3[1]);
        telemetry.addData("Estimated 3", estimate(point3[0], quad));

        telemetry.update();

        // 0.05
        // 2.45
        //12.66

        // 0.195


        // 0.2
        // -0.9
        // 20

        while (opModeIsActive()) {
            idle();
        }
    }
}