package org.firstinspires.ftc.teamcode.blackIce.blackIceX;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class BreakSpeedTest extends RobotMovement {
    public double[] getStoppingDistance(double power) {

        goStraightForSeconds(1, power);

        double startingX = odometry.x;
        double xVelocity = Math.abs(odometry.odometry.getVelocity().getX(DistanceUnit.INCH));

        brakeForSeconds(2);

        double newDistance = odometry.x;

        double stoppingDistance = Math.abs(newDistance - startingX);

        return new double[] {xVelocity, stoppingDistance};
    }

    public String stringify(double[] array) {
        return String.format("%.3f", array[0]) + " " + String.format("%.3f", array[1]);
    }

    @Override
    public void runOpMode() {
        initOdometry();
        initWheels();
        waitForStart();

        double points = 10;

        for (int i = 0; i <= points; i++) {
            double direction = i % 2 == 0 ? 1 : -1;

            double[] point = getStoppingDistance(direction * (1-1*(i/points)));
            telemetry.addData(Integer.toString(i), stringify(point));
        }

        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}