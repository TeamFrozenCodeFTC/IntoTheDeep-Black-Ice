package org.firstinspires.ftc.teamcode.blackIceX;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class BreakSpeedTest2 extends BlackIce {
    public double[] getStoppingDistance(double power) {

        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        while (opModeIsActive() && timer.seconds() < 1) {
            updatePosition();
            double turnCorrection = simplifyAngle(0 - robotHeading) * 0.02;

            frontLeftWheel.setPower(power-turnCorrection);
            frontRightWheel.setPower(power+turnCorrection);
            backLeftWheel.setPower(power-turnCorrection);
            backRightWheel.setPower(power+turnCorrection);
        }
        updatePosition();

        double startingX = odometry.getPosition().getX(DistanceUnit.INCH);
        double xVelocity = Math.abs(odometry.getVelocity().getX(DistanceUnit.INCH));

        timer.reset();
        while (opModeIsActive() && timer.seconds() < 2) {
            updatePosition();
            double turnCorrection = simplifyAngle(0 - robotHeading) * 0.01;

            frontLeftWheel.setPower(0);
            frontRightWheel.setPower(0);
            backLeftWheel.setPower(0);
            backRightWheel.setPower(0);
        }
        updatePosition();

        double newDistance = odometry.getPosition().getX(DistanceUnit.INCH);

        double stoppingDistance = Math.abs(newDistance - startingX);

        return new double[] {xVelocity, stoppingDistance};
    }

    public String stringify(double[] array) {
        return String.format("%.3f", array[0]) + " " + String.format("%.3f", array[1]);
    }

    @Override
    public void runOpMode() {
        initOdo();
        initWheels();
        waitForStart();

        double points = 10;

        for (int i = 0; i <= points; i++) {
            double direction = i % 2 == 0 ? 1 : -1;
            // 1-1*(0/10) 1-0
            // 1-1*(10/10) 1-1
            double[] point = getStoppingDistance(direction * (1-1*(i/points)));
            telemetry.addData(Integer.toString(i), stringify(point));
        }
        // 62.29 23.652
        // 58.89 19.62
        // 50.86 17.5
        // 46.26 13.96
        // 41.543 11.49
        // 33.827 7.84
        //27.518 5.817
        // 19.938 3.759
        // 11.681 1.983
        // 6.472 0.484
        // 0 ,0
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}