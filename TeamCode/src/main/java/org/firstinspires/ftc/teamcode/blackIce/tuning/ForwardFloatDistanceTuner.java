package org.firstinspires.ftc.teamcode.blackIce.tuning;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "Tuning")
public class ForwardFloatDistanceTuner extends Robot {
    public double[] getStoppingDistance(double power) {
        movement.target.setTarget(0,0,0);
        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        while (opModeIsActive() && timer.seconds() < 0.6) {
            loopUpdate();
            drive.power(drive.combine(
                drive.forward(power),
                movement.headingCorrections.getWheelPowers(movement.headingCorrections.locked)));
            telemetry.update();
        }

        double startingX = odometry.x;
        double xVelocity = Math.abs(odometry.xVelocity);

        drive.brakeFor(4);

        double newDistance = odometry.x;

        double stoppingDistance = Math.abs(newDistance - startingX);

        return new double[] {xVelocity, stoppingDistance};
    }

    public String stringify(double[] array) {
        return String.format("%.3f", array[0]) + " " + String.format("%.3f", array[1]);
    }

    @Override
    public void runOpMode() {
        initRobot();

        //floatAllWheels();

        waitForStart();

        double points = 10;

        for (int i = 0; i <= points; i++) {
            double direction = i % 2 == 0 ? 1 : -1;

            double[] point = getStoppingDistance(direction * ((1-1*(i/points)) * 2));
            telemetry.addData(Integer.toString(i), stringify(point));
        }

        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}