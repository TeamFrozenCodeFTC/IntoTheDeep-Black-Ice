package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.autonomous.custom.GoBildaPinpointDriver;

public class Odometry {
    public double heading;
    public double x;
    public double y;

    public double velocity;
    public double headingVelocity;
    public double xVelocity;
    public double yVelocity;

    public double xBrakingDistance;
    public double yBrakingDistance;

    public double brakingDistance;

    private double estimateXStoppingDistance() {
        return 0.00130445 * Math.pow(xVelocity, 2) + 0.0644448 * xVelocity + 0.0179835;
        // 0.00439157 * Math.pow(velocity, 2) + 0.0985017 * velocity - 0.0700498;
    }

    private double estimateYStoppingDistance() {
        return 0.00130445 * Math.pow(yVelocity, 2) + 0.0644448 * yVelocity + 0.0179835;
        // 0.00439157 * Math.pow(velocity, 2) + 0.0985017 * velocity - 0.0700498;
    }

    private double estimateStoppingDistance() {
        return 0.00130445 * Math.pow(velocity, 2) + 0.0644448 * velocity + 0.0179835;
        // 0.00439157 * Math.pow(velocity, 2) + 0.0985017 * velocity - 0.0700498;
    }

    public final GoBildaPinpointDriver odometry;

    public Odometry(Robot op) {
        odometry = op.hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        odometry.setOffsets(-36, 0);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odometry.resetPosAndIMU();
    }

    public void update() {
        odometry.update();

        Pose2D pos = odometry.getPosition();
        heading = pos.getHeading(AngleUnit.DEGREES);
        x = pos.getX(DistanceUnit.INCH);
        y = pos.getY(DistanceUnit.INCH);

        Pose2D velocities = odometry.getVelocity();
        xVelocity = velocities.getX(DistanceUnit.INCH);
        yVelocity = velocities.getY(DistanceUnit.INCH);
        velocity = Math.abs(xVelocity) + Math.abs(yVelocity);

        headingVelocity = velocities.getHeading(AngleUnit.DEGREES);

        xBrakingDistance = estimateXStoppingDistance();
        yBrakingDistance = estimateYStoppingDistance();
        brakingDistance = estimateStoppingDistance();
    }

    public void setPosition(double startingHeading, double startingX, double startingY) {
        odometry.setPosition(new Pose2D(
                DistanceUnit.INCH,
                startingX,
                startingY,
                AngleUnit.DEGREES,
                startingHeading
        ));
    }

    public void setHeading(double newHeading) {
        setPosition(newHeading, x, y);
    }

    public void setX(double newX) {
        setPosition(heading, newX, y);
    }

    public void setY(double newY) {
        setPosition(heading, x, newY);
    }

    public void resetHeading() {
        setHeading(0);
    }
}