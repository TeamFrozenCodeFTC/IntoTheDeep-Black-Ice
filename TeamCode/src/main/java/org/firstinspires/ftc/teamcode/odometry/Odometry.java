package org.firstinspires.ftc.teamcode.odometry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.blackIce.Target;

public final class Odometry {
    public static GoBildaPinpointDriver odometry;

    // Private constructor to prevent instantiation
    private Odometry() {}

    public static void init() {
        odometry = robot.hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odometry.setOffsets(-36, 0);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        odometry.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.REVERSED,
            GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        odometry.resetPosAndIMU();

        update();
    }

    private static final Robot robot = Robot.robot;

    public static double heading;
    public static double x;
    public static double y;

    public static double velocity;
    public static double headingVelocity;
    public static double xVelocity;
    public static double yVelocity;

    public static double xBrakingDistance;
    public static double yBrakingDistance;

    public static double forwardBrakingDistance;
    public static double lateralBrakingDistance;

    public static double brakingDistance;

    private static double estimateForwardBrakingDistance() {
        return Math.signum(xVelocity) * 0.00130445 * Math.pow(xVelocity, 2) + 0.0644448 * xVelocity + 0.0179835;
    }

    private static double estimateLateralBrakingDistance() {
        return Math.signum(yVelocity) * 0.00130445 * Math.pow(yVelocity, 2) + 0.0644448 * yVelocity + 0.0179835;
    }
    // reverse rotate ^^^

    private static double estimateXStoppingDistance() {
        if (Math.abs(xVelocity) < 0.01) {
            return 0;
        }
        return Math.signum(xVelocity) * 0.00130445 * Math.pow(xVelocity, 2) + 0.0644448 * xVelocity + 0.0179835;
        // 0.00439157 * Math.pow(velocity, 2) + 0.0985017 * velocity - 0.0700498;
    }

    private static double estimateYStoppingDistance() {
        if (Math.abs(yVelocity) < 0.01) {
            return 0;
        }
        //return 0.00156045 * Math.pow(yVelocity, 2) + 0.0523188 * yVelocity + 0.0317991;
        return Math.signum(yVelocity) *0.00130445 * Math.pow(yVelocity, 2) + 0.0644448 * yVelocity + 0.0179835;
        // 0.00439157 * Math.pow(velocity, 2) + 0.0985017 * velocity - 0.0700498;
    }

    private static double estimateStoppingDistance() {
        return 0.00130445 * Math.pow(velocity, 2) + 0.0644448 * velocity + 0.0179835;
        // 0.00439157 * Math.pow(velocity, 2) + 0.0985017 * velocity - 0.0700498;
    }

    public static void update() {
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

        lateralBrakingDistance = estimateLateralBrakingDistance();
        forwardBrakingDistance = estimateForwardBrakingDistance();
    }

    public static void setPosition(double startingHeading, double startingX, double startingY) {
        odometry.setPosition(new Pose2D(
                DistanceUnit.INCH,
                startingX,
                startingY,
                AngleUnit.DEGREES,
                startingHeading
        ));
        Target.previousHeading = startingHeading;
        Target.previousX = startingX;
        Target.previousY = startingY;
    }

    public static void setHeading(double newHeading) {
        setPosition(newHeading, x, y);
    }

    public static void setX(double newX) {
        setPosition(heading, newX, y);
    }

    public static void setY(double newY) {
        setPosition(heading, x, newY);
    }
}