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
        velocity = Math.abs(velocities.getX(DistanceUnit.INCH))
                + Math.abs(velocities.getY(DistanceUnit.INCH));

        headingVelocity = velocities.getHeading(AngleUnit.DEGREES);
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