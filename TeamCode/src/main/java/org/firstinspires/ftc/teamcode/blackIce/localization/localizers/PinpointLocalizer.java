package org.firstinspires.ftc.teamcode.blackIce.localization.localizers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.blackIce.localization.Localizer;


/**
 * A global class that holds the odometry object and provides methods to update the odometry data.
 */
public final class PinpointLocalizer implements Localizer {
    private final GoBildaPinpointDriver odometry;
    private final DistanceUnit distanceUnit;

    private double heading;
    private double x;
    private double y;

    private double angularVelocity;
    private double xVelocity;
    private double yVelocity;

    public PinpointLocalizer(HardwareMap hardwareMap, DistanceUnit distanceUnit) {
        this.distanceUnit = distanceUnit;
        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odometry.setOffsets(-36, 0);
        odometry.setEncoderResolution(
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        odometry.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.REVERSED,
            GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        odometry.resetPosAndIMU(); // TODO test: if we don't do this, the robot could know where it is

        update();
    }
    
    @Override
    public void update() {
        odometry.update();

        Pose2D position = odometry.getPosition();
        x = position.getX(distanceUnit);
        y = position.getY(distanceUnit);
        heading = position.getHeading(AngleUnit.RADIANS);

        Pose2D velocity = odometry.getVelocity();
        xVelocity = velocity.getX(distanceUnit);
        yVelocity = velocity.getY(distanceUnit);
        angularVelocity = velocity.getHeading(AngleUnit.RADIANS);
    }

    @Override
    public double getX() {
        return x;
    }

    @Override
    public double getY() {
        return y;
    }

    @Override
    public double getFieldVelocityX() {
        return xVelocity;
    }

    @Override
    public double getFieldVelocityY() {
        return yVelocity;
    }

    @Override
    public double getHeading() {
        return heading;
    }

    @Override
    public double getAngularVelocity() {
        return angularVelocity;
    }

    public void setPose(double x, double y, double heading) {
        odometry.setPosition(new Pose2D(
            distanceUnit,
            x,
            y,
            AngleUnit.DEGREES,
            heading
        ));
    }

    @Override
    public void reset() {
        odometry.recalibrateIMU();
    }
}