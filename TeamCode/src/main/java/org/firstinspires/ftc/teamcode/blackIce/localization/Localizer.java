package org.firstinspires.ftc.teamcode.blackIce.localization;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.blackIce.localization.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackIce.motion.MotionTracker;

/**
 * Interface for localizing the robot's position, heading, and velocity.
 * Managed by the {@link MotionTracker}.
 *
 * @implNote get methods are only called once per loop in MotionTracker.
 */
public interface Localizer {
    Factory PINPOINT = PinpointLocalizer::new;
    
    double getX();
    double getY();
    double getHeading();
    
    double getFieldVelocityX();
    double getFieldVelocityY();
    
    double getAngularVelocity();
    
    void update();
    void reset();

    void setPose(double x, double y, double heading);
    default void setPose(Pose pose) {
        setPose(pose.getPosition().getX(), pose.getPosition().getY(), pose.getHeading());
    };
    
    default void setX(double x) {
        setPose(x, getY(), getHeading());
    }
    default void setY(double y) {
        setPose(getX(), y, getHeading());
    }
    default void setHeading(double heading) {
        setPose(getX(), getY(), heading);
    }
    
    @FunctionalInterface
    interface Factory {
        Localizer create(HardwareMap map, DistanceUnit distanceUnit);
    }
}
