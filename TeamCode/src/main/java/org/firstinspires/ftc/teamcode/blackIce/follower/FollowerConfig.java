package org.firstinspires.ftc.teamcode.blackIce.follower;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.blackIce.localization.Localizer;
import org.firstinspires.ftc.teamcode.blackIce.paths.MotionProfile;
import org.firstinspires.ftc.teamcode.blackIce.paths.PathBehavior;
import org.firstinspires.ftc.teamcode.blackIce.paths.calculators.BrakingCalculator;
import org.firstinspires.ftc.teamcode.blackIce.paths.calculators.DriveVectorCalculator;

import org.firstinspires.ftc.teamcode.blackIce.paths.calculators.HeadingPowerCalculator;
import org.firstinspires.ftc.teamcode.blackIce.paths.calculators.TranslationalVectorCalculator;
import org.firstinspires.ftc.teamcode.blackIce.paths.calculators.WheelPowersCalculator;
import org.firstinspires.ftc.teamcode.blackIce.paths.pidf.PIDFController;
import org.firstinspires.ftc.teamcode.blackIce.robot.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blackIce.robot.drivetrain.DrivetrainConfig;
import org.firstinspires.ftc.teamcode.blackIce.robot.drivetrain.MecanumDrivetrainConfig;

/**
 * Configuration for a Follower.
 * DefaultFollower -> Follower (OpMode) -> Path
 * <p>
 * Important: Might be able to create a static follower if you don't initialize it with hardware map.
 */
public class FollowerConfig {
    // Note: These are internal defaults. Do not change these here.
    // Use DefaultFollowerConfig instead or change the Follower's config inside an OpMode.
    
    public DistanceUnit distanceUnit = DistanceUnit.INCH;
    public AngleUnit inputAngleUnit = AngleUnit.DEGREES;
    
    public Localizer.Factory localizer = Localizer.PINPOINT;
    
    public DrivetrainConfig drivetrainConfig = new MecanumDrivetrainConfig();
    public Drivetrain.Factory drivetrainFactory = Drivetrain.MECANUM;

    public WheelPowersCalculator wheelPowersCalculator =
        new WheelPowersCalculator(
            MotionProfile.maxSpeed(),
            //MotionProfile.deceleration(60, 60),
            TranslationalVectorCalculator.toClosestPose,
            BrakingCalculator.velocityController,
            HeadingPowerCalculator.maxSpeedProportional,
            PIDFController.createProportionalDerivativeFeedforward(2, 0.1, 0),
            PIDFController.createProportionalDerivativeFeedforward(0.5, 0, 0), // positional PID
            PIDFController.createProportionalDerivativeFeedforward(0.3, 0, 0), // translational PID
            PIDFController.createProportionalDerivativeFeedforward(0.02, 0, 0.015) // velocity
    // PIDF
        );
    
    public PathBehavior defaultPathBehavior = path -> {};

    public FollowerConfig() {
        DefaultFollowerConfig.defaultFollowerConfig(this);
    }
    
    public FollowerConfig distanceUnit(DistanceUnit distanceUnit) {
        this.distanceUnit = distanceUnit;
        return this;
    }
    
    public FollowerConfig addDefaultPathBehavior(PathBehavior defaultPathBehavior) {
        this.defaultPathBehavior.combine(defaultPathBehavior);
        return this;
    }
    
    /**
     * The unit used for input angles.
     * All internal calculations are done in radians, but any unit can be used for input.
     */
    public FollowerConfig inputAngleUnit(AngleUnit inputAngleUnit) {
        this.inputAngleUnit = inputAngleUnit;
        return this;
    }
    
    /**
     * The localizer used to determine the robot's position on the field.
     */
    public FollowerConfig localizer(Localizer.Factory localizer) {
        this.localizer = localizer;
        return this;
    }

    public FollowerConfig drivetrainConfig(DrivetrainConfig drivetrainConfig) {
        this.drivetrainConfig = drivetrainConfig;
        return this;
    }
    
    public FollowerConfig drivetrain(Drivetrain.Factory drivetrain) {
        this.drivetrainFactory = drivetrain;
        return this;
    }
    
    public FollowerConfig wheelPowersCalculator(WheelPowersCalculator wheelPowersCalculator) {
        this.wheelPowersCalculator = wheelPowersCalculator;
        return this;
    }
}
