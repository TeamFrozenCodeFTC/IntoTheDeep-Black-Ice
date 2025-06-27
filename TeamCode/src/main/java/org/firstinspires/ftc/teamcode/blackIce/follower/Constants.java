package org.firstinspires.ftc.teamcode.blackIce.follower;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.blackIce.localization.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.blackIce.controller.PIDFController;
import org.firstinspires.ftc.teamcode.blackIce.math.kinematics.QuadraticLinearBrakingModel;
import org.firstinspires.ftc.teamcode.blackIce.math.kinematics.VelocityToStoppingDistanceVectorModel;
import org.firstinspires.ftc.teamcode.blackIce.robot.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.blackIce.robot.drivetrain.MecanumDriveConfig;

/**
 * Set default configuration for all OpModes using the Follower.
 * <p>
 * Note: the drivetrainConfig is required to be set specifically to your robot.
 * <pre><code>
 * .drivetrainConfig(new MecanumDrivetrainConfig()
 *     .frontLeftName("frontLeft")
 *     .backLeftName("backLeft")
 *     .frontRightName("frontRight")
 *     .backRightName("backRight")
 *     .frontLeftDirection(DcMotorSimple.Direction.REVERSE)
 *     .backLeftDirection(DcMotorSimple.Direction.FORWARD)
 *     .frontRightDirection(DcMotorSimple.Direction.FORWARD)
 *     .backRightDirection(DcMotorSimple.Direction.FORWARD))
 * </code></pre>
 */
public class Constants {
    public static FollowerConfig defaultFollowerConfig(OpMode opMode) {
        return new FollowerConfig()
            .localizer(new PinpointLocalizer(opMode.hardwareMap, DistanceUnit.INCH))
            .drivetrain(new MecanumDrive(opMode.hardwareMap, new MecanumDriveConfig()
                .frontLeft("frontLeft", DcMotorSimple.Direction.REVERSE)
                .backLeft("backLeft", DcMotorSimple.Direction.REVERSE)
                .frontRight("frontRight", DcMotorSimple.Direction.REVERSE)
                .backRight("backRight", DcMotorSimple.Direction.FORWARD)
                .maxForwardSpeed(60)
                .maxLateralSpeed(45)))
            .headingPIDF(new PIDFController(2, 0, 0.1, 0))
            .positionalPIDF(new PIDFController(0.5, 0, 0, 0))
            .translationalPIDF(new PIDFController(0.5, 0, 0, 0))
            .driveVelocityPIDF(new PIDFController(0.02, 0, 0, 0.015))
            .brakingDisplacement(new VelocityToStoppingDistanceVectorModel(
                //Drift, braking force
                new QuadraticLinearBrakingModel(0.00112, 0.07316),
                new QuadraticLinearBrakingModel(0.00165, 0.05054)
            ))
            .defaultPathBehavior(path -> {});
    }
}

// ! https://visualizer.pedropathing.com