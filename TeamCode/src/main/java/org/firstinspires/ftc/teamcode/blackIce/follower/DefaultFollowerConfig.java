package org.firstinspires.ftc.teamcode.blackIce.follower;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.blackIce.localization.Localizer;
import org.firstinspires.ftc.teamcode.blackIce.localization.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.blackIce.robot.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blackIce.robot.drivetrain.MecanumDrivetrainConfig;

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
public class DefaultFollowerConfig {
    public static void defaultFollowerConfig(FollowerConfig config) {
        config // follower?
            .distanceUnit(DistanceUnit.INCH)
            .inputAngleUnit(AngleUnit.DEGREES)
            .localizer(Localizer.PINPOINT)
            .drivetrain(Drivetrain.MECANUM) // make drivetrain and config the same
            .drivetrainConfig(new MecanumDrivetrainConfig()
                .frontLeft("frontLeft", DcMotorSimple.Direction.REVERSE)
                .backLeft("backLeft", DcMotorSimple.Direction.REVERSE)
                .frontRight("frontRight", DcMotorSimple.Direction.REVERSE)
                .backRight("backRight", DcMotorSimple.Direction.FORWARD));
    }
}

// ! https://visualizer.pedropathing.com