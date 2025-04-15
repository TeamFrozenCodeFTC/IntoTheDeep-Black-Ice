package org.firstinspires.ftc.teamcode.blackIce.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blackIce.Follower;

/**
 * Encapsulates the drivetrain of the robot and provides methods
 * to control its motors. It includes initialization, power control, and
 * zero-power behavior settings.
 */
public final class Drive {
    private Drive() {}

    public static DcMotor frontLeftWheel;
    public static DcMotor backLeftWheel;
    public static DcMotor frontRightWheel;
    public static DcMotor backRightWheel;

    private static DcMotor[] motors;

    /**
     * Initializes the drivetrain motors.
     */
    public static void init(HardwareMap hardwareMap) {
        // TODO change these to your actual motor names, and change the directions as needed.
        frontLeftWheel = hardwareMap.get(DcMotor.class, "frontLeft");
        frontLeftWheel.setDirection(DcMotor.Direction.REVERSE);

        backLeftWheel = hardwareMap.get(DcMotor.class, "backLeft");
        backLeftWheel.setDirection(DcMotor.Direction.FORWARD);

        frontRightWheel = hardwareMap.get(DcMotor.class, "frontRight");
        frontRightWheel.setDirection(DcMotor.Direction.FORWARD);

        backRightWheel = hardwareMap.get(DcMotor.class, "backRight");
        backRightWheel.setDirection(DcMotor.Direction.FORWARD);

        motors = new DcMotor[] {frontLeftWheel, backLeftWheel, frontRightWheel, backRightWheel};
    }

    /**
     * Sets the power for each motor in the drivetrain based on the provided
     * DrivePowers object.
     */
    static void power(DrivePowers powers) {
        frontLeftWheel.setPower(powers.frontLeftPower);
        backLeftWheel.setPower(powers.backLeftPower);
        frontRightWheel.setPower(powers.frontRightPower);
        backRightWheel.setPower(powers.backRightPower);
    }

    /**
     * Sets all motors to zero-power float mode, allowing the robot to coast
     * when no power is applied.
     */
    public static void zeroPowerFloatMode() {
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    /**
     * Sets all motors to zero-power brake mode, causing the robot to resist
     * movement when no power is applied.
     */
    public static void zeroPowerBrakeMode() {
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * Sets all motors to zero power.
     */
    public static void zeroPower() {
        power(DrivePowers.forward(0));
    }

    /**
     * Brakes for the given amount of seconds.
     */
    public static void brakeFor(double seconds) {
       zeroPower();

        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        while (Follower.opMode.opModeIsActive() && timer.seconds() < seconds) {
            Follower.opMode.idle();
        }
    }
}