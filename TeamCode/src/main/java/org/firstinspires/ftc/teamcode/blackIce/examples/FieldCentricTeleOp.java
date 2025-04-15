package org.firstinspires.ftc.teamcode.blackIce.examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.blackIce.Follower;

/**
 * Run a basic field-centric tele-op.
 * Sets motor powers based on the controls and brakes at zero power.
 * Will implement teleOp velocity constraints in the future.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class FieldCentricTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower.initTeleOp(this);

        waitForStart();

        while (opModeIsActive()) {
            Follower.fieldCentricTeleOpDrive(
                -gamepad1.left_stick_y, // y is reversed on game-pads
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
            );
        }
    }
}