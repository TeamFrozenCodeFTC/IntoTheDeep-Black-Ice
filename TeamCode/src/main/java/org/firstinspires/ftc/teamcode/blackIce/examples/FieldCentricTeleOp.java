package org.firstinspires.ftc.teamcode.blackIce.examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.blackIce.Follower;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class FieldCentricTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower.initTeleOp(this);

        waitForStart();

        while (opModeIsActive()) {
            Follower.fieldCentricTeleOpDrive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
            );
        }
    }
}