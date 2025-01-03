package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.ViperSlide;

// Uses gamepad2
public class SampleControls {
    Robot op;

    public SampleControls(Robot op) {
        this.op = op;
    }

    void control() {
        linearSlide();
        dumper();
        intakeExtender();
        intakeArm();
        intakeSweeper();
        claw();
    }

    void claw() {
        if (op.gamepad2.triangle) {
            op.viperSlide.clawGrab();
        }
        if (op.gamepad2.cross) {
            op.viperSlide.clawOut();
        }
    }

    void linearSlide() {
        if (op.gamepad2.left_trigger > 0) {
            op.viperSlide.topBarRaise();
        }
//        else if (op.gamepad2.right_trigger > 0) {
//            op.viperSlide.topBarPull();
//        }
        else if (op.gamepad2.right_bumper) {
            op.viperSlide.topBasketRaise();
        }
        else if (op.gamepad2.dpad_down) {
            op.viperSlide.lower();
        }
    }

    void dumper() {
        if (op.gamepad2.dpad_up) {
            op.viperSlide.dump();
        }
        else {
            op.viperSlide.bucketDown();
        }
    }

    void intakeExtender() {
//        double power = -op.gamepad2.left_stick_y;
//        double position = op.intakeExtender.getCurrentPosition();
//
//        if (position > Intake.MAX_TICKS && power > 0) {
//            power = 0;
//        }
//        if (position < Intake.MIN_TICKS && power < 0) {
//            power = 0;
//        }
//        op.intakeExtender.setPower(power);

        double power = -op.gamepad2.left_stick_y;

        if (power > 0) {
            op.intakeExtender.setTargetPosition(1500);
        }
        if (power < 0) {
            op.intakeExtender.setTargetPosition(0);
        }

        op.intakeExtender.setPower(power);
    }

    void intakeArm() {
        // Once a sample is acquired, this lifts and retracts the intake.
        if (op.gamepad2.left_stick_button) {
            op.intake.armUp();
            op.intake.retract();
            // go back
        }
        // Sweep Samples out of the side of the submersible.
        else if (op.gamepad2.right_stick_button) {
            op.intake.armDown();
            op.intake.spinSweeperOut();
        }
        else if (op.gamepad2.right_stick_y < 0) {
            op.intake.armOut();
            //op.sleep(500);
            //op.sweeperRotator.getController().pwmDisable();
        }
        else if (op.gamepad2.right_stick_y > 0) {
            //op.sweeperRotator.getController().pwmEnable();
            op.intake.armIn();
        }
    }

    void intakeSweeper() {
        if (op.gamepad2.right_bumper) {
            op.intake.spinSweeperIn();
        }
        else if (op.gamepad2.left_bumper) {
            op.intake.spinSweeperOut();
        }
        else {
            op.sweeper.setPower(0);
        }
//        if (op.gamepad2.square) {
//            op.intake.spinSweeperOut();
//        }
//        else if (op.gamepad2.circle) {
//            op.intake.spinSweeperIn();
//        }
//        else {
//            op.sweeper.setPower(0);
//        }
    }
}
