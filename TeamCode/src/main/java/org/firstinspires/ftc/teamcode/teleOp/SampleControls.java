package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

// Uses gamepad2
public class SampleControls {
    Robot op;

    public SampleControls(Robot op) {
        this.op = op;
    }

    void control() {
        linearSlide();
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
        if (op.gamepad2.dpad_up) {
            op.viperSlide.upperChamberRaise();
        }
        else if (op.gamepad2.dpad_left) {
            op.viperSlide.upperBasketRaise();
        }
        else if (op.gamepad2.dpad_right) {
            op.viperSlide.raise(1700);
        }
        else if (op.gamepad2.dpad_down) {
            op.viperSlide.lower();
        }
        else if (op.gamepad2.touchpad_finger_1) {
            ElapsedTime timer = new ElapsedTime();
            op.viperSlide.lower();
            timer.reset();
            while (timer.seconds() < 3) {
                double progress = 1 - timer.seconds() / 3;
                op.viperSlideMotor.setPower(progress);
            }
        }
    }

    void intakeExtender() {
        if (op.gamepad2.right_stick_button) {
            return;
        }

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
//        if (op.gamepad2.right_stick_button) {
//            op.intake.armUp();
//            op.intake.retract();
//            // go back
//        }
        // Sweep Samples out of the side of the submersible.
        if (op.gamepad2.left_stick_button) {
            op.intake.armDown();
            op.intake.spinSweeperBy(-1);
        }
        // Get Samples from submersible
        else if (op.gamepad2.right_stick_y < -0.2) {
            op.intake.armOut();
            //op.sleep(500);
            //op.sweeperRotator.getController().pwmDisable();
        }
        else if (op.gamepad2.right_stick_y > 0.2) {
//            op.intake.armIn();
            op.intake.armIn();
        }
//        else if (op.gamepad2.right_stick_button) {
//            op.intake.armIn();
//        }
    }

    void intakeSweeper() {
        // Sweeper In
        if (op.gamepad2.right_trigger > 0) {
            op.intake.spinSweeperBy(op.gamepad2.right_trigger * 0.75);
        }
        else if (op.gamepad2.left_trigger > 0) {
            op.intake.spinSweeperBy(-op.gamepad2.left_trigger * 0.65);
        }
        else {
            op.sweeper.setPower(0);
        }
        if (op.gamepad2.square) {
            op.viperSlide.clawGrab();
            op.viperSlide.dump();
        }
        else {
            op.viperSlide.bucketDown();
        }
    }
}
