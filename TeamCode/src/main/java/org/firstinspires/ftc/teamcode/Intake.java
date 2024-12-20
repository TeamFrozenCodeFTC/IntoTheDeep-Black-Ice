package org.firstinspires.ftc.teamcode;

public class Intake {
    Robot op;

    public Intake(Robot op) {
        this.op = op;
    }

    static final double ARM_MIN_POSITION = .35;
    static final double ARM_MAX_POSITION = 1;

    public static final int MAX_TICKS = 3500;
    public static final int TICKS_MARGIN = 100;

    public static final int TICKS_PER_INCH = 445;

    public static final int MIN_TICKS = TICKS_MARGIN;

    public void stopExtender() {
        op.intakeExtender.setPower(0);
    }

    public void moveExtenderInches(double inches) {
        op.intakeExtender.setTargetPosition((int) inches * TICKS_PER_INCH);
        op.intakeExtender.setPower(1);
    }

    public void moveExtenderBack() {
        op.intakeExtender.setTargetPosition(0);
        op.intakeExtender.setPower(1);
    }

    public void waitForExtension() {
        while (op.intakeExtender.isBusy()) {
            op.idle();
        }
    }

    public void armOut() {
        op.sweeperRotator.setPosition(ARM_MAX_POSITION);
    }

    public void armIn() {
        op.sweeperRotator.setPosition(ARM_MIN_POSITION);
    }

    public void spinSweeperIn() {
        op.sweeper.setPower(0.5);
    }

    public void spinSweeperOut() {
        op.sweeper.setPower(-1);
    }

    public void stopSweeper() {
        op.sweeper.setPower(0);
    }
}
