package org.firstinspires.ftc.teamcode;

public class ViperSlide {
    Robot op;

    public ViperSlide(Robot op) {
        this.op = op;
    }

    public final static int MAX_TICKS = 4300;
    public double targetTicks;

    private void raise(int ticks) {
        op.sweeperRotator.setPosition(.5);
        op.viperSlideMotor.setTargetPosition(ticks);
        targetTicks = ticks;
        op.viperSlideMotor.setPower(1);
    }

    public void topBasketRaise() {
        raise(4300);
    }

    public void bottomBasketRaise() {
        raise(3000);
    }

    public void topBarRaise() {
        raise(2256);
    }
    public void topBarPull() {
        raise(1400);
    }

    public void bottomBarRaise() {
        raise(644);
    }
    public void bottomBarPull() {
        raise(171);
    }

    public void lower() {
        op.sweeperRotator.setPosition(.5);
        bucketDown();
        op.viperSlideMotor.setTargetPosition(0);

        op.viperSlideMotor.setPower(-1);

        new Thread(() -> {
            long start = System.currentTimeMillis();
            while (op.opModeIsActive() && op.viperSlideMotor.getCurrentPosition() > 10 && (System.currentTimeMillis() - start) < 10000) {

            }
            op.viperSlideMotor.setPower(0);
        }).start();
    }

    public void waitForExtension() {
        while (op.viperSlideMotor.isBusy()
                || op.viperSlideMotor.getCurrentPosition() < targetTicks-10) {

        }

    }

    public void dump() {
        op.viperSlide.clawGrab();
        op.dumperServo.setPosition(0.6);
    }

    public void bucketFlat() {
        op.dumperServo.setPosition(0.6);
    }

    public void bucketDown() {
        op.dumperServo.setPosition(0.2);
    }

    public void clawGrab() {
        op.clawLeft.setPosition(.095);//
        op.clawRight.setPosition(.7);
    }

    public void clawOut() {
        op.clawLeft.setPosition(.5);
        op.clawRight.setPosition(.3);
    }
}