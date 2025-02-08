package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ViperSlide {
    Robot op;

    public ViperSlide(Robot op) {
        this.op = op;
    }

    public double targetTicks;

    boolean lowering = false;

    static final int RELEASE_CLAW_TICKS = 1300;

    public void raise(int ticks) {
        op.sweeperRotator.setPosition(.65);

        op.viperSlideMotor.setTargetPosition(ticks);
        targetTicks = ticks;
        op.viperSlideMotor.setPower(1);
    }

    public void loopUpdate() {
        restMotorAfterLowered();

        releaseClawAtTicks();
    }

    private void releaseClawAtTicks() {
        if (lowering && op.viperSlideMotor.getCurrentPosition() < RELEASE_CLAW_TICKS) {
            op.viperSlide.clawOut();
        }
    }

    ElapsedTime loweringTimer = new ElapsedTime();

    private void restMotorAfterLowered() {
        // Powers off the motor when finished lowering
        if (lowering && (op.viperSlideMotor.getCurrentPosition() < 110 || loweringTimer.seconds() > 5)) {
            lowering = false;
            op.viperSlideMotor.setPower(0);
        }
    }

    public void upperBasketRaise() {
        raise(4400);
    }

    public void bottomBasketRaise() {
        raise(3000);
    }

    public void upperChamberRaise() {
        raise(2000);
    }

    public void upperChamberPull() {
        raise(1300);
    }

    public void bottomBarRaise() {
        raise(644);
    }
    public void bottomBarPull() {
        raise(171);
    }

    public void maxInitRaise() {
        raise(200);
    }

    public void lower() {
        lowering = true;
        loweringTimer.reset();
        bucketDown();

        op.viperSlideMotor.setTargetPosition(100); // if you change this you have to change restMotorAfterLowered() ticks

        op.viperSlideMotor.setPower(-1);
    }

    public void waitForExtension() {
        op.telemetry.addData("waiting", "for slide");
        op.telemetry.update();
        while (op.isNotInterrupted() && !isExtended()) {
            op.idle();
        }
    }

    public boolean isExtended() {
        // don't have .isBusy()
        double x = op.viperSlideMotor.getCurrentPosition();
        return (x > targetTicks-10 && x < targetTicks+10);
    }

    public boolean isExtendedPast(double ticks) {
        // don't have .isBusy()
        double x = op.viperSlideMotor.getCurrentPosition();
        return (x > ticks-10);
    }

    public void dump() {
        op.viperSlide.clawGrab();
        op.dumperServo.setPosition(0.63);
    }

    public void bucketDown() {
        op.dumperServo.setPosition(0.375);
    }

    public void clawGrab() {
        op.clawLeft.setPosition(.095);
        op.clawRight.setPosition(.55);
    }

    public void clawOut() {
        op.clawLeft.setPosition(.5);
        op.clawRight.setPosition(.22);
    }

    public void liftRobot() {
        op.leftLift.setPosition(.5);
        op.rightLift.setPosition(.4144);
    }

    public void liftersIn() {
        op.leftLift.setPosition(.8425);
        op.rightLift.setPosition(.0912);
    }
}

