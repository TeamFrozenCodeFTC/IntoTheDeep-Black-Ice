package org.firstinspires.ftc.teamcode.teleOp.lastSeason;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ViperSlide {
    public DcMotor viperSlideMotor;
    public Intake intake;
    public Servo dumperServo;
    public Servo clawLeft;
    public Servo clawRight;

    public ViperSlide(HardwareMap hardwareMap, Intake intake) {
        viperSlideMotor = hardwareMap.get(DcMotor.class, "linearSlide");
        viperSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        viperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideMotor.setTargetPosition(0);
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");
        dumperServo = hardwareMap.get(Servo.class, "dumperServo");
        viperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.intake = intake;
    }

    public int targetTicks;

    boolean lowering = false;

    static final int RELEASE_CLAW_TICKS = 1300;

    public void raise(int ticks) {
        intake.sweeperRotator.setPosition(.65);

        viperSlideMotor.setTargetPosition(ticks);
        targetTicks = ticks;
        viperSlideMotor.setPower(1);
    }

    public void loopUpdate() {
        restMotorAfterLowered();

        releaseClawAtTicks();
//
//        op.viperSlideMotor.setTargetPosition(targetTicks);
    }

    private void releaseClawAtTicks() {
        if (lowering && viperSlideMotor.getCurrentPosition() < RELEASE_CLAW_TICKS) {
            clawOut();
        }
    }

    ElapsedTime loweringTimer = new ElapsedTime();

    private void restMotorAfterLowered() {
        // Powers off the motor when finished lowering
        if (lowering && (viperSlideMotor.getCurrentPosition() < 110 || loweringTimer.seconds() > 5)) {
            lowering = false;
            viperSlideMotor.setPower(0);
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

        viperSlideMotor.setTargetPosition(100); // if you change this you have to change restMotorAfterLowered() ticks
        targetTicks = 100;
        viperSlideMotor.setPower(-1);
    }

    public boolean isExtended() {
        // don't have .isBusy()
        double x = viperSlideMotor.getCurrentPosition();
        return (x > targetTicks-10 && x < targetTicks+10);
    }

    public boolean isExtendedPast(double ticks) {
        // don't have .isBusy()
        double x = viperSlideMotor.getCurrentPosition();
        return (x > ticks-10);
    }

    public void dump() {
        clawGrab();
        dumperServo.setPosition(0.63);
    }

    public void bucketDown() {
        dumperServo.setPosition(0.375);
    }

    public void clawGrab() {
        clawLeft.setPosition(.095);
        clawRight.setPosition(.55);
    }

    public void clawOut() {
        clawLeft.setPosition(.315);
        clawRight.setPosition(.22);
    }

//    public void liftRobot() {
//        leftLift.setPosition(.5);
//        rightLift.setPosition(.4144);
//    }
//
//    public void liftersIn() {
//        leftLift.setPosition(.8425);
//        rightLift.setPosition(.0912);
//    }
}

