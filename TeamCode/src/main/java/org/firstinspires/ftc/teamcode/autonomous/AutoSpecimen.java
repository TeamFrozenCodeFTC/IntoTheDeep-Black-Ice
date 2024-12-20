package org.firstinspires.ftc.teamcode.autonomous;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public abstract class AutoSpecimen extends Autonomous {
    double allPower = 0.55;
    double turning = 0.45;

    public void hookFirst() {
        viperSlide.topBarRaise();
        goBackward(24, allPower);
        goBackwardsSeconds(0.7, 0.3);

        hangSpecimen();
    }

    public void observationZonePark() {
        goForward(22, allPower);
        slideLeft(24*2, allPower);
    }

    public void hangSpecimen() {
        viperSlide.waitForExtension();
        viperSlide.topBarPull();
        viperSlide.waitForExtension();
        viperSlide.clawOut();
        viperSlide.lower();
    }
}
