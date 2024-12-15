package org.firstinspires.ftc.teamcode.autonomous.custom;

import org.firstinspires.ftc.teamcode.autonomous.custom.Autonomous;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class MultiMovement extends Autonomous {
    @Override
    public void runOpMode() {
        initOdo();
        initWheels();
        waitForStart();

        goToPosTurn(0, -20, 0);
        goToPosTurn(180, 20, 0);

    }
}