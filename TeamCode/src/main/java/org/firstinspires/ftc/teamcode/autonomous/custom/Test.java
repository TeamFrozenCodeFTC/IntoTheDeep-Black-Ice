package org.firstinspires.ftc.teamcode.autonomous.custom;

import org.firstinspires.ftc.teamcode.autonomous.custom.Autonomous;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Test extends Autonomous {
    @Override
    public void runOpMode() {
        initOdo();
        initWheels();
        waitForStart();

        //goToPosTurn(0, 10, 0);
        //sleep(1000);
        goToPosTurn(0, -10, 0);
        goToPosTurn(0, 10, 0);
        goToPosTurn(0, -10, 0);
        goToPosTurn(0, 10, 0);

//        goToPosTurn(0, 10, 0);
//        goToPosTurn(0, -10, 0);

//        goToPosTurn(0, 0, 2);
//
//        sleep(1000);
//
//        goToPosTurn(0, 5, 5);
//
//        sleep(1000);
//
//        goToPosTurn(45, 10, 5);
//
//        stopWheels();
    }
}