//package org.firstinspires.ftc.teamcode.autonomous.tests.miscellaneous;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;
//import org.firstinspires.ftc.teamcode.blackIce.movement.Movement;
//import org.firstinspires.ftc.teamcode.blackIce.old.BezierCurveOld;
//import org.firstinspires.ftc.teamcode.blackIce.localization.localizers.PinpointLocalizer;
//
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
//public class BezierLineTest extends LinearOpMode {
//    public void buildMovements() {
//
//    }
//
//    @Override
//    public void runOpMode() {
//        Follower.initAuto(this);
//        buildMovements();
//
//        waitForStart();
//
//        PinpointLocalizer.setPosition(90, 0, 0);
//
//        path.waitForMovement();
//    }
//
////    static MovementBuild somePositionBuild = MovementBuilder.stopAtPosition(0, 48, 90);
////    static Movement somePosition = somePositionBuild.build();
////
////    private static final Movement m = MovementBuilder.stopAtPosition(0, 48, 90)
////        .build();
//
//    // Recommended to build all movements and paths outside of runOpMode and as `static final`
//    // and with private keyword if not needed anywhere else
//    private static final Movement path = new BezierCurveOld( // TODO why does this error
//        new double[][]
//            {{ 1.68073593,  0.29761905},
//                {14.74350649, 23.73376623},
//                {55.08441558, 31.03354978},
//                {81.20995671, 28.72835498},
//                {77.36796537,  0.29761905},
//                {56.23701299,  3.75541126},
//                {34.33766234,  3.75541126}}
//    ).build();
//
//}