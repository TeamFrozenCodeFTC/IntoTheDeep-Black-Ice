//package org.firstinspires.ftc.teamcode.teleOp.lastSeason;
//
//import static org.firstinspires.ftc.teamcode.blackIce.configConstants.Constants.Measurement.HALF_OF_ROBOT;
//import static org.firstinspires.ftc.teamcode.blackIce.configConstants.Constants.Measurement.TILE;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.blackIce.localization.localizers.PinpointLocalizer;
//
//@Config
//@TeleOp
//public class NewControls extends Robot2 {
//    public static double POWER = 3;
//
//    @Override
//    public void runOpMode() {
//        // TODO: priotize turning heading in tele-op over movement
//        initRobot();
//
//        clawLeft.getController().pwmDisable();
//        leftLift.getController().pwmDisable();
//
//        PinpointLocalizer.setPosition(-90+45, TILE + HALF_OF_ROBOT, 0);
//
//        SampleControls specimenControls = new SampleControls(this);
//        NewDriveControls relativeWheelControls = new NewDriveControls(this);
//
//        timer = new ElapsedTime();
//
//        waitForStart();
//
//        timer.reset();
//
//        PinpointLocalizer.setPosition(-90+45, TILE + HALF_OF_ROBOT, 0);
//
//        while (opModeIsActive()) {
//            specimenControls.control();
//            relativeWheelControls.control();
//
//            telemetry.addData("viper slide power", viperSlideMotor.getPower());
//            telemetry.update();
//
//            if (120-timer.seconds() < 35 && 120-timer.seconds() > 34) {
//                gamepad1.rumble(1, 1, 100);
//                gamepad2.rumble(1, 1, 100);
//            }
//
//            loopUpdate();
//        }
//
//    }
//
//}
