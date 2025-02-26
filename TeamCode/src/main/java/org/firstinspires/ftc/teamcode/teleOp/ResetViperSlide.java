//
//package org.firstinspires.ftc.teamcode.teleOp;
//
//import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.HALF_OF_ROBOT;
//import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.TILE;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Robot;
//
//
//@TeleOp
//public class ResetViperSlide extends Robot {
//    @Override
//    public void runOpMode() {
//        initRobot();
//        intake.armOut();
//
//        viperSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        waitForStart();
//        while (opModeIsActive()) {
//            viperSlideMotor.setPower(gamepad1.left_stick_y);
//
//            loopUpdate();
//        }
//
//    }
//}
