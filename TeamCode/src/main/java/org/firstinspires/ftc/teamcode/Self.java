//
///***********************************************************************
// *                                                                      *
// * OnbotJava Editor is still : beta! Please inform us of any bugs       |
// * on our discord channel! https://discord.gg/e7nVjMM                   *
// * Only BLOCKS code is submitted when in Arena                          *
// *                                                                      *
// ***********************************************************************/
//
//
//public class MyFIRSTJavaOpMode extends LinearOpMode {
//    DcMotor backLeftDrive;
//    DcMotor backRightDrive;
//    DcMotor frontLeftDrive;
//    DcMotor frontRightDrive;
//    DcMotor armTilt;
//    DcMotor armExtend;
//    DcMotor claw;
//    ColorSensor color1;
//    DistanceSensor distance1;
//    BNO055IMU imu;
//
//    private double xPosition = 0.0; // Field-relative X position in inches
//    private double yPosition = 0.0; // Field-relative Y position in inches
//    private double lastEncoderBL = 0.0;
//    private double lastEncoderBR = 0.0;
//    private double lastEncoderFL = 0.0;
//    private double lastEncoderFR = 0.0;
//
//    @Override
//    public void runOpMode() {
//        Gamepad gamepad1 = new FTCGamepad();
//        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
//        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
//        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
//        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
//
//        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
//        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
//
//        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITH_ENCODER);
//        backRightDrive.setMode(DcMotor.RunMode.RUN_WITH_ENCODER);
//        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITH_ENCODER);
//        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITH_ENCODER);
//        armTilt = hardwareMap.get(DcMotor.class, "armTilt");
//        armExtend = hardwareMap.get(DcMotor.class, "armExtend");
//        claw = hardwareMap.get(DcMotor.class, "claw");
//        color1 = hardwareMap.get(ColorSensor.class, "color1");
//        distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(new BNO055IMU.Parameters());
//        // Put initialization blocks here
//        waitForStart();
//        // Put run blocks here
//        while (opModeIsActive()) {
//            // Put loop blocks here
//            // Get current encoder values
//            double currentEncoderBL = backLeftDrive.getCurrentPosition();
//            double currentEncoderBR = backRightDrive.getCurrentPosition();
//            double currentEncoderFL = frontLeftDrive.getCurrentPosition();
//            double currentEncoderFR = frontRightDrive.getCurrentPosition();
//
//            // Calculate changes in encoder values
//            double deltaBLX = (currentEncoderBL - lastEncoderBL);
//            double deltaBRX = (currentEncoderBR - lastEncoderBR);
//            double deltaFLX = (currentEncoderFL - lastEncoderFL);
//            double deltaFRX = (currentEncoderFR - lastEncoderFR);
//            double ticksPerInch = 250;
//            double deltaBL = deltaBLX / ticksPerInch;
//            double deltaBR = deltaBRX / ticksPerInch;
//            double deltaFL = deltaFLX / ticksPerInch;
//            double deltaFR = deltaFRX / ticksPerInch;
//
//            telemetry.addData("lastEncoderBL", lastEncoderBL);
//            telemetry.addData("currentEncoderBL", currentEncoderBL);
//            telemetry.addData("deltaBL", deltaBL);
//
//            // Update last encoder values
//            lastEncoderBL = currentEncoderBL;
//            lastEncoderBR = currentEncoderBR;
//            lastEncoderFL = currentEncoderFL;
//            lastEncoderFR = currentEncoderFR;
//
//            // Calculate robot-centric movement deltas
//            double deltaX = (deltaFL + deltaFR + deltaBL + deltaBR) / 4.0; // Forward/backward
//            double deltaY = (-deltaFL + deltaFR + deltaBL - deltaBR) / 4.0; // Strafe
//
//            // Calculate field-relative movement (assumes gyro heading is available)
//            double robotHeading = imu.getAngle() * (3.14 / 180); // Replace with actual gyro heading method
//            double cosHeading = Math.cos(robotHeading);
//            double sinHeading = Math.sin(robotHeading);
//
//            double fieldDeltaX = deltaX * cosHeading - deltaY * sinHeading;
//            double fieldDeltaY = deltaX * sinHeading + deltaY * cosHeading;
//
//            // Update field-relative position
//            xPosition += fieldDeltaX;
//            yPosition += fieldDeltaY;
//
//            telemetry.addData("deltaX", deltaX);
//
//            // Telemetry for debugging
//            telemetry.addData("X Position (in)", xPosition);
//            telemetry.addData("Y Position (in)", yPosition);
//            telemetry.addData("Heading (deg)", robotHeading);
//            telemetry.addData("position", backLeftDrive.getCurrentPosition());
//            telemetry.update();
//
//            fieldVectorToLocalWheelPowers(10-xPosition, 10-yPosition);
//            sleep(90);
//        }
//    }
//    public double[] fieldVectorToLocalWheelPowers(double x, double y) {
//        // positive heading is counterclockwise
//        double heading = imu.getAngle() * (3.1419 / 180);
//        double cos = Math.cos(heading);
//        double sin = Math.sin(heading);
//        double localForwards = (x * cos + y * sin) / 50; // clockwise rotation
//        double localSlide = (-x * sin + y * cos) / 50;
//
//        double headingPower = 0 ;
//
//        frontLeftDrive.setPower(localForwards-localSlide-headingPower);
//        backLeftDrive.setPower(localForwards+localSlide-headingPower);
//        frontRightDrive.setPower(localForwards+localSlide+headingPower);
//        backRightDrive.setPower(localForwards-localSlide+headingPower);
//    }
//
//
//
//}