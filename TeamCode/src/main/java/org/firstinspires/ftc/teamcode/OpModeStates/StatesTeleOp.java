package org.firstinspires.ftc.teamcode.OpModeStates;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.HardwareClassState.HangerStates;
import org.firstinspires.ftc.teamcode.HardwareClassState.IntakeStates;
import org.firstinspires.ftc.teamcode.HardwareClassState.LinearSlidesStates;
import org.firstinspires.ftc.teamcode.HardwareClassState.ShooterStates;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
@Config
public class StatesTeleOp extends OpMode {

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();


    LinearSlidesStates slides = new LinearSlidesStates();
    IntakeStates intake = new IntakeStates();
    ShooterStates shooter = new ShooterStates();
    HangerStates hanger = new HangerStates();
    VisionPortal visionPortal;
    WebcamName webcamBack;
    AprilTagProcessor aprilTag;
    AprilTagProcessor myAprilTagProcessor;
    List<AprilTagDetection> myAprilTagDetections;  // list of all detections// current detection in for() loop
    int myAprilTagIdCode;

    boolean intakeFlipInit = true;
    boolean faceUpFlipInit = true;
    boolean ClawLeftInit = true;
    boolean ClawRightInit = true;
    boolean intakeArmDown = true;
    boolean shooterInit = true;

    boolean buzzable = true;
    boolean buzzableDrop = false;

    public static double backMulti = 0.8;

    public DcMotorEx backLeftMotor;
    public DcMotorEx backRightMotor;
    public DcMotorEx frontLeftMotor;
    public DcMotorEx frontRightMotor;

    public DistanceSensor distanceSensor;

    public static double DEAD_ZONE_AMOUNT = 0.05;

    IMU imu;

    double botHeading;
    double setX;
    double setY;

    double chassis_multi = 1;

    double armPosLeft = intake.getLeftStackCycle();
    double armPosRight = intake.getRightStackCycle();

    double clawPos = intake.getFlipUp();


    boolean moveArm = false;
    boolean moveDown = false;

    boolean moveArmBack = false;

    boolean armUp = false;
    boolean armFaceUp = false;
    ElapsedTime armTimer = new ElapsedTime();

    ElapsedTime extraTime = new ElapsedTime();
    ElapsedTime slideTimer = new ElapsedTime();

    double prevPositionLeft = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


//        webcamBack = hardwareMap.get(WebcamName.class, "backCam");
//
//        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
//
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(webcamBack)
//                .addProcessor(aprilTag)
//                .build();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );

        backLeftMotor = hardwareMap.get(DcMotorEx.class, "leftRear");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "rightRear");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");

//        distanceSensor = hardwareMap.get(DistanceSensor.class, "DistanceSensor");

        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);

        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        slides.init(hardwareMap);
        hanger.init(hardwareMap);
//        sweeper.init(hardwareMap);
//
        shooter.shooterInit();
        intake.flipClawUp();
        intake.closeClawLeft();
        intake.closeClawRight();
//        sweeper.sweepIn();
//        intake.armOffGround();
//        intake.flipPurpleUp();

    }

    @Override
    public void loop() {
        if(currentGamepad2.b && !previousGamepad2.b){
            if (armFaceUp){
                if(faceUpFlipInit){
                    clawPos = intake.getFlipFaceUpSingle();
                }
                else{
                    clawPos = intake.getFlipFaceUp();
                }
                faceUpFlipInit = !faceUpFlipInit;
            }
            else {
                if (intakeFlipInit) {
                    clawPos = intake.getFlipDown();
                } else {
                    clawPos = intake.getFlipUp();
                }
                intakeFlipInit = !intakeFlipInit;
            }
        }
        if((currentGamepad1.right_trigger > 0.3 && !(previousGamepad1.right_trigger > 0.3))){
            if(ClawLeftInit){
                if (armFaceUp) intake.openWideClawL();
                else intake.openClawLeft();
            }
            else{
                intake.closeClawLeft();
            }
            ClawLeftInit = !ClawLeftInit;
        }
        if((currentGamepad1.left_trigger > 0.3 && !(previousGamepad1.left_trigger > 0.3))){
            if(ClawRightInit){
                if (armFaceUp) intake.openWideClawR();
                else intake.openClawRight();
            }
            else{
                intake.closeClawRight();
            }
            ClawRightInit = !ClawRightInit;
        }
        if(currentGamepad2.a && !previousGamepad2.a && !armUp){
//            checkDistance(false);
            slides.resetLiftEncoder();
            slides.liftSlides();
            intakeFlipInit = true;
            intake.setUpServoProfile(armPosLeft, armPosRight);
            clawPos = intake.getFlipUp();
            armUp = true;
            buzzableDrop = true;
            moveArmBack = true;
            armTimer.reset();
        }
        if (currentGamepad2.x && !previousGamepad2.x){
//            checkDistance(true);
            if (armUp && slides.ExtensionLeft.getCurrentPosition() < 1700){
                slides.resetLiftEncoder();
                slides.moveSlides(0.5);
            }
            moveDown = true;
            intakeFlipInit = true;
            buzzableDrop = false;
            if (!armFaceUp)
                clawPos = intake.getFlipUp();
            armUp = false;
            armFaceUp = false;
            slideTimer.reset();
        }
        if (currentGamepad2.y && !previousGamepad2.y && !armUp){
//            checkDistance(false);
            intake.setUpServoProfileFaceUp(armPosLeft, armPosRight);
            armUp = true;
            buzzableDrop = true;
            moveArm = true;
            moveArmBack = true;
            armFaceUp = true;
            faceUpFlipInit = true;
            armTimer.reset();
        }
        if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down){
            intakeFlipInit = true;
            armPosLeft = intake.getLeftStackCycle();
            armPosRight = intake.getRightStackCycle();
            clawPos = intake.getFlipUp();
        }
        if(currentGamepad1.b && !previousGamepad1.b){
            shooter.shooterOut();
        }
        if(currentGamepad2.left_bumper && !previousGamepad2.left_bumper){
            armPosLeft = intake.getLeftDown();
            armPosRight = intake.getRightDown();
            clawPos = intake.getFlipDown();
            armTimer.reset();
            ClawLeftInit = false;
            ClawRightInit = false;
            intakeFlipInit = false;
            buzzable = true;
        }
        if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up){
            slides.resetLiftEncoder();

            slides.ExtensionRight.setPower(0);
            slides.ExtensionLeft.setPower(0);
            moveDown = false;
        }
//        if (currentGamepad2.x && !previousGamepad2.x){
//            if (slides.maxPower == 1){
//                slides.maxPower = 0.5;
//            }
//            else{
//                slides.maxPower = 1;
//            }
//        }
//
//        if(currentGamepad1.x && !previousGamepad1.x){
//            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));
//            TrajectorySequence backOff = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
//                    .setAccelConstraint((v, pose2d, pose2d1, pose2d2) -> 20)
//                    .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> 20)
//                    .addTemporalMarker(0, () -> {
//                        slides.resetLiftEncoder();
//                        slides.liftSlides();
//                    })
//                    .lineTo(new Vector2d(0, 2))
//                    .addTemporalMarker(0.5, () -> {
//                        intake.flipFace2();
//                        slides.resetSlides();
//                    })
//                    .lineTo(new Vector2d(2, 2))
//                    .build();
//            drive.followTrajectorySequence(backOff);
//        }
//
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;


        if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left){
            boolean isSuccessful = slides.decreaseLevel();
            if (isSuccessful){
                gamepad2.rumbleBlips((slides.leftPreset-300)/300+1);
            }
        }

        if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right){
            boolean isSuccessful = slides.increaseLevel();
            if (isSuccessful){
                gamepad2.rumbleBlips((slides.leftPreset - 300) / 300 + 1);
            }
        }

        if (buzzable && ClawLeftInit && ClawRightInit){
            gamepad2.rumble(1000);
            buzzable = false;
        }

        if (buzzable && armTimer.seconds() > 0.4){
            intake.openWideClawL();
            intake.openWideClawR();
            buzzable = false;
        }

        if (buzzableDrop && !ClawLeftInit && !ClawRightInit){
            gamepad2.rumbleBlips(2);
            buzzableDrop = false;
        }
//
        findBotHeading();
        RBReset();
        slowmode();
        triggersTurnServo();
        dpadClawWrist();

        if(!slides.isLifting && !moveDown) {
            double power = -gamepad2.right_stick_y;
            if (slides.ExtensionLeft.getCurrentPosition() > 2100 && power > 0)
                power = 0;
            slides.moveSlides(power);
        }
            hanger.hangerMotorLeft.setPower(-gamepad2.left_stick_y);
            hanger.hangerMotorRight.setPower(-gamepad2.left_stick_y);

        if (moveArm && armTimer.seconds() > 0.3){
            clawPos = intake.getFlipFaceUp();
            moveArm = false;
        }


        if (moveDown && slideTimer.seconds() > 0.1){
            if (!moveArmBack){
                armTimer.reset();
                intake.setUpServoProfileDown(armPosLeft, armPosRight);
                moveArmBack = true;
            }
            if (slideTimer.seconds() > 0.4){
                clawPos = intake.getFlipUp();
                slides.moveSlides(-1);
                if (slides.ExtensionLeft.getCurrent(CurrentUnit.AMPS) > 3.5 && slideTimer.seconds() > 1.5){
                    slides.moveSlides(0);
                    moveDown = false;
                }
            }
        }
//
        if (moveArmBack){
            telemetry.addData("Finished", intake.armProfileL.isFinished(armTimer.seconds()));
            intake.updateState(armTimer.seconds());
            armPosLeft = intake.armStateL.position;
            armPosRight = intake.armStateR.position;
            if (!intake.armProfileL.isFinished(armTimer.seconds()) && armTimer.seconds() < 4){
                prevPositionLeft = armPosLeft;
            }
            else{
                moveArmBack = false;
            }
        }

        if (currentGamepad1.y && currentGamepad1.y){
            intakeFlipInit = true;
            armPosLeft = intake.getStackLeft();
            armPosRight = intake.getStackRight();
            clawPos = intake.getStackFlip();
        }
//
        slides.checkMode();

//
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        setX = rotX;
        setY = rotY;

        if (Math.abs(y) < DEAD_ZONE_AMOUNT) { //y deadzone
            y = 0;
        }
        if (Math.abs(x) < DEAD_ZONE_AMOUNT) { //x deadzone
            x = 0;
        }
        if (Math.abs(rx) < DEAD_ZONE_AMOUNT){ //rx deadzone
            rx = 0;
        }

        if (slides.ExtensionLeft.getCurrentPosition() > 2100){
            slides.moveSlides(0);
        }


        //april tag
//        myAprilTagDetections = aprilTag.getDetections();
//        boolean gotTag = false;
//        for (AprilTagDetection detection : myAprilTagDetections) {
//
//            if (detection.metadata != null && !gotTag) {  // This check for non-null Metadata is not needed for reading only ID code.
//                gotTag = true;
//                myAprilTagIdCode = detection.id;
//                range = Math.sqrt(Math.pow(detection.ftcPose.x, 2) + Math.pow(detection.ftcPose.y, 2));
//                bearing = detection.ftcPose.bearing;
//                yaw = detection.ftcPose.yaw;
//
//                telemetry.addData("id", myAprilTagIdCode);
//                telemetry.addData("name", detection.metadata.name);
//                telemetry.addData("bearing", detection.ftcPose.bearing);
//                telemetry.addData("elevation", detection.ftcPose.elevation);
//                telemetry.addData("range", detection.ftcPose.range);
//                telemetry.addData("yaw", detection.ftcPose.yaw);
//            }
//        }
//        telemetry.addData("GOT TAG??", gotTag);
//        if (gotTag) {
//            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
//                alignAprilTag(myAprilTagIdCode, "Middle");
//            } else if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
//                alignAprilTag(myAprilTagIdCode, "Left");
//            } else if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
//                alignAprilTag(myAprilTagIdCode, "Right");
//            }
//        }


        frontLeftMotor.setPower((setY + setX + rx)*chassis_multi);
        backLeftMotor.setPower((setY - setX + rx)*chassis_multi*backMulti);
        frontRightMotor.setPower((setY - setX - rx)*chassis_multi);
        backRightMotor.setPower((setY + setX - rx)*chassis_multi*backMulti);

        telemetry.addData("Heading", Math.toDegrees(botHeading));
        telemetry.addData("FrontLeft", (setY + setX + rx));
        telemetry.addData("BackLeft", (setY - setX + rx));
        telemetry.addData("FrontRight", (setY - setX - rx));
        telemetry.addData("BackRight", (setY + setX - rx));
        telemetry.addData("rotX", rotX);
        telemetry.addData("rotY", rotY);

        try {

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad2.copy(gamepad2);
            currentGamepad1.copy(gamepad1);
        } catch (RuntimeException e) {
            e.printStackTrace();
        }
        telemetry.addData("Lift Left", slides.ExtensionLeft.getPower());
        telemetry.addData("Lift Left amp", slides.ExtensionLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Lift Left Ticks", slides.ExtensionLeft.getCurrentPosition());
        telemetry.addData("Lift Right Ticks", slides.ExtensionRight.getCurrentPosition());
        telemetry.addData("Lift Left Target", slides.targetPositionL);
        telemetry.addData("Lift Right Target", slides.targetPositionR);
        telemetry.addData("Left Servo Pos", intake.IntakeLeftFlip.getPosition());
        telemetry.addData("Right Servo Pos", intake.IntakeRightFlip.getPosition());
        telemetry.addData("Claw Servo Pos", intake.IntakeWrist.getPosition());
        telemetry.addData("HangLeft Draw", hanger.hangerMotorLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("HangRight Draw", hanger.hangerMotorRight.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("HangLeft Ticks", hanger.hangerMotorLeft.getCurrentPosition());
        telemetry.addData("HangRight Ticks", hanger.hangerMotorLeft.getCurrentPosition());
        telemetry.update();
    }

    public void findBotHeading(){
        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();

// Now use these simple methods to extract each angle
// (Java type double) from the object you just created:
        botHeading  = -robotOrientation.getYaw(AngleUnit.RADIANS);

        //deals with cases like when both offset and heading are negative its super convoluted but if you think about it for like 20 hours it makes sense it only took me 20 hours to make :)
        if (Math.toDegrees(botHeading)<-180){
            botHeading = Math.toRadians(180 - ((Math.abs(Math.toDegrees(botHeading))) - 180));
        }
        if (Math.toDegrees(botHeading) >= 180){
            botHeading = Math.toRadians((Math.toDegrees(botHeading) - 180)-180);
        }
    }

    public void RBReset(){
        if (gamepad1.right_bumper){
            imu.resetYaw();
        }
    }

    public void slowmode(){
        if (!previousGamepad1.left_bumper && currentGamepad1.left_bumper){
            if (chassis_multi < 1){
                chassis_multi = 1;
            }
            else {
                chassis_multi = 0.4;
            }
        }
    }

    public void dpadClawWrist(){
        double multi = 0;

//        if (gamepad2.dpad_left) multi = -1;
//        else if (gamepad2.dpad_right) multi = 1;

        clawPos += multi/500;


        if (clawPos >= 1){
            clawPos = 1;
        }
        else if (clawPos <= 0){
            clawPos = 0;
        }

        intake.IntakeWrist.setPosition(clawPos);
    }

    public void triggersTurnServo(){
        if (currentGamepad2.right_trigger > 0.3 && !(previousGamepad2.right_trigger > 0.3)) {
            armPosLeft += 0.012;
            armPosRight -= 0.01176;
        }
        if (currentGamepad2.left_trigger > 0.3 && !(previousGamepad2.left_trigger > 0.3)) {
            armPosLeft -= 0.012;
            armPosRight += 0.01176;
        }

        if (armPosLeft >= 1){
            armPosLeft = 1;
        }
        else if (armPosLeft <= 0){
            armPosLeft = 0;
        }

        if (armPosRight >= 1){
            armPosRight = 1;
        }
        else if (armPosRight <= 0){
            armPosRight = 0;
        }

        intake.IntakeLeftFlip.setPosition(armPosLeft);
        intake.IntakeRightFlip.setPosition(armPosRight);


    }


    public void alignAprilTag(int detectedTag, String targetTag){
//        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));
        double currentPos = 0;
        double targetPos = 0;
        if (detectedTag > 3) detectedTag = detectedTag - 3;
        if (detectedTag == 1) currentPos = 0;
        else if (detectedTag == 2) currentPos = 6;
        else currentPos = 12;
        if (targetTag.equals("Left")) targetPos = 0;
        else if(targetTag.equals("Middle")) targetPos = 6;
        else targetPos = 12;
        double horizontalDistance = currentPos - targetPos;
//        TrajectorySequence path1 = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
//                .turn(Math.toRadians(bearing))
//                .lineToSplineHeading(new Pose2d(range*Math.sin(Math.toRadians(bearing))-(distanceFromTag+camToCenter)*Math.sin(Math.toRadians(yaw))+(horizontalDistance)*Math.cos(Math.toRadians(yaw)),
//                        -(range*Math.cos(Math.toRadians(bearing))
//                                + camToCenter
//                                - (distanceFromTag+camToCenter) * Math.cos(Math.toRadians(yaw))) + (horizontalDistance)*Math.sin(Math.toRadians(yaw)), Math.toRadians(90+yaw)))
//                .build();
//        drive.followTrajectorySequence(path1);
//        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));
//        if (Math.abs(currentPos-targetPos) > 0) {
//            TrajectorySequence path2 = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
//                    .lineToSplineHeading(new Pose2d(, 0, Math.toRadians(90)))
//                    .build();
//            drive.followTrajectorySequence(path2);
//        }
    }

    public void checkDistance(boolean goDown){
        double threshold = 13;
        if (goDown) threshold = 7.3;
        double distance = distanceSensor.getDistance(DistanceUnit.INCH);
        if (distance < threshold){
//            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));
//            TrajectorySequence moveThing = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
//                    .lineTo(new Vector2d(0, Range.clip(15 - distance, 0, 15)))
//                    .build();
//            drive.followTrajectorySequence(moveThing);
        }
    }
}
