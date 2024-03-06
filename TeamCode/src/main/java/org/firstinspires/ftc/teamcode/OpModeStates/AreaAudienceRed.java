//package org.firstinspires.ftc.teamcode.OpModeStates;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.IMU;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.teamcode.EOCV.TeamPropPipeline;
//import org.firstinspires.ftc.teamcode.HardwareClassState.IntakeArea;
//import org.firstinspires.ftc.teamcode.HardwareClassState.LinearSlidesArea;
//import org.firstinspires.ftc.teamcode.HardwareClasses.IntakeM1;
//import org.firstinspires.ftc.teamcode.HardwareClasses.LinearSlidesM1;
//import org.firstinspires.ftc.teamcode.HardwareClasses.OuttakeM1;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import java.util.List;
//
//
//@Autonomous
//public class AreaAudienceRed extends LinearOpMode {
//    Pose2d startPose = new Pose2d(-35, -62, Math.toRadians(90));
//
//
//    WebcamName webcamFront, webcamBack;
//    VisionPortal visionPortal;
//
//    TeamPropPipeline propCV;
//    AprilTagProcessor aprilTag;
//    double conePos = 1;
//
//    IntakeArea intake = new IntakeArea();
//    LinearSlidesArea slides = new LinearSlidesArea();
//
//    IMU imu;
//    List<AprilTagDetection> myAprilTagDetections;
//
//    int myAprilTagIdCode;
//    Pose2d robotPose;
//    double botHeading;
//
//    SampleMecanumDrive drive;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        drive = new SampleMecanumDrive(hardwareMap);
//        propCV = new TeamPropPipeline(telemetry);
//
//        //Vision init
//        webcamFront = hardwareMap.get(WebcamName.class, "Webcam 1");
//        webcamBack = hardwareMap.get(WebcamName.class, "backCam");
//
//        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
//
//        CameraName switchableCamera = ClassFactory.getInstance()
//                .getCameraManager().nameForSwitchableCamera(webcamFront, webcamBack);
//
//
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(switchableCamera)
//                .addProcessor(aprilTag)
//                .addProcessor(propCV)
//                .build();
//
//        sleep(500);
//
//        visionPortal.setProcessorEnabled(aprilTag, false);
//        visionPortal.setProcessorEnabled(propCV, true);
//        visionPortal.setActiveCamera(webcamFront);
//
//        imu = hardwareMap.get(IMU.class, "imu");
//
//        imu.initialize(
//                new IMU.Parameters(
//                        new RevHubOrientationOnRobot(
//                                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
//                                RevHubOrientationOnRobot.UsbFacingDirection.UP
//                        )
//                )
//        );
//
//        intake.init(hardwareMap);
//        intake.closeClawRight();
//        intake.closeClawLeft();
//        sleep(500);
//        intake.flipClawUp();
//        intake.armOffGround();
//        slides.init(hardwareMap);
//
//        while (!isStarted()){
//            telemetry.addData("Cone pos", propCV.finalPosition);
//            telemetry.update();
//        }
//
//        waitForStart();
//
//        conePos = propCV.finalPosition;
//
//        TrajectorySequence goToStackTag = drive.trajectorySequenceBuilder(startPose)
//                .lineToSplineHeading(new Pose2d(-45, -35, Math.toRadians(0)))
//                .build();
//
//        TrajectorySequence dropPurpleSpike;
//
//        visionPortal.setActiveCamera(webcamBack);
//        visionPortal.setProcessorEnabled(propCV, false);
//        visionPortal.setProcessorEnabled(aprilTag, true);
//
//        if (conePos == 2) {
//            dropPurpleSpike = drive.trajectorySequenceBuilder(goToStackTag.end())
//                    .lineToSplineHeading(new Pose2d(-35, -10, Math.toRadians(270)))
//                    .addDisplacementMarker(() -> {
//                        intake.openWideClawL();
//                        sleep(500);
//                    })
//                    .lineToSplineHeading(new Pose2d(-30, -7, Math.toRadians(250)))
//                    .build();
//        }
//        else if (conePos == 3){
//            dropPurpleSpike = drive.trajectorySequenceBuilder(goToStackTag.end())
//                    .lineToSplineHeading(new Pose2d(-33.5, -34, Math.toRadians(0)))
//                    .addDisplacementMarker(() -> {
//                        intake.openWideClawL();
//                        sleep(500);
//                    })
//                    .lineToSplineHeading(new Pose2d(-37, -36, Math.toRadians(0)))
//                    .build();
//        }
//        else {
//            dropPurpleSpike = drive.trajectorySequenceBuilder(goToStackTag.end())
//                    .lineToLinearHeading(new Pose2d(-48, -20, Math.toRadians(270)))
//                    .build();
//        }
//
//        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(dropPurpleSpike.end())
//                .lineToSplineHeading(new Pose2d(-40, -10.05, Math.toRadians(180)))
//                .build();
//
//        TrajectorySequence goToStack2 = drive.trajectorySequenceBuilder(goToStack.end())
//                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> 20)
//                .setAccelConstraint((v, pose2d, pose2d1, pose2d2) -> 20)
//                .lineToSplineHeading(new Pose2d(-58, -11, Math.toRadians(170)))
//                .addTemporalMarker(0, ()->{
//                    intake.flipToStack();
//                })
//                .build();
//
//        TrajectorySequence goToBackdrop = drive.trajectorySequenceBuilder(goToStack2.end())
//                .lineToSplineHeading(new Pose2d(25, -8, Math.toRadians(180)))
//                .lineToSplineHeading(new Pose2d(42, -16, Math.toRadians(150)))
//                .addTemporalMarker(1, () -> {
//                    intake.flipClawUp();
//                })
//                .build();
//
//        TrajectorySequence goToBackdrop2;
//        if (conePos == 2) {
//            goToBackdrop2 = drive.trajectorySequenceBuilder(goToBackdrop.end())
//                    .lineToSplineHeading(new Pose2d(48, -39, Math.toRadians(180)))
//                    .build();
//        }
//        else if (conePos == 3) {
//            goToBackdrop2 = drive.trajectorySequenceBuilder(goToBackdrop.end())
//                    .lineToSplineHeading(new Pose2d(48, -44, Math.toRadians(175)))
//                    .build();
//        }
//        else {//left
//            goToBackdrop2 = drive.trajectorySequenceBuilder(goToBackdrop.end())
//                    .lineToSplineHeading(new Pose2d(48, -33, Math.toRadians(180)))
//                    .build();
//        }
//
//        TrajectorySequence goToBackdrop3;
//        if (conePos == 2) {
//            goToBackdrop3 = drive.trajectorySequenceBuilder(goToBackdrop2.end())
//                    .lineToSplineHeading(new Pose2d(48, -40, Math.toRadians(180)))
//                    .build();
//        }
//        else {
//            goToBackdrop3 = drive.trajectorySequenceBuilder(goToBackdrop2.end())
//                    .lineToSplineHeading(new Pose2d(48, -39, Math.toRadians(180)))
//                    .build();
//        }
//
//        TrajectorySequence goPark = drive.trajectorySequenceBuilder(goToBackdrop3.end())
//                .lineToSplineHeading(new Pose2d(41, -10, Math.toRadians(180)))
//                .addTemporalMarker(1, () -> {
//                    intake.closeClawRight();
//                    intake.closeClawLeft();
//                    slides.resetSlides();
//                    slides.moveSlides(-0.8);
//                })
//                .lineToSplineHeading(new Pose2d(60, -10, Math.toRadians(180)))
//                .build();
//
//        drive.setPoseEstimate(startPose);
//        if(!isStopRequested()){
//            intake.flipToStack();
//            intake.armToStack();
//
//
//            drive.followTrajectorySequence(goToStackTag);
//
//            sleep(1000);
//
//            drive.followTrajectorySequence(dropPurpleSpike);
//            intake.openWideClawL();
//
//            sleep(1000);
//
//            drive.followTrajectorySequence(goToStack);
//            intake.armToStack1();
//            intake.flipToStack();
//
//            drive.followTrajectorySequence(goToStack2);
//            intake.clawAuto();
//
//            drive.followTrajectorySequence(goToBackdrop);
//
//            sleep(1500);
//
//            drive.setPoseEstimate(new Pose2d(35.2, -17.7, drive.getPoseEstimate().getHeading()));
//
//            drive.setPoseEstimate(getNewPoseBack(false));
//
//            sleep(2500);
//
//            intake.flipArmUp();
//            intake.flipClawUp();
//            slides.autoLiftSlides();
//
//            sleep(2500);
//
//            drive.followTrajectorySequence(goToBackdrop2);
//
//            slides.resetLiftEncoder();
//            slides.liftSlides();
//
//            intake.openWideClawR();
//
//            sleep(500);
//
//            drive.followTrajectorySequence(goToBackdrop3);
//
//            intake.openWideClawL();
//
//            sleep(1000);
//
//            intake.armOffGround();
//            intake.flipClawUp();
//
//            intake.closeClawRight();
//            intake.closeClawLeft();
//            slides.resetLiftEncoder();
//            slides.moveSlides(-0.8);
//
//            sleep(1000);
//
//            slides.ExtensionLeft.setPower(0);
//            slides.ExtensionRight.setPower(0);
//
//        }
//
//    }
//
//    public void getHeading(){
//        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
//
//// Now use these simple methods to extract each angle
//// (Java type double) from the object you just created:
//        botHeading  = robotOrientation.getYaw(AngleUnit.RADIANS);
//
//        //deals with cases like when both offset and heading are negative its super convoluted but if you think about it for like 20 hours it makes sense it only took me 20 hours to make :)
//        if (Math.toDegrees(botHeading) < -180){
//            botHeading = Math.toRadians(360 + Math.toDegrees(botHeading));
//        }
//        if (Math.toDegrees(botHeading) > 180){
//            botHeading = Math.toRadians(Math.toRadians(botHeading) - 360);
//        }
//    }
//
//    public Pose2d getNewPoseBack(boolean stack){
//        for (int i = 0; i < 10; i++) {
//            myAprilTagDetections = aprilTag.getDetections();
//            boolean gotTag = false;
//            for (AprilTagDetection detection : myAprilTagDetections) {
//
//                if (detection.metadata != null && !gotTag) {  // This check for non-null Metadata is not needed for reading only ID code.
//                    gotTag = true;
//                    myAprilTagIdCode = detection.id;
//                    double bearing = detection.ftcPose.bearing;
//                    double yaw = detection.ftcPose.yaw;
//                    double y = detection.ftcPose.y;
//
//                    Vector2d myVector = new Vector2d(detection.ftcPose.x + 1, detection.ftcPose.y + 9);
////                    myVector.rotated(drive.getPoseEstimate().getHeading());
//                    myVector = myVector.rotated(Math.toRadians(-yaw));
//
//                    drive.updatePoseEstimate();
//
//
//                    double myX = detection.metadata.fieldPosition.get(0) - myVector.getY();
//                    if (stack) myX = detection.metadata.fieldPosition.get(0) + myVector.getY();
//                    double myY = detection.metadata.fieldPosition.get(1) + myVector.getX();
//                    robotPose = new Pose2d(myX,myY,drive.getPoseEstimate().getHeading());
//
//
//                    telemetry.addData("Heading", -yaw);
//                    telemetry.addData("RR Heading", Math.toDegrees(drive.getPoseEstimate().getHeading()));
//                    telemetry.addData("IMU Heading", Math.toDegrees(botHeading));
//                    telemetry.addData("myX", myX);
//                    telemetry.addData("myY", myY);
//                    telemetry.update();
//
//                    if (Math.abs(180-yaw - Math.toDegrees(drive.getPoseEstimate().getHeading())) > 45) continue;
//                    if (Math.abs(myX - drive.getPoseEstimate().getX()) > 10) continue;
//                    if (Math.abs(myY - drive.getPoseEstimate().getY()) > 10) continue;
//                    return robotPose;
//                }
//            }
//        }
//        return drive.getPoseEstimate();
//    }
//
//    public Pose2d getNewPoseFront(){
//        for (int i = 0; i < 10; i++) {
//            myAprilTagDetections = aprilTag.getDetections();
//            boolean gotTag = false;
//            for (AprilTagDetection detection : myAprilTagDetections) {
//
//                if (detection.metadata != null && !gotTag) {  // This check for non-null Metadata is not needed for reading only ID code.
//                    gotTag = true;
//                    myAprilTagIdCode = detection.id;
//                    double yaw = detection.ftcPose.yaw;
//
//                    Vector2d myVector = new Vector2d(detection.ftcPose.x - 4, detection.ftcPose.y + 3);
////                    myVector.rotated(drive.getPoseEstimate().getHeading());
//                    myVector = myVector.rotated(Math.toRadians(-yaw));
//
//                    drive.updatePoseEstimate();
//
//
//                    double myX = detection.metadata.fieldPosition.get(0) + myVector.getY();
//                    double myY = detection.metadata.fieldPosition.get(1) - myVector.getX();
//                    robotPose = new Pose2d(myX,myY,Math.toRadians(180-yaw));
//
//
//                    telemetry.addData("Heading", -yaw);
//                    telemetry.addData("RR Heading", Math.toDegrees(drive.getPoseEstimate().getHeading()));
//                    telemetry.addData("IMU Heading", Math.toDegrees(botHeading));
//                    telemetry.addData("myX", myX);
//                    telemetry.addData("myY", myY);
//                    telemetry.update();
//
//                    if (Math.abs(180-yaw - Math.toDegrees(drive.getPoseEstimate().getHeading())) > 45) continue;
//                    if (Math.abs(myX - drive.getPoseEstimate().getX()) > 5) continue;
//                    if (Math.abs(myY - drive.getPoseEstimate().getY()) > 5) continue;
//                    return robotPose;
//                }
//            }
//        }
//        return drive.getPoseEstimate();
//    }
//}
