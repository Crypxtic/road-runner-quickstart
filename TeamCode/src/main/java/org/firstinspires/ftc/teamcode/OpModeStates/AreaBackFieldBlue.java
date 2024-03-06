//package org.firstinspires.ftc.teamcode.OpModeStates;
//
//import android.util.Size;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.EOCV.BluePropPipeline;
//import org.firstinspires.ftc.teamcode.HardwareClassState.IntakeArea;
//import org.firstinspires.ftc.teamcode.HardwareClassState.LinearSlidesArea;
//import org.firstinspires.ftc.teamcode.HardwareClasses.IntakeM1;
//import org.firstinspires.ftc.teamcode.HardwareClasses.LinearSlidesM1;
//import org.firstinspires.ftc.teamcode.HardwareClasses.OuttakeM1;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.openftc.easyopencv.OpenCvCamera;
//
//import java.util.List;
//
//@Autonomous
//public class AreaBackFieldBlue extends LinearOpMode {
//    double camToCenter = 6.25;
//    double rightAprilTagPos = -12;
//    double centerAprilTagPos = -6;
//    double leftAprilTagPos = 0;
//    double desiredYPos = 5;
//    double conePos = 1;
//    double botHeading;
//
//    WebcamName webcamFront, webcamBack;
//    VisionPortal visionPortal;
//
//    BluePropPipeline propCV;
//    AprilTagProcessor aprilTag;
//    int myAprilTagIdCode;
//    Pose2d robotPose;
//    OpenCvCamera camera;
//    Pose2d startPose = new Pose2d(12, 62, Math.toRadians(-90));
//
//    IntakeArea intake = new IntakeArea();
//    LinearSlidesArea slides = new LinearSlidesArea();
//
//    double range;
//    double bearing;
//    double heading;
//    double yaw;
//    double currentxPos;
//    double targetxMove;
//    double y;
//
//    List<AprilTagDetection> myAprilTagDetections;
//
//    TrajectorySequence goToBackdrop2;
//    TrajectorySequence goToStage;
//    TrajectorySequence goPark;
//    TrajectorySequence goToStack;
//    TrajectorySequence goToStage2;
//
//    SampleMecanumDrive drive;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        propCV = new BluePropPipeline(telemetry);
////
////        Vision init
//        webcamFront = hardwareMap.get(WebcamName.class, "Webcam 1");
//        webcamBack = hardwareMap.get(WebcamName.class, "backCam");
////
//        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
//
//        CameraName switchableCamera = ClassFactory.getInstance()
//                .getCameraManager().nameForSwitchableCamera(webcamFront, webcamBack);
//
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(switchableCamera)
//                .setCameraResolution(new Size(640, 480))
//                .addProcessors(aprilTag, propCV)
//                .build();
//
//        sleep(500);
//
//        visionPortal.setActiveCamera(webcamFront);
//        visionPortal.setProcessorEnabled(aprilTag, false);
//        visionPortal.setProcessorEnabled(propCV, true);
//
//        intake.init(hardwareMap);
//
//        intake.closeClawLeft();
//        intake.closeClawRight();
//
//        sleep(500);
//
//        intake.armOffGround();
//        intake.flipClawUp();
//        slides.init(hardwareMap);
//
//        while (!isStarted()){
//            telemetry.addData("Cone pos", propCV.finalPosition);
//            conePos = propCV.finalPosition;
//            telemetry.update();
//        }
//
//        waitForStart();
//
//        conePos = propCV.finalPosition;
//
//        visionPortal.setActiveCamera(webcamBack);
//        visionPortal.setProcessorEnabled(propCV, false);
//        visionPortal.setProcessorEnabled(aprilTag, true);
//
//        drive = new SampleMecanumDrive(hardwareMap);
//        TrajectorySequence dropPurpleSpike;
//
//        if (conePos == 2) {
//            dropPurpleSpike = drive.trajectorySequenceBuilder(startPose)
//                    .lineToSplineHeading(new Pose2d(9, 34, Math.toRadians(-90)))
//                    .build();
//        }
//        else if (conePos == 1) {
//            dropPurpleSpike = drive.trajectorySequenceBuilder(startPose)
//                    .lineToLinearHeading(new Pose2d(35, 30, Math.toRadians(180)))
//                    .build();
//        }
//        else {//left
//            dropPurpleSpike = drive.trajectorySequenceBuilder(startPose)
//                    .lineToSplineHeading(new Pose2d(12, 50, Math.toRadians(-100)))
//                    .lineToSplineHeading(new Pose2d(12, 35, Math.toRadians(-180)))
//                    .build();
//        }
//
//        TrajectorySequence goToBackdrop = drive.trajectorySequenceBuilder(dropPurpleSpike.end())
//                .addTemporalMarker(0.8, () -> {
//                    intake.armHalfway();
//                    intake.flipClawUp();
//                    slides.liftSlides();})
//                .lineToSplineHeading(new Pose2d(39, 35, Math.toRadians(180)))
//                .build();
//
//        TrajectorySequence goToBackdrop2;
//        if (conePos == 2) {
//            goToBackdrop2 = drive.trajectorySequenceBuilder(goToBackdrop.end())
//                    .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> 45)
//                    .setAccelConstraint((v, pose2d, pose2d1, pose2d2) -> 45)
//                    .lineToSplineHeading(new Pose2d(49, 35, Math.toRadians(180)))
//                    .build();
//        }
//        else if (conePos == 1) {
//            goToBackdrop2 = drive.trajectorySequenceBuilder(goToBackdrop.end())
//                    .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> 45)
//                    .setAccelConstraint((v, pose2d, pose2d1, pose2d2) -> 45)
//                    .lineToSplineHeading(new Pose2d(49, 42, Math.toRadians(180)))
//                    .build();
//        }
//        else {//left
//            goToBackdrop2 = drive.trajectorySequenceBuilder(goToBackdrop.end())
//                    .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> 45)
//                    .setAccelConstraint((v, pose2d, pose2d1, pose2d2) -> 45)
//                    .lineToSplineHeading(new Pose2d(49, 28, Math.toRadians(180)))
//                    .build();
//        }
//
//        TrajectorySequence goPark = drive.trajectorySequenceBuilder(goToBackdrop2.end())
//                .lineToSplineHeading(new Pose2d(41, 60, Math.toRadians(180)))
//                .lineToSplineHeading(new Pose2d(60, 60, Math.toRadians(180)))
//                .build();
//
//        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(goToBackdrop2.end())
//                .addTemporalMarker(0.6, () -> {slides.ExtensionLeft.setPower(0);
//                    slides.ExtensionRight.setPower(0);})
//                .lineTo(new Vector2d(35, 12))
//                .addDisplacementMarker(() ->{
//                    slides.resetLiftEncoder();
//                    intake.armToStack();
//                    intake.flipToStack();
//                    intake.openWideClawR();
//                    intake.openWideClawL();
//                })
//                .lineToSplineHeading(new Pose2d(-40, 12, Math.toRadians(180)))
//                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> 20)
//                .setAccelConstraint((v, pose2d, pose2d1, pose2d2) -> 20)
//                .lineToSplineHeading(new Pose2d(-63.5, 11.5, Math.toRadians(-170)))
//                .build();
//
//        TrajectorySequence goToStage;
//        if (conePos == 3 || conePos == 2) {
//            goToStage = drive.trajectorySequenceBuilder(goToStack.end())
//                    .lineToSplineHeading(new Pose2d(23, 10, Math.toRadians(180)))
//                    .addTemporalMarker(1, () -> {
//                        intake.flipClawUp();
//                    })
//                    .lineToSplineHeading(new Pose2d(50, 38, Math.toRadians(180)))
////                    .addDisplacementMarker(() -> {
////                        intake.armFaceUp();
////                        intake.wristFaceUp();})
//                    .addSpatialMarker(new Vector2d(10, 10), () -> {
//                        intake.armFaceUp();
//                        intake.wristFaceUp();
//                    })
//                    .build();
//        }
//        else {//goes to the left cuz its right or center
//            goToStage = drive.trajectorySequenceBuilder(goToStack.end())
//                    .lineToSplineHeading(new Pose2d(23, 10, Math.toRadians(180)))
//                    .addTemporalMarker(1, () -> {
//                        intake.flipClawUp();
//                    })
////                    .addDisplacementMarker(() -> {
////                        intake.armFaceUp();
////                        intake.wristFaceUp();})
//                    .lineToSplineHeading(new Pose2d(50, 34, Math.toRadians(180)))
//                    .addSpatialMarker(new Vector2d(10, 10), () -> {
//                        intake.armFaceUp();
//                        intake.wristFaceUp();
//                    })
//                    .build();
//        }
//
//        TrajectorySequence backOff = drive.trajectorySequenceBuilder(goToStage.end())
//                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> 20)
//                .setAccelConstraint((v, pose2d, pose2d1, pose2d2) -> 20)
//                .lineTo(new Vector2d(goToStage.end().getX() - 2, goToStage.end().getY()))
//                .lineTo(new Vector2d(goToStage.end().getX() - 4, goToStage.end().getY() + 2))
//                .build();
//
//
//        drive.setPoseEstimate(startPose);
//
//        if(!isStopRequested()){
//            intake.flipClawDown();
//            intake.armToStack();
//            drive.followTrajectorySequence(dropPurpleSpike);
//            intake.openWideClawL();
//
//            drive.followTrajectorySequence(goToBackdrop);
//
//            sleep(1000);
//
//            drive.setPoseEstimate(new Pose2d(37, 36.6, drive.getPoseEstimate().getHeading()));
//
//            drive.setPoseEstimate(getNewPose());
//
//            visionPortal.close();
//
//            intake.flipArmUp();
//
//            drive.followTrajectorySequence(goToBackdrop2);
//
//            intake.openWideClawR();
//            sleep(500);
//            slides.resetLiftEncoder();
//            slides.moveSlides(-1);
//            intake.armOffGround();
//
//            drive.followTrajectorySequence(goToStack);
//            intake.clawAuto();
//
//            sleep(100);
//            intake.armToStack1();
//
//            drive.followTrajectorySequence(goToStage);
//
//            intake.openWideClawR();
//            intake.openWideClawL();
//
//            drive.followTrajectorySequence(backOff);
//
//            sleep(1000);
//
//
//            intake.armOffGround();
//            intake.flipClawUp();
//            sleep(1000);//put in a marker
//        }
//
//    }
//
//    public Pose2d getNewPose(){
//        for (int i = 0; i < 10; i++) {
//            myAprilTagDetections = aprilTag.getDetections();
//            boolean gotTag = false;
//            for (AprilTagDetection detection : myAprilTagDetections) {
//
//                if (detection.metadata != null && !gotTag) {  // This check for non-null Metadata is not needed for reading only ID code.
//                    gotTag = true;
//                    myAprilTagIdCode = detection.id;
//                    range = detection.ftcPose.range*Math.cos(Math.toRadians(heading+11));
//                    bearing = detection.ftcPose.bearing;
//                    yaw = detection.ftcPose.yaw;
//                    y = detection.ftcPose.y;
//
//                    telemetry.addData("id", myAprilTagIdCode);
//                    telemetry.addData("name", detection.metadata.name);
//                    telemetry.addData("range", detection.ftcPose.range);
//                    telemetry.addData("yaw", detection.ftcPose.yaw);
//                    telemetry.addData("x", detection.ftcPose.x);
//                    telemetry.addData("y", detection.ftcPose.y);
//
//                    Vector2d myVector = new Vector2d(detection.ftcPose.x + 1, detection.ftcPose.y + 9);
////                    myVector.rotated(drive.getPoseEstimate().getHeading());
//                    myVector = myVector.rotated(Math.toRadians(-yaw));
//
//                    drive.updatePoseEstimate();
//
//
//                    double myX = detection.metadata.fieldPosition.get(0) - myVector.getY();
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
//}
