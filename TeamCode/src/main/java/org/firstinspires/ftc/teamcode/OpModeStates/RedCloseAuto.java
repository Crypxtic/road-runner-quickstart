package org.firstinspires.ftc.teamcode.OpModeStates;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.EOCV.BluePropPipeline;
import org.firstinspires.ftc.teamcode.EOCV.TeamPropPipeline;
import org.firstinspires.ftc.teamcode.HardwareClassState.ActionAuto;
import org.firstinspires.ftc.teamcode.HardwareClassState.IntakeStates;
import org.firstinspires.ftc.teamcode.HardwareClassState.LinearSlidesStates;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.List;

@Config
@Autonomous
public class RedCloseAuto extends LinearOpMode {
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    WebcamName webcamFront, webcamBack;
    VisionPortal visionPortal;
    double conePos = 1;
    int myAprilTagIdCode;
    Pose2d robotPose;
    OpenCvCamera camera;
    IntakeStates intake = new IntakeStates();
    LinearSlidesStates slides = new LinearSlidesStates();
    List<AprilTagDetection> myAprilTagDetections;
    double range;
    double bearing;
    double heading;
    double yaw;
    double y;
    long delay = 0;
    boolean twoOnly = false;
    TeamPropPipeline propCV;
    AprilTagProcessor aprilTag;
    MecanumDrive drive;

    ActionAuto autoActions = new ActionAuto();
    ElapsedTime armTimer = new ElapsedTime();
    ElapsedTime slideTimer = new ElapsedTime();
    boolean stageCycle1 = false;
    boolean stageCycle2 = false;
    boolean throughTruss = false;
    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(12, -62, Math.toRadians(90)));

        // vision here that outputs position
        int conePos = 1;

        Action trajectoryAction1;
        Action trajectoryAction2;
        Action trajectoryAction3;
        Action goToBackdrop;
        Action goToStage;
        Action goToStack2Cycle;

        propCV = new TeamPropPipeline(telemetry);
        webcamFront = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcamBack = hardwareMap.get(WebcamName.class, "backCam");
//
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcamFront, webcamBack);

        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .setCameraResolution(new Size(640, 480))
                .addProcessors(aprilTag, propCV)
                .build();

        sleep(500);

        visionPortal.setActiveCamera(webcamFront);
        visionPortal.setProcessorEnabled(aprilTag, false);
        visionPortal.setProcessorEnabled(propCV, true);

        intake.init(hardwareMap);

        intake.closeClawLeft();
        intake.closeClawRight();

        sleep(500);

        intake.armToCycle();
        intake.flipClawUp();
        slides.init(hardwareMap);
        slides.resetLiftEncoder();

        autoActions.init(intake, slides, armTimer, slideTimer);


        trajectoryAction3 = drive.actionBuilder(new Pose2d(12, -62, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(35, -33), Math.toRadians(180))
                .build();
        trajectoryAction2 = drive.actionBuilder(new Pose2d(12, -62, Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(13, -35.5), Math.toRadians(90))
                .build();
        trajectoryAction1 = drive.actionBuilder(new Pose2d(12, -62, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(11.5, -33, Math.toRadians(160)), Math.toRadians(160))
                .build();

        Action trajectoryAction1Back;
        Action trajectoryAction2Back;
        Action trajectoryAction3Back;
        Action goToStage2Cycle;

        trajectoryAction3Back = drive.actionBuilder(new Pose2d(39, -35, Math.toRadians(180)))
                .strafeToSplineHeading(new Vector2d(48, -46), Math.toRadians(180), new TranslationalVelConstraint(45), new ProfileAccelConstraint(-45, 45))
                .afterDisp(0, new ParallelAction(
                        autoActions.openWideClawR()
                ))
                .strafeToSplineHeading(new Vector2d(45, -30), Math.toRadians(180))
                .afterDisp(0, new ParallelAction(
                        autoActions.lowerArmDown()
                ))
                .strafeToSplineHeading(new Vector2d(45, -25), Math.toRadians(180))
                .afterDisp(0.8, new ParallelAction(
                        autoActions.openWideClawL(),
                        autoActions.openWideClawR()
                ))
                .splineToConstantHeading(new Vector2d(25, -10), Math.toRadians(180))
                .endTrajectory()
                .strafeToSplineHeading(new Vector2d(-50, -10), Math.toRadians(180))
                .afterDisp(0, new ParallelAction(
                        autoActions.armToStack(),
                        new InstantAction(() -> intake.flipToStack())
                ))
                .strafeToSplineHeading(new Vector2d(-62.5, -9), Math.toRadians(180), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-20, 20))
                .build();

        trajectoryAction2Back = drive.actionBuilder(new Pose2d(39, -35, Math.toRadians(180)))
                .strafeToSplineHeading(new Vector2d(48, -38.5), Math.toRadians(180), new TranslationalVelConstraint(45), new ProfileAccelConstraint(-45, 45))
                .afterDisp(0, new ParallelAction(
                        autoActions.openWideClawR()
                ))
                .strafeToSplineHeading(new Vector2d(45, -30), Math.toRadians(180))
                .afterDisp(0, new ParallelAction(
                        autoActions.lowerArmDown()
                ))
                .strafeToSplineHeading(new Vector2d(45, -25), Math.toRadians(180))
                .afterDisp(0.8, new ParallelAction(
                        autoActions.openWideClawL(),
                        autoActions.openWideClawR()
                ))
                .splineToConstantHeading(new Vector2d(25, -10), Math.toRadians(180))
                .endTrajectory()
                .strafeToSplineHeading(new Vector2d(-53, -10), Math.toRadians(180))
                .afterDisp(0, new ParallelAction(
                        autoActions.armToStack(),
                        new InstantAction(() -> intake.flipToStack())
                ))
                .strafeToSplineHeading(new Vector2d(-62.5, -9), Math.toRadians(180), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-20, 20))
                .build();

        trajectoryAction1Back = drive.actionBuilder(new Pose2d(39, -35, Math.toRadians(180)))
                .strafeToSplineHeading(new Vector2d(48, -32), Math.toRadians(180), new TranslationalVelConstraint(45), new ProfileAccelConstraint(-45, 45))
                .afterDisp(0, new ParallelAction(
                        autoActions.openWideClawR()
                ))
                .strafeToSplineHeading(new Vector2d(45, -30), Math.toRadians(180))
                .afterDisp(0, new ParallelAction(
                        autoActions.lowerArmDown()
                ))
                .strafeToSplineHeading(new Vector2d(45, -25), Math.toRadians(180))
                .afterDisp(0.8, new ParallelAction(
                        autoActions.openWideClawL(),
                        autoActions.openWideClawR()
                ))
                .splineToConstantHeading(new Vector2d(25, -10), Math.toRadians(180))
                .endTrajectory()
                .strafeToSplineHeading(new Vector2d(-53, -10), Math.toRadians(180))
                .afterDisp(0, new ParallelAction(
                        autoActions.armToStack(),
                        new InstantAction(() -> intake.flipToStack())
                ))
                .strafeToSplineHeading(new Vector2d(-62.5, -9), Math.toRadians(180), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-20, 20))
                .build();

        Action trajectoryAction1Truss = drive.actionBuilder(new Pose2d(39, -35, Math.toRadians(180)))
                .strafeToSplineHeading(new Vector2d(48, -32), Math.toRadians(180), new TranslationalVelConstraint(45), new ProfileAccelConstraint(-45, 45))
                .afterDisp(0, new ParallelAction(
                        autoActions.openWideClawR(),
                        autoActions.openWideClawL()
                ))
                .strafeToSplineHeading(new Vector2d(45, -33.5), Math.toRadians(180))
                .afterDisp(0, new ParallelAction(
                        autoActions.lowerArmDown()
                ))
                .strafeToSplineHeading(new Vector2d(45, -34.5), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(20, -59), Math.toRadians(180))
                .endTrajectory()
                .strafeToSplineHeading(new Vector2d(-30, -59), Math.toRadians(180))
                .afterDisp(0.8, new ParallelAction(
                                new InstantAction(() -> intake.flipToStack()),
                                autoActions.armToStack(),
                                autoActions.openWideClawL(),
                                autoActions.openWideClawR()
                        )
                )
                .splineToConstantHeading(new Vector2d(-50, -38), Math.toRadians(-45))
                .strafeToSplineHeading(new Vector2d(-61, -38), Math.toRadians(180), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-20, 20))
                .build();

        Action trajectoryAction2Truss = drive.actionBuilder(new Pose2d(39, -35, Math.toRadians(180)))
                .strafeToSplineHeading(new Vector2d(48, -38.5), Math.toRadians(180), new TranslationalVelConstraint(45), new ProfileAccelConstraint(-45, 45))
                .afterDisp(0, new ParallelAction(
                        autoActions.openWideClawR(),
                        autoActions.openWideClawL()
                ))
                .strafeToSplineHeading(new Vector2d(45, -33.5), Math.toRadians(180))
                .afterDisp(0, new ParallelAction(
                        autoActions.lowerArmDown()
                ))
                .strafeToSplineHeading(new Vector2d(45, -34.5), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(20, -59), Math.toRadians(180))
                .endTrajectory()
                .strafeToSplineHeading(new Vector2d(-30, -59), Math.toRadians(180))
                .afterDisp(0.8, new ParallelAction(
                                new InstantAction(() -> intake.flipToStack()),
                                autoActions.armToStack(),
                                autoActions.openWideClawL(),
                                autoActions.openWideClawR()
                        )
                )
                .splineToConstantHeading(new Vector2d(-50, -38), Math.toRadians(-45))
                .strafeToSplineHeading(new Vector2d(-61, -38), Math.toRadians(180), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-20, 20))
                .build();

        Action trajectoryAction3Truss = drive.actionBuilder(new Pose2d(39, -35, Math.toRadians(180)))
                .strafeToSplineHeading(new Vector2d(48, -46), Math.toRadians(180), new TranslationalVelConstraint(45), new ProfileAccelConstraint(-45, 45))
                .afterDisp(0, new ParallelAction(
                        autoActions.openWideClawR(),
                        autoActions.openWideClawL()
                ))
                .strafeToSplineHeading(new Vector2d(45, -33.5), Math.toRadians(180))
                .afterDisp(0, new ParallelAction(
                        autoActions.lowerArmDown()
                ))
                .strafeToSplineHeading(new Vector2d(45, -34.5), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(20, -59), Math.toRadians(180))
                .endTrajectory()
                .strafeToSplineHeading(new Vector2d(-30, -59), Math.toRadians(180))
                .afterDisp(0.8, new ParallelAction(
                                new InstantAction(() -> intake.flipToStack()),
                                autoActions.armToStack(),
                                autoActions.openWideClawL(),
                                autoActions.openWideClawR()
                        )
                )
                .splineToConstantHeading(new Vector2d(-50, -38), Math.toRadians(-45))
                .strafeToSplineHeading(new Vector2d(-61, -38), Math.toRadians(180), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-20, 20))
                .build();

        Action dropStage = drive.actionBuilder(new Pose2d(-61, -10, Math.toRadians(180)))
                .afterTime(1.5, new ParallelAction(
                        autoActions.raiseArmFaceUp(),
                        new InstantAction(() -> slides.resetLiftEncoder())
//                        autoActions.liftSlides()
                ))
                .afterTime(2, new ParallelAction(
                        new InstantAction(() -> intake.wristFaceUp())
                ))
                .strafeToSplineHeading(new Vector2d(40, -11), Math.toRadians(180))
                .afterDisp(0, new ParallelAction(
                        autoActions.openWideClawR(),
                        autoActions.openWideClawL(),
                        new SleepAction(0.1),
                        autoActions.flipClawUp(),
                        autoActions.lowerArmDown()
                ))
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(-50, -10), Math.toRadians(180))
                .afterDisp(0, new ParallelAction(
                        autoActions.armToStackCycle(),
                        new InstantAction(() -> intake.flipToStack())
                ))
                .strafeToSplineHeading(new Vector2d(-62.5, -9), Math.toRadians(180), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-20, 20))
                .build();

        Action dropStageCycle2 = drive.actionBuilder(new Pose2d(-61, -10, Math.toRadians(180)))
                .afterTime(1.5, new ParallelAction(
                        autoActions.raiseArmFaceUp(),
                        new InstantAction(() -> slides.resetLiftEncoder())
//                        autoActions.liftSlides()
                ))
                .afterTime(2, new ParallelAction(
                        new InstantAction(() -> intake.wristFaceUp())
                ))
                .strafeToSplineHeading(new Vector2d(40, -10), Math.toRadians(180))
                .afterDisp(0, new ParallelAction(
                        autoActions.openWideClawR(),
                        autoActions.openWideClawL(),
                        autoActions.lowerArmDown()
                ))
                .build();


        Action goToStageTruss = drive.actionBuilder(new Pose2d(-61, -38, Math.toRadians(180)))
                .afterTime(1, () -> {
                    intake.flipClawUp();
                })
                .strafeToSplineHeading(new Vector2d(-57, -46.5), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-30, -59), Math.toRadians(0))
                .endTrajectory()
                .strafeToSplineHeading(new Vector2d(22.5, -59), Math.toRadians(180))
                .afterTime(0, new ParallelAction(
                        autoActions.raiseArmFaceUp()
                ))
                .afterTime(0.5, new ParallelAction(
                        new InstantAction(() -> slides.resetLiftEncoder()),
                        autoActions.autoLiftSlides(),
                        new InstantAction(() -> intake.wristFaceUp())
                ))
                .splineToConstantHeading(new Vector2d(46.5, -44), Math.toRadians(-45))
                .build();

        Action dropStageTruss = drive.actionBuilder(new Pose2d(-61, -38, Math.toRadians(180)))
                .afterTime(1, () -> {
                    intake.flipClawUp();
                })
                .strafeToSplineHeading(new Vector2d(-57, -46.5), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-30, -59), Math.toRadians(0))
                .endTrajectory()
                .afterTime(1, new ParallelAction(
                        autoActions.raiseArmFaceUp()
                ))
                .afterTime(1.2, new ParallelAction(
                        new InstantAction(() -> intake.wristFaceUp())
                ))
                .strafeToSplineHeading(new Vector2d(40, -59), Math.toRadians(180))
                .afterDisp(0, new ParallelAction(
                        autoActions.openWideClawR(),
                        autoActions.openWideClawL(),
                        autoActions.lowerArmDown()
                ))
                .waitSeconds(1)
                .strafeToSplineHeading(new Vector2d(-30, -59), Math.toRadians(180))
                .afterDisp(0.8, new ParallelAction(
                                new InstantAction(() -> intake.flipToStack()),
                                autoActions.armToStackCycle(),
                                autoActions.openWideClawL(),
                                autoActions.openWideClawR()
                        )
                )
                .splineToConstantHeading(new Vector2d(-50, -38), Math.toRadians(-45))
                .strafeToSplineHeading(new Vector2d(-61, -38), Math.toRadians(180), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-20, 20))
                .build();

        Action goToStackTrussCycle2 = drive.actionBuilder(new Pose2d(47, -44, Math.toRadians(180)))
                .strafeToSplineHeading(new Vector2d(45, -34.5), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(20, -59), Math.toRadians(180))
                .afterDisp(0.8, new ParallelAction(
                                new InstantAction(() -> intake.flipClawUp())
                        )
                )
                .endTrajectory()
                .strafeToSplineHeading(new Vector2d(-30, -59), Math.toRadians(180))
                .afterDisp(0.8, new ParallelAction(
                                new InstantAction(() -> intake.flipToStack()),
                                autoActions.armToStackCycle(),
                                autoActions.openWideClawL(),
                                autoActions.openWideClawR()
                        )
                )
                .splineToConstantHeading(new Vector2d(-50, -38), Math.toRadians(-45))
                .strafeToSplineHeading(new Vector2d(-61, -38), Math.toRadians(180), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-20, 20))
                .build();

        Action goToStageTrussCycle2 = drive.actionBuilder(new Pose2d(-61, -38, Math.toRadians(180)))
                .afterTime(1, () -> {
                    intake.flipClawUp();
                })
                .strafeToSplineHeading(new Vector2d(-57, -46.5), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-30, -59), Math.toRadians(0))
                .endTrajectory()
                .afterTime(1, new ParallelAction(
                        autoActions.raiseArmFaceUp()
                ))
                .afterTime(1.2, new ParallelAction(
                        new InstantAction(() -> slides.resetLiftEncoder()),
                        new InstantAction(() -> intake.wristFaceUp())
                ))
                .strafeToSplineHeading(new Vector2d(40, -59), Math.toRadians(180))
                .build();

        goToStage = drive.actionBuilder(new Pose2d(-62.5, -10, Math.toRadians(180)))
                .afterTime(2.4, new ParallelAction(
                        autoActions.raiseArmFaceUp(),
                        new InstantAction(() -> slides.resetLiftEncoder())
//                        autoActions.liftSlides()
                ))
                .afterTime(2.7, new ParallelAction(
                        new InstantAction(() -> intake.wristFaceUp()),
                        new SequentialAction(
                                new InstantAction(() -> slides.resetLiftEncoder()),
                                autoActions.autoLiftSlides()
                        )
                ))
                .strafeToSplineHeading(new Vector2d(20, -10), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(46.5, -29), Math.toRadians(-45))
                .build();

        goToStack2Cycle = drive.actionBuilder(new Pose2d(47, -28, Math.toRadians(180)))
                .strafeToSplineHeading(new Vector2d(47, -23), Math.toRadians(180))
                .afterDisp(0.8, new ParallelAction(
                        autoActions.openWideClawL(),
                        autoActions.openWideClawR()
                ))
                .splineToConstantHeading(new Vector2d(25, -11), Math.toRadians(180))
                .endTrajectory()
                .afterDisp(0, new ParallelAction(
                        autoActions.flipClawUp()
                ))
                .strafeToSplineHeading(new Vector2d(-50, -11), Math.toRadians(180))
                .afterDisp(0, new ParallelAction(
                        autoActions.armToStackCycle(),
                        new InstantAction(() -> intake.flipToStack())
                ))
                .strafeToSplineHeading(new Vector2d(-62.5, -10), Math.toRadians(180), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-20, 20))
                .build();

        goToStage2Cycle = drive.actionBuilder(new Pose2d(-62.5, -12, Math.toRadians(180)))
                .afterTime(2.4, new ParallelAction(
                        autoActions.raiseArmFaceUp(),
                        new InstantAction(() -> slides.resetLiftEncoder())
//                        autoActions.liftSlides()
                ))
                .afterTime(2.7, new ParallelAction(
                        new InstantAction(() -> intake.wristFaceUp()),
                        new SequentialAction(
                                new InstantAction(() -> slides.resetLiftEncoder()),
                                autoActions.farLiftSlides()
                        )
                ))
                .strafeToSplineHeading(new Vector2d(20, -12), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(46.25, -33), Math.toRadians(-60))
                .build();

        Action goPark = drive.actionBuilder(drive.pose)
                .strafeToSplineHeading(new Vector2d(41, 10), Math.toRadians(180))
                .strafeToSplineHeading(new Vector2d(60, 10), Math.toRadians(180))
                .build();



        // actions that need to happen on init; for instance, a claw tightening.
//        Actions.runBlocking(claw.closeClaw());

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Cone pos", propCV.finalPosition);
            telemetry.addData("Go through Truss?", throughTruss);
            telemetry.addData("To backstage 1st cycle?", stageCycle1);
            telemetry.addData("To backstage 2nd cycle?", stageCycle2);
            conePos = propCV.finalPosition;
            telemetry.update();

            if (currentGamepad1.x && !previousGamepad1.x){throughTruss = !throughTruss;}
            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left){stageCycle1 = !stageCycle1;}
            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right){stageCycle2 = !stageCycle2;}

            try {

                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad2);

                currentGamepad2.copy(gamepad2);
                currentGamepad1.copy(gamepad1);
            } catch (RuntimeException e) {
                e.printStackTrace();
            }
        }

        waitForStart();

        conePos = propCV.finalPosition;

        visionPortal.setActiveCamera(webcamBack);
        visionPortal.setProcessorEnabled(propCV, false);
        visionPortal.setProcessorEnabled(aprilTag, true);

        if (isStopRequested()) return;

        Action dropPurpleSpike;
        Action goToBackdrop2;
        Action goToStack;
        Action goToBackdrop2Truss;
        if (conePos == 1) {
            dropPurpleSpike = trajectoryAction1;
            goToBackdrop2 = trajectoryAction1Back;
            goToBackdrop2Truss = trajectoryAction1Truss;
            goToBackdrop = drive.actionBuilder(new Pose2d(12, -33, Math.toRadians(160)))
                    .afterTime(0.8, new ParallelAction(
                                    autoActions.raiseArmUp(),
                                    autoActions.flipClawUp(),
                                    autoActions.autoLiftSlides()
                            )
                    )
                    .strafeToSplineHeading(new Vector2d(39, -35), Math.toRadians(180))
                    .build();
        } else if (conePos == 2) {
            dropPurpleSpike = trajectoryAction2;
            goToBackdrop2 = trajectoryAction2Back;
            goToBackdrop2Truss = trajectoryAction2Truss;
            goToBackdrop = drive.actionBuilder(new Pose2d(13, -34.5, Math.toRadians(90)))
                    .afterTime(0.8, new ParallelAction(
                                    autoActions.raiseArmUp(),
                                    autoActions.flipClawUp(),
                                    autoActions.autoLiftSlides()
                            )
                    )
                    .strafeToSplineHeading(new Vector2d(39, -35), Math.toRadians(180))
                    .build();
        } else {
            dropPurpleSpike = trajectoryAction3;
            goToBackdrop2 = trajectoryAction3Back;
            goToBackdrop2Truss = trajectoryAction3Truss;
            goToBackdrop = drive.actionBuilder(new Pose2d(35, -33, Math.toRadians(180)))
                    .afterTime(0.8, new ParallelAction(
                                    autoActions.raiseArmUp(),
                                    autoActions.flipClawUp(),
                                    autoActions.autoLiftSlides()
                            )
                    )
                    .strafeToSplineHeading(new Vector2d(39, -35), Math.toRadians(180))
                    .build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        new InstantAction(() -> intake.flipToStack()),
                        new ParallelAction(
                                autoActions.armToStack(),
                                dropPurpleSpike
                        ),
                        new SleepAction(0.1),
                        autoActions.openWideClawL(),
                        goToBackdrop
                )
        );

        sleep(400);

        drive.pose = new Pose2d(35.8, -32.84, drive.pose.heading.toDouble());
        drive.pose = getNewPose();

        if (!throughTruss)
        {
            if (stageCycle1){
                Actions.runBlocking(
                        new SequentialAction(
                                goToBackdrop2,
                                autoActions.closeClaw(),
                                new SleepAction(0.3),
                                new InstantAction(() -> intake.armOffGround()),
                                dropStage
                        )
                );
            }
            else {
                //thru center, drop on board, go back
                Actions.runBlocking(
                        new SequentialAction(
                                goToBackdrop2,
                                autoActions.closeClaw(),
                                new SleepAction(0.3),
                                new InstantAction(() -> intake.armOffGround()),
                                goToStage,
                                autoActions.openWideClawL(),
                                autoActions.openWideClawR(),
                                new SleepAction(0.5),
                                new ParallelAction(autoActions.lowerArmDown(),
                                        goToStack2Cycle)
                        )
                );
            }
            if (stageCycle2){
                Actions.runBlocking(
                        new SequentialAction(
                                autoActions.closeClaw(),
                                dropStageCycle2,
                                autoActions.openWideClawL(),
                                autoActions.openWideClawR(),
                                autoActions.flipClawUp(),
                                autoActions.lowerArmDown()
                        )
                );
            }
            else {
                //thru center, drop on board cycle 2
                Actions.runBlocking(
                        new SequentialAction(
                                autoActions.closeClaw(),
                                goToStage2Cycle,
                                autoActions.openWideClawL(),
                                autoActions.openWideClawR(),
                                new SleepAction(0.5),
                                new ParallelAction(autoActions.lowerArmDown(),
                                        new SequentialAction(new SleepAction(0.5),
                                                autoActions.flipClawUp()))
                        )
                );
            }
        }
        else{
            //thru truss, drop on board, go back
            if (stageCycle1){
                Actions.runBlocking(
                        new SequentialAction(
                                goToBackdrop2Truss,
                                autoActions.closeClaw(),
                                new SleepAction(0.3),
                                new InstantAction(() -> intake.armOffGround()),
                                dropStageTruss
                        )
                );
            }
            else {
                Actions.runBlocking(
                        new SequentialAction(
                                goToBackdrop2Truss,
                                autoActions.closeClaw(),
                                new SleepAction(0.3),
                                new InstantAction(() -> intake.armOffGround()),
                                goToStageTruss,
                                autoActions.openWideClawL(),
                                autoActions.openWideClawR(),
                                new SleepAction(0.5),
                                new ParallelAction(autoActions.lowerArmDown(),
                                        goToStackTrussCycle2)
                        )
                );
            }

            //thru truss, drop on board cycle 2
            Actions.runBlocking(
                    new SequentialAction(
                            autoActions.closeClaw(),
                            goToStageTrussCycle2,
                            autoActions.openWideClawL(),
                            autoActions.openWideClawR(),
                            new SleepAction(0.5),
                            new ParallelAction(autoActions.lowerArmDown(),
                                    new SequentialAction(new SleepAction(0.5),
                                            autoActions.flipClawUp()))
                    )
            );
        }
    }

    public Pose2d getNewPose(){
        boolean flag = false;
        for (int i = 0; i < 10; i++) {
            myAprilTagDetections = aprilTag.getDetections();
            boolean gotTag = false;
            for (AprilTagDetection detection : myAprilTagDetections) {

                if (detection.metadata != null && !gotTag) {  // This check for non-null Metadata is not needed for reading only ID code.
                    gotTag = true;
                    myAprilTagIdCode = detection.id;
                    range = detection.ftcPose.range*Math.cos(Math.toRadians(heading+11));
                    bearing = detection.ftcPose.bearing;
                    yaw = detection.ftcPose.yaw;
                    y = detection.ftcPose.y;

                    telemetry.addData("id", myAprilTagIdCode);
                    telemetry.addData("name", detection.metadata.name);
                    telemetry.addData("range", detection.ftcPose.range);
                    telemetry.addData("yaw", detection.ftcPose.yaw);
                    telemetry.addData("x", detection.ftcPose.x);
                    telemetry.addData("y", detection.ftcPose.y);

                    Vector2d myVector = new Vector2d(detection.ftcPose.x + 1, detection.ftcPose.y + 9);
//                    myVector.rotated(drive.getPoseEstimate().getHeading());
                    myVector = rotateVector(myVector, Math.toRadians(-yaw));

                    drive.updatePoseEstimate();


                    double myX = detection.metadata.fieldPosition.get(0) - myVector.y;
                    double myY = detection.metadata.fieldPosition.get(1) + myVector.x;
                    robotPose = new Pose2d(myX,myY,drive.pose.heading.toDouble());



                    telemetry.addData("Heading", -yaw);
                    telemetry.addData("RR Heading", Math.toDegrees(drive.pose.heading.toDouble()));
                    telemetry.addData("myX", myX);
                    telemetry.addData("myY", myY);
                    telemetry.update();

                    flag = true;
                    if (Math.abs(yaw) > 45) continue;
                    if (Math.abs(myX - drive.pose.position.x) > 10) continue;
                    if (Math.abs(myY - drive.pose.position.y) > 10) continue;
                    return robotPose;
                }
            }
        }
        if (!flag) telemetry.addData("Status", "DIDNT ");
        else telemetry.addData("Status", "very off");
        telemetry.update();
        return drive.pose;
    }

    public Vector2d rotateVector(Vector2d vector, double angle){
        return new Vector2d(Math.cos(angle)* vector.x - Math.sin(angle)* vector.y,
                Math.sin(angle) * vector.x + Math.cos(angle) * vector.y);
    }
}