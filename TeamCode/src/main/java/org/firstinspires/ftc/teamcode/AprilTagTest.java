package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.EOCV.TeamPropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class AprilTagTest extends LinearOpMode {
    AprilTagLibrary.Builder myAprilTagLibraryBuilder;
    AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    AprilTagLibrary myAprilTagLibrary;
    AprilTagProcessor myAprilTagProcessor;
    VisionPortal.Builder myVisionPortalBuilder;
    VisionPortal myVisionPortal;

    List<AprilTagDetection> myAprilTagDetections;  // list of all detections// current detection in for() loop
    int myAprilTagIdCode;
    double range;
    double bearing;
    double heading;
    double yaw;
    double currentxPos;
    double targetxMove;
    double y;

    double camToCenter = 4.375;
    double rightAprilTagPos = -12;
    double centerAprilTagPos = -6;
    double leftAprilTagPos = 0;
    double desiredYPos = 8;
    double conePos = 1;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    IMU imu;

    Pose2d robotPose = new Pose2d(0, 0, 0);
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        TeamPropPipeline propCV = new TeamPropPipeline(telemetry);


        // Create a new AprilTagLibrary.Builder object and assigns it to a variable.
        myAprilTagLibraryBuilder = new AprilTagLibrary.Builder();

// Add all the tags from the given AprilTagLibrary to the AprilTagLibrary.Builder.
// Get the AprilTagLibrary for the current season.
        myAprilTagLibraryBuilder.addTags(AprilTagGameDatabase.getCenterStageTagLibrary());

// Build the AprilTag library and assign it to a variable.
        myAprilTagLibrary = myAprilTagLibraryBuilder.build();

// Create a new AprilTagProcessor.Builder object and assign it to a variable.
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();

// Set the tag library.
        myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);

// Build the AprilTag processor and assign it to a variable.
        myAprilTagProcessor = myAprilTagProcessorBuilder.build();

        // Create a new VisionPortal Builder object.
        myVisionPortalBuilder = new VisionPortal.Builder();

// Specify the camera to be used for this VisionPortal.
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "backCam"));      // Other choices are: RC phone camera and "switchable camera name".

// Add the AprilTag Processor to the VisionPortal Builder.
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        // An added Processor is enabled by default.

// Optional: set other custom features of the VisionPortal (4 are shown here).
        myVisionPortalBuilder.setCameraResolution(new Size(640, 480));  // Each resolution, for each camera model, needs calibration values for good pose estimation.
        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);  // MJPEG format uses less bandwidth than the default YUY2.
        myVisionPortalBuilder.enableLiveView(true);      // Enable LiveView (RC preview).
        myVisionPortalBuilder.setAutoStopLiveView(true);     // Automatically stop LiveView (RC preview) when all vision processors are disabled.

// Create a VisionPortal by calling build()
        myVisionPortal = myVisionPortalBuilder.build();


        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );

        imu.resetYaw();

        while(!isStarted()){

// Cycle through through the list and process each AprilTag.
            myAprilTagDetections = myAprilTagProcessor.getDetections();
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
//                    myVector = myVector.rotated(Math.toRadians(-yaw));



                    double myX = detection.metadata.fieldPosition.get(0) - myVector.x;
                    double myY = detection.metadata.fieldPosition.get(1) + myVector.y;
                    robotPose = new Pose2d(myX,myY,Math.toRadians(180-yaw));

                    YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();

// Now use these simple methods to extract each angle
// (Java type double) from the object you just created:
                    double botHeading  = -robotOrientation.getYaw(AngleUnit.RADIANS);

                    //deals with cases like when both offset and heading are negative its super convoluted but if you think about it for like 20 hours it makes sense it only took me 20 hours to make :)
                    if (Math.toDegrees(botHeading)<-180){
                        botHeading = Math.toRadians(180 - ((Math.abs(Math.toDegrees(botHeading))) - 180));
                    }
                    if (Math.toDegrees(botHeading) >= 180){
                        botHeading = Math.toRadians((Math.toDegrees(botHeading) - 180)-180);
                    }

                    telemetry.addData("Heading", -yaw);
                    telemetry.addData("IMU Heading", Math.toDegrees(botHeading));
                    telemetry.addData("myX", myX);
                    telemetry.addData("myY", myY);
                    telemetry.update();

                }
            }

            try {
                // Push telemetry to the Driver Station.
                TelemetryPacket packet = new TelemetryPacket();
            drawRobot(packet.fieldOverlay(), robotPose);
//                packet.fieldOverlay().strokeCircle(robotPose.getX(), robotPose.getY(), 10);
                dashboard.sendTelemetryPacket(packet);
            }
            catch (RuntimeException e) {
                e.printStackTrace();
            }
        }

    }
    public static void drawRobot(Canvas canvas, Pose2d pose) {
        canvas.strokeCircle(pose.position.x, pose.position.y, 9);
//        Vector2d v = pose.headingVec().times(9);
//        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
//        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
//        canvas.strokeLine(x1, y1, x2, y2);
    }
}
