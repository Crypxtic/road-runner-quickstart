package org.firstinspires.ftc.teamcode.OpModeStates;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClassState.IntakeStates;

@Config
@TeleOp
public class ServoTestStates extends OpMode {

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    ElapsedTime timer = new ElapsedTime();

    IntakeStates intake = new IntakeStates();

    liftState myState = liftState.DOWN;

    enum liftState{
        DOWN,
        FLIPPING,
        LIFTING,
        UP
    }

    public static double open1 = 0;
    public static double open2 = 0;

    public static double close1 = 1;
    public static double close2 = 1;

    double armPosLeft = 0.5;
    double armPosRight = 0.5;

    double clawPos = 0.358;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake.init(hardwareMap);
    }

    @Override
    public void loop() {
        try {
            sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        dpadClawWrist();
        triggersTurnServo();

        telemetry.addData("Claw pos", clawPos);
        telemetry.addData("Left pos", armPosLeft);
        telemetry.addData("Right pos", armPosRight);
        telemetry.addData("Left range", intake.IntakeLeftFlip.getPwmRange().usPulseLower + " " + intake.IntakeLeftFlip.getPwmRange().usPulseUpper);
        telemetry.addData("Right range", intake.IntakeRightFlip.getPwmRange().usPulseLower + " " + intake.IntakeRightFlip.getPwmRange().usPulseUpper);
        telemetry.update();
    }

    public void dpadClawWrist(){
        double multi = 0;

        if (gamepad2.dpad_left) multi = -1;
        else if (gamepad2.dpad_right) multi = 1;

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
        armPosRight -= gamepad2.right_trigger/200;
        armPosRight += gamepad2.left_trigger/200;
        armPosLeft += gamepad2.right_trigger/200;
        armPosLeft -= gamepad2.left_trigger/200;

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

        if (!gamepad2.right_bumper) intake.IntakeLeftFlip.setPosition(armPosLeft);
        if (!gamepad2.left_bumper) intake.IntakeRightFlip.setPosition(armPosRight);

    }
}
