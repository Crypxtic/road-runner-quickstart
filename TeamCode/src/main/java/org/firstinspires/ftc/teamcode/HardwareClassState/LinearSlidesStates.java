package org.firstinspires.ftc.teamcode.HardwareClassState;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
public class LinearSlidesStates {
    public DcMotorEx ExtensionLeft;
    public DcMotorEx ExtensionRight;

    public static double ENCODER_PER_LVL = 0;
    public static double FIRST_LVL = 0;
    public static double MAX_TICKS = 1000000;

    public int leftPreset = 300;
    public int rightPreset = -300;

    public static double Kp = 10;
    public static double Ki = 5;
    public static double Kd = 0;
    public static double Kf = 10;

    public static double KpDown = 10;
    public static double KiDown = 0;
    public static double KdDown = 5;
    public static double KfDown = 10;

    PIDFCoefficients PID = new PIDFCoefficients(Kp, Ki, Kd, Kf);
    PIDFCoefficients PIDDown = new PIDFCoefficients(KpDown, KiDown, KdDown, KfDown);

    public int targetPositionL = 0;
    public int targetPositionR = 0;
    public double currentLevel = 1;

    public static double kG = 0.05;

    public boolean goingDown = false;

    public boolean isLifting = false;

    public ElapsedTime timer = new ElapsedTime();

    //lift up:
    //left ticks: 806
    //right ticks -807

    public double maxPower = 1;

    public void init(HardwareMap hardwareMap){

        ExtensionLeft = hardwareMap.get(DcMotorEx.class, "LiftLeft");
        ExtensionLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        ExtensionLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        ExtensionLeft.setPower(0);
        ExtensionLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        ExtensionLeft.setDirection(DcMotorEx.Direction.REVERSE);

        ExtensionRight = hardwareMap.get(DcMotorEx.class, "LiftRight");
        ExtensionRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        ExtensionRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        ExtensionRight.setPower(0);
        ExtensionRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        ExtensionRight.setDirection(DcMotorEx.Direction.REVERSE);

    }

//    public void setUpMotion(){
//        if (targetPositionL >= MAX_TICKS) targetPositionL = MAX_TICKS;
//        MotorProfileL = MotionProfileGenerator.generateSimpleMotionProfile(
//                new MotionState(ExtensionLeft.getCurrentPosition(), 0, 0),
//                new MotionState(targetPositionL, 0, 0),
//                maxVel,
//                maxAccel
//        );
//        MotorProfileR = MotionProfileGenerator.generateSimpleMotionProfile(
//                new MotionState(ExtensionRight.getCurrentPosition(), 0, 0),
//                new MotionState(targetPositionR, 0, 0),
//                maxVel,
//                maxAccel
//        );
//        MotorStateL = MotorProfileL.get(0);
//        MotorStateR = MotorProfileR.get(0);
//    }
//
//    public void updateMotionProfile(double seconds){
//        MotorStateL = MotorProfileL.get(seconds);
//        MotorStateR = MotorProfileR.get(seconds);
//
//        controllerL.setTargetPosition(MotorStateL.getX());
//        controllerR.setTargetPosition(MotorStateR.getX());
//
//        double correctionLeft = controllerL.update(ExtensionLeft.getCurrentPosition());
//        double correctionRight = controllerR.update(ExtensionRight.getCurrentPosition());
//
//        ExtensionLeft.setPower(correctionLeft + kG);
//        ExtensionRight.setPower(correctionRight - kG);
//    }

    public void resetSlides(){
        targetPositionL = 0;
        targetPositionR = 0;

        setPIDDown();

        ExtensionLeft.setTargetPosition(targetPositionL);
        ExtensionRight.setTargetPosition(targetPositionR);

        ExtensionLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExtensionRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ExtensionLeft.setVelocity(-2000);
        ExtensionRight.setVelocity(2000);

        goingDown = true;

        isLifting = true;
    }

    public void liftSlides(){

        targetPositionL = leftPreset;
        targetPositionR = rightPreset;

        setPIDUp();

        ExtensionLeft.setTargetPosition(targetPositionL);
        ExtensionRight.setTargetPosition(targetPositionR);

        ExtensionLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExtensionRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ExtensionLeft.setVelocity(2000);
        ExtensionRight.setVelocity(-2000);

        isLifting = true;
    }

    public void autoLiftSlides(){
        targetPositionL = 900;
        targetPositionR = -900;

        setPIDUp();

        ExtensionLeft.setTargetPosition(targetPositionL);
        ExtensionRight.setTargetPosition(targetPositionR);


        ExtensionLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExtensionRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ExtensionLeft.setVelocity(2000);
        ExtensionRight.setVelocity(-2000);

        isLifting = true;
    }

    public void smallLiftSlides(){
        targetPositionL = 300;
        targetPositionR = -300;

        setPIDUp();

        ExtensionLeft.setTargetPosition(targetPositionL);
        ExtensionRight.setTargetPosition(targetPositionR);


        ExtensionLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExtensionRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ExtensionLeft.setVelocity(2000);
        ExtensionRight.setVelocity(-2000);

        isLifting = true;
    }


    public boolean checkMode(){
        if (Math.abs(ExtensionLeft.getTargetPosition()-ExtensionLeft.getCurrentPosition()) < 10){
            ExtensionLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ExtensionRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            ExtensionLeft.setPower(0);
            ExtensionRight.setPower(0);

            isLifting = false;
            timer.reset();
            return true;
        }
        return false;
    }


    public boolean increaseLevel(){
        if (leftPreset < 1800){
            leftPreset += 300;
            rightPreset -= 300;
            return true;
        }
        else{
            return false;
        }
    }

    public boolean decreaseLevel(){
        if (leftPreset > 300){
            leftPreset -= 300;
            rightPreset += 300;
            return true;
        }
        else{
            return false;
        }
    }

//    public void liftUpSlides(){
//        targetPosition = FIRST_LVL + ENCODER_PER_LVL * (currentLevel - 1);
//        setUpMotion();
//    }

    public void moveSlides(double power){
        ExtensionLeft.setPower(Range.clip(power + kG, -maxPower, maxPower));
        ExtensionRight.setPower(Range.clip(-power - kG, -maxPower, maxPower));
    }

    public void resetLiftEncoder(){
        ExtensionRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ExtensionLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ExtensionRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ExtensionLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPIDUp(){
        PID = new PIDFCoefficients(Kp, Ki, Kd, Kf);
        ExtensionLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PID);
        ExtensionRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PID);

    }

    public void setPIDDown(){
        PIDDown = new PIDFCoefficients(KpDown, KiDown, KdDown, KfDown);
        ExtensionLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDDown);
        ExtensionRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDDown);

    }


}
