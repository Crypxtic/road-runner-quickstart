package org.firstinspires.ftc.teamcode.HardwareClassState;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class IntakeStates {
    public Servo ClawRight;
    public Servo IntakeWrist;
    public ServoImplEx IntakeLeftFlip;
    public ServoImplEx IntakeRightFlip;
    public Servo PurpleStick;
    public Servo ClawLeft;

    public TrapezoidProfile.Constraints constraints =
            new TrapezoidProfile.Constraints(0.9, 1);

    public static double CLAW_OPEN_R = 0.67;
    public static double CLAW_CLOSE_R = 0.49;

    public static double CLAW_OPEN_L = 0.2;
    public static double CLAW_CLOSE_L = 0.4;

    public static double CLAW_WIDE_OPEN_L = 0.12;
    public static double CLAW_WIDE_OPEN_R = 0.77;

    public static double FLIP_DOWN = 0.528;
    public static double FLIP_UP = 0.7;

    public static double FLIP_FACE_UP = 0.3;

    public static double FLIP_FACE_UP_SINGLE = 0.3;

    public static double FLIP_STACK = 0.526;//358

    public static double LEFT_UP = 0.968; //0.778
    public static double LEFT_DOWN = 0.291;
    public static double RIGHT_UP = 0.021; //0.222
    public static double RIGHT_DOWN = 0.698;

    //claw: 0.4
    //left arm: 0.076
    //right arm: 0.977

    public static double LEFT_LIFTED = 0.382;
    public static double RIGHT_LIFTED = 0.607;

    public static double LEFT_FACE_UP = 0.933;
    public static double RIGHT_FACE_UP = 0.056;



    public static double LEFT_STACK = 0.318;
    public static double RIGHT_STACK = 0.671;

    //2nd cycle
    //claw: 0.366
    //Left: 0.15
    //Right: 0.91

    public static double LEFT_STACK_1 = 0.328;
    public static double RIGHT_STACK_1 = 0.661;

    public static double LEFT_STACK_CYCLE = 0.298;
    public static double RIGHT_STACK_CYCLE = 0.681;

    public static double LEFT_STACK_CYCLE_1 = 0.305;
    public static double RIGHT_STACK_CYCLE_1 = 0.674;

    public static double LEFT_HALF = 0.68;
    public static double RIGHT_HALF = 0.32;


    public TrapezoidProfile armProfileL;

    public TrapezoidProfile armProfileR;

    public TrapezoidProfile.State armStateL;
    public TrapezoidProfile.State armStateR;


    public void init(HardwareMap hardwareMap) {
        ClawLeft = hardwareMap.servo.get("ClawLeft");
        ClawRight = hardwareMap.servo.get("ClawRight");
        IntakeWrist = hardwareMap.servo.get("IntakeWrist");
        IntakeLeftFlip = hardwareMap.get(ServoImplEx.class, "LeftFlip");
        IntakeRightFlip = hardwareMap.get(ServoImplEx.class,"RightFlip");
    }

    public void openClawLeft(){ClawLeft.setPosition(CLAW_OPEN_L);}

    public void closeClawLeft(){ClawLeft.setPosition(CLAW_CLOSE_L);}

    public void openClawRight(){ClawRight.setPosition(CLAW_OPEN_R);}

    public void closeClawRight(){ClawRight.setPosition(CLAW_CLOSE_R);}

    public void flipClawUp(){
        IntakeWrist.setPosition(FLIP_UP);
    }

    public double getFlipUp(){
        return FLIP_UP;
    }

    public void flipClawDown(){
        IntakeWrist.setPosition(FLIP_DOWN);
    }

    public double getFlipDown(){
        return FLIP_DOWN;
    }

    public void flipArmDown(){
        IntakeRightFlip.setPosition(RIGHT_DOWN);
        IntakeLeftFlip.setPosition(LEFT_DOWN);
    }

    public double getFlipFaceUpSingle() {return FLIP_FACE_UP_SINGLE;}

    public double getRightDown(){
        return RIGHT_DOWN;
    }

    public double getLeftDown(){
        return LEFT_DOWN;
    }

    public void flipArmUp(){
        IntakeRightFlip.setPosition(RIGHT_UP);
        IntakeLeftFlip.setPosition(LEFT_UP);
    }

    public double getRightUp(){
        return RIGHT_UP;
    }

    public double getLeftUp(){
        return LEFT_UP;
    }

    public void armOffGround(){
        IntakeRightFlip.setPosition(RIGHT_LIFTED);
        IntakeLeftFlip.setPosition(LEFT_LIFTED);
    }

    public double getRightLifted(){
        return RIGHT_LIFTED;
    }

    public double getLeftLifted(){
        return LEFT_LIFTED;
    }

    public double getStackLeft() {return LEFT_STACK;}

    public double getStackRight() {return RIGHT_STACK;}

    public double getStackFlip() {return FLIP_STACK;}

    public void autoStack(){
        IntakeRightFlip.setPosition(0.55);
        IntakeLeftFlip.setPosition(0.395);
    }

    public void autoStack2(){
        IntakeRightFlip.setPosition(0.5648);
        IntakeLeftFlip.setPosition(0.3801);
    }

    public void wristFaceUp(){
        IntakeWrist.setPosition(FLIP_FACE_UP);
    }

    public void armFaceUp(){
        IntakeRightFlip.setPosition(RIGHT_FACE_UP);
        IntakeLeftFlip.setPosition(LEFT_FACE_UP);
    }

    public void armFaceUp2(){
        IntakeRightFlip.setPosition(RIGHT_FACE_UP);
        IntakeLeftFlip.setPosition(LEFT_FACE_UP);
    }

    public double getFlipFaceUp(){
        return FLIP_FACE_UP;
    }

    public double getLeftFaceUp(){
        return LEFT_FACE_UP;
    }

    public double getRightFaceUp(){
        return RIGHT_FACE_UP;
    }

    public double getRightStackCycle() {return RIGHT_STACK_CYCLE;}

    public double getLeftStackCycle() {return LEFT_STACK_CYCLE;}

    public double getRightStackCycle1() {return RIGHT_STACK_CYCLE_1;}

    public double getLeftStackCycle1() {return LEFT_STACK_CYCLE_1;}

    public double getRightStack1() {return RIGHT_STACK_1;}

    public double getLeftStack1() {return LEFT_STACK_1;}


    public void openWideClawR(){ ClawRight.setPosition(CLAW_WIDE_OPEN_R);}

    public void openWideClawL(){ ClawLeft.setPosition(CLAW_WIDE_OPEN_L);}

    public void armToStack(){
        IntakeLeftFlip.setPosition(LEFT_STACK);
        IntakeRightFlip.setPosition(RIGHT_STACK);
    }

    public void armToStack1(){
        IntakeLeftFlip.setPosition(LEFT_STACK_1);
        IntakeRightFlip.setPosition(RIGHT_STACK_1);
    }

    public void armToCycle(){
        IntakeLeftFlip.setPosition(LEFT_STACK_CYCLE);
        IntakeRightFlip.setPosition(RIGHT_STACK_CYCLE);
    }

    public void armToCycle1(){
        IntakeLeftFlip.setPosition(LEFT_STACK_CYCLE_1);
        IntakeRightFlip.setPosition(RIGHT_STACK_CYCLE_1);
    }

    public void armHalfway(){
        IntakeLeftFlip.setPosition(LEFT_HALF);
        IntakeRightFlip.setPosition(RIGHT_HALF);
    }

    public double getLeftHalf(){
        return LEFT_HALF;
    }

    public double getRightHalf(){
        return RIGHT_HALF;
    }

    public void flipToStack(){
        IntakeWrist.setPosition(FLIP_STACK);
    }

    public void flipFace2(){
            IntakeWrist.setPosition(FLIP_STACK);
    }

    public void setUpServoProfile(double currentPosL, double currentPosR){

        armProfileL =
                new TrapezoidProfile(constraints, new TrapezoidProfile.State(LEFT_UP, 0),
                        new TrapezoidProfile.State(currentPosL, 0)
                );

        armProfileR =
                new TrapezoidProfile(constraints, new TrapezoidProfile.State(RIGHT_UP, 0),
                        new TrapezoidProfile.State(currentPosR, 0)
                );


        armStateL = armProfileL.calculate(0);
        armStateR = armProfileR.calculate(0);
    }

    public void setUpServoProfileDown(double currentPosL, double currentPosR){
        armProfileL =
                new TrapezoidProfile(constraints, new TrapezoidProfile.State(LEFT_LIFTED, 0),
                        new TrapezoidProfile.State(currentPosL, 0)
                );

        armProfileR =
                new TrapezoidProfile(constraints, new TrapezoidProfile.State(RIGHT_LIFTED, 0),
                        new TrapezoidProfile.State(currentPosR, 0)
                );


        armStateL = armProfileL.calculate(0);
        armStateR = armProfileR.calculate(0);
    }

    public void setUpServoProfileFaceUp(double currentPosL, double currentPosR){
        armProfileL =
                new TrapezoidProfile(constraints, new TrapezoidProfile.State(LEFT_FACE_UP, 0),
                        new TrapezoidProfile.State(currentPosL, 0)
                );

        armProfileR =
                new TrapezoidProfile(constraints, new TrapezoidProfile.State(RIGHT_FACE_UP, 0),
                        new TrapezoidProfile.State(currentPosR, 0)
                );


        armStateL = armProfileL.calculate(0);
        armStateR = armProfileR.calculate(0);
    }

    public void setUpServoProfileCustom(double currentPosL, double currentPosR, double targetL, double targetR){
        armProfileL =
                new TrapezoidProfile(constraints, new TrapezoidProfile.State(targetL, 0),
                        new TrapezoidProfile.State(currentPosL, 0)
                );

        armProfileR =
                new TrapezoidProfile(constraints, new TrapezoidProfile.State(targetR, 0),
                        new TrapezoidProfile.State(currentPosR, 0)
                );


        armStateL = armProfileL.calculate(0);
        armStateR = armProfileR.calculate(0);
    }

    public void updateState(double seconds){
        armStateL = armProfileL.calculate(seconds);
        armStateR = armProfileR.calculate(seconds);
    }

    public void clawAuto(){
        ClawLeft.setPosition(0.5);
        ClawRight.setPosition(0.4);
    }


}
