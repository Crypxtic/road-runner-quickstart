package org.firstinspires.ftc.teamcode.HardwareClassState;//package org.firstinspires.ftc.teamcode.HardwareClassState;
import androidx.annotation.NonNull;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class ActionAuto {

    IntakeStates intake;
    LinearSlidesStates slides;

    ElapsedTime armTimer = new ElapsedTime();
    ElapsedTime slideTimer = new ElapsedTime();

    public void init(IntakeStates newIntake, LinearSlidesStates newSlides, ElapsedTime arm, ElapsedTime slides){
        this.intake = newIntake;
        this.slides = newSlides;
        this.slideTimer = slides;
        this.armTimer = arm;
    }

    public void updateArm(double seconds){
        intake.updateState(seconds);
        intake.IntakeLeftFlip.setPosition(intake.armStateL.position);
        intake.IntakeRightFlip.setPosition(intake.armStateR.position);
    }

    public class armUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                intake.setUpServoProfile(intake.IntakeLeftFlip.getPosition(), intake.IntakeRightFlip.getPosition());
                initialized = true;
                armTimer.reset();
            }

            packet.addLine(armTimer.seconds()+"");
            updateArm(armTimer.seconds());
            return !intake.armProfileL.isFinished(armTimer.seconds());
        }
    }

    public Action raiseArmUp(){
        return new armUp();
    }

    public class armDown implements Action {
        private boolean initialized = false;
        boolean moveArmBack = false;
        boolean stopSlides = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                intake.setUpServoProfileDown(intake.IntakeLeftFlip.getPosition(), intake.IntakeRightFlip.getPosition());
                initialized = true;
                armTimer.reset();
                slides.resetLiftEncoder();
                slides.moveSlides(0.5);
            }
            if (armTimer.seconds() > 0.1) {
                if (!moveArmBack) {
                    armTimer.reset();
                    moveArmBack = true;
                }
                if (armTimer.seconds() > 0.4) {
                    slides.moveSlides(-1);
                    if (slides.ExtensionLeft.getCurrent(CurrentUnit.AMPS) > 4) {
                        slides.moveSlides(0);
                        stopSlides = true;
                    }
                }
                updateArm(armTimer.seconds());
            }
            return !intake.armProfileL.isFinished(armTimer.seconds()) || !stopSlides;
        }
    }

    public Action lowerArmDown(){
        return new armDown();
    }

    public class armFaceUp implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                intake.setUpServoProfileFaceUp(intake.IntakeLeftFlip.getPosition(), intake.IntakeRightFlip.getPosition());
                initialized = true;
                armTimer.reset();
            }

            updateArm(armTimer.seconds());
            return !intake.armProfileL.isFinished(armTimer.seconds());
        }
    }

    public Action raiseArmFaceUp(){
        return new armFaceUp();
    }

    public class armStack implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                intake.setUpServoProfileCustom(intake.IntakeLeftFlip.getPosition(), intake.IntakeRightFlip.getPosition(), intake.getStackLeft(), intake.getStackRight());
                initialized = true;
                armTimer.reset();
            }

            updateArm(armTimer.seconds());
            return !intake.armProfileL.isFinished(armTimer.seconds());
        }
    }

    public Action armToStack(){
        return new armStack();
    }

    public class armStackCycle1 implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                intake.setUpServoProfileCustom(intake.IntakeLeftFlip.getPosition(), intake.IntakeRightFlip.getPosition(), intake.getLeftStackCycle1(), intake.getRightStackCycle1());
                initialized = true;
                armTimer.reset();
            }

            updateArm(armTimer.seconds());
            if (intake.armProfileL.isFinished(armTimer.seconds())) {
                intake.IntakeRightFlip.setPosition(intake.getRightStackCycle1());
                intake.IntakeLeftFlip.setPosition(intake.getLeftStackCycle1());
            }
            return !intake.armProfileL.isFinished(armTimer.seconds());
        }
    }

    public Action armToStackCycle1(){
        return new armStackCycle1();
    }

    public class armStackCycle implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                intake.setUpServoProfileCustom(intake.IntakeLeftFlip.getPosition(), intake.IntakeRightFlip.getPosition(), intake.getLeftStackCycle(), intake.getRightStackCycle());
                initialized = true;
                armTimer.reset();
            }

            updateArm(armTimer.seconds());
            if (intake.armProfileL.isFinished(armTimer.seconds())) {
                intake.IntakeRightFlip.setPosition(intake.getRightStackCycle());
                intake.IntakeLeftFlip.setPosition(intake.getLeftStackCycle());
            }
            return !intake.armProfileL.isFinished(armTimer.seconds());
        }
    }

    public Action armToStackCycle(){
        return new armStackCycle();
    }

    public class armStack1 implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                intake.setUpServoProfileCustom(intake.IntakeLeftFlip.getPosition(), intake.IntakeRightFlip.getPosition(), intake.getLeftStack1(), intake.getRightStack1());
                initialized = true;
                armTimer.reset();
            }

            updateArm(armTimer.seconds());
            if (intake.armProfileL.isFinished(armTimer.seconds())) {
                intake.IntakeRightFlip.setPosition(intake.getRightStack1());
                intake.IntakeLeftFlip.setPosition(intake.getLeftStack1());
            }
            return !intake.armProfileL.isFinished(armTimer.seconds());
        }
    }

    public Action armToStack1(){
        return new armStack1();
    }

    public class CloseClaws implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.closeClawLeft();
            intake.closeClawRight();
            return false;
        }
    }
    public Action closeClaw() {
        return new CloseClaws();
    }

    public class OpenClawWideRight implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.openWideClawR();
            return false;
        }
    }
    public Action openWideClawR() {
        return new OpenClawWideRight();
    }

    public class OpenClawWideLeft implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.openWideClawL();
            return false;
        }
    }
    public Action openWideClawL() {
        return new OpenClawWideLeft();
    }

    public class FlipClawUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.flipClawUp();
            return false;
        }
    }
    public Action flipClawUp() {
        return new FlipClawUp();
    }

    public class FlipClawDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.flipClawDown();
            return false;
        }
    }
    public Action flipClawDown() {
        return new FlipClawDown();
    }

    public class FlipStack implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.flipToStack();
            intake.armToStack();
            return false;
        }
    }
    public Action intakeStack() {
        return new FlipStack();
    }

    public class FlipStack1 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.flipToStack();
            intake.armToStack1();
            return false;
        }
    }
    public Action intakeStack1() {
        return new FlipStack1();
    }

    public class LiftSlides implements Action {

        double setPointLeft = slides.leftPreset;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            slides.moveSlides(slides.slidesPID.calculate(slides.ExtensionLeft.getCurrentPosition(), setPointLeft));
            if (!(Math.abs(slides.ExtensionLeft.getCurrentPosition() - setPointLeft) < 10)) return true;
            else {
                slides.moveSlides(0.01);
                return false;
            }
        }
    }
    public Action liftSlides() {
        return new LiftSlides();
    }
    public class FarLiftSlides implements Action {
        double setPointLeft = 650;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            slides.moveSlides(slides.slidesPID.calculate(slides.ExtensionLeft.getCurrentPosition(), setPointLeft));
            if (!(Math.abs(slides.ExtensionLeft.getCurrentPosition() - setPointLeft) < 10)) return true;
            else {
                slides.moveSlides(0.01);
                return false;
            }
        }
    }
    public Action farLiftSlides() {
        return new FarLiftSlides();
    }

    public class AutoLiftSlides implements Action {
        double setPointLeft = 500;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            slides.moveSlides(slides.slidesPID.calculate(slides.ExtensionLeft.getCurrentPosition(), setPointLeft));
            if (!(Math.abs(slides.ExtensionLeft.getCurrentPosition() - setPointLeft) < 10)) return true;
            else {
                slides.moveSlides(0.01);
                return false;
            }
        }
    }
    public Action autoLiftSlides() {
        return new AutoLiftSlides();
    }
}
