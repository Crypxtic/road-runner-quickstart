package org.firstinspires.ftc.teamcode.OpModeStates;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClassState.IntakeStates;
import org.firstinspires.ftc.teamcode.HardwareClassState.LinearSlidesStates;

@Config
@Autonomous
public class StateCloseRed extends LinearOpMode {

    IntakeStates intake = new IntakeStates();
    LinearSlidesStates slides = new LinearSlidesStates();
    @Override
    public void runOpMode() throws InterruptedException {
        intake.init(hardwareMap);
        slides.init(hardwareMap);

    }

    public class armUp implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;
        ElapsedTime armTimer = new ElapsedTime();

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                intake.setUpServoProfile(intake.IntakeLeftFlip.getPosition(), intake.IntakeRightFlip.getPosition());
                initialized = true;
            }

            // checks lift's current position

            if (!intake.armProfileL.isFinished(armTimer.seconds())) {
                // true causes the action to rerun
                return true;
            } else {
                // false stops action rerun
                return false;
            }
            // overall, the action powers the lift until it surpasses
            // 3000 encoder ticks, then powers it off
        }
    }
}
