package org.firstinspires.ftc.teamcode.HardwareClassState;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ShooterStates {
    public Servo shooter;

    public static double SHOOTER_INIT = 1;
    public static double SHOOTER_OUT = 0.5;

    public void init(HardwareMap hardwareMap) {
        shooter = hardwareMap.servo.get("shooter");
    }

    public void shooterInit(){
        shooter.setPosition(SHOOTER_INIT);
    }

    public void shooterOut(){
        shooter.setPosition(SHOOTER_OUT);
    }
}
