package org.firstinspires.ftc.teamcode.HardwareClassState;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HangerStates {
    public DcMotorEx hangerMotorLeft;
    public DcMotorEx hangerMotorRight;

    public void init(HardwareMap hardwareMap) {
        hangerMotorLeft = hardwareMap.get(DcMotorEx.class, "HangerLeft");
        hangerMotorRight = hardwareMap.get(DcMotorEx.class, "HangerRight");

        hangerMotorLeft.setPower(0);
        hangerMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangerMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangerMotorLeft.setTargetPosition(0);
        hangerMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hangerMotorRight.setPower(0);
        hangerMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangerMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangerMotorRight.setTargetPosition(0);
        hangerMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void fullSpeed(){
        hangerMotorLeft.setPower(1);
    }

    public void fullHalt(){
        hangerMotorLeft.setPower(0);
    }
}
