package org.firstinspires.ftc.teamcode.OpModeStates;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class ActiveTestStates extends OpMode {
    CRServo servo1;
    CRServo servo2;
    CRServo servo3;
    CRServo servo4;
    @Override
    public void init() {
        servo1 = hardwareMap.get(CRServo.class, "active1");
        servo2 = hardwareMap.get(CRServo.class, "active2");
        servo3 = hardwareMap.get(CRServo.class, "active3");
        servo4 = hardwareMap.get(CRServo.class, "active4");
    }

    @Override
    public void loop() {
        if (gamepad1.a){
            servo1.setPower(1);
            servo2.setPower(1);
            servo3.setPower(1);
            servo4.setPower(1);
        }
        else if (gamepad1.b){
            servo1.setPower(-1);
            servo2.setPower(-1);
            servo3.setPower(-1);
            servo4.setPower(-1);
        }
        else{
            servo1.setPower(0);
            servo2.setPower(0);
            servo3.setPower(0);
            servo4.setPower(0);
        }
    }
}
