package org.firstinspires.ftc.teamcode.OpModeStates;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class MotorTestStates extends OpMode {
    DcMotorEx backLeftMotor;
    DcMotorEx backRightMotor;
    DcMotorEx frontLeftMotor;
    DcMotorEx frontRightMotor;

    @Override
    public void init() {
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "leftRear");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "rightRear");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");

//        distanceSensor = hardwareMap.get(DistanceSensor.class, "DistanceSensor");

        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);

        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_down){
            backLeftMotor.setPower(gamepad1.left_stick_x);
        }
        if (gamepad1.dpad_left){
            backRightMotor.setPower(gamepad1.left_stick_x);
        }
        if (gamepad1.dpad_right){
            frontLeftMotor.setPower(gamepad1.left_stick_x);
        }
        if (gamepad1.dpad_up){
            frontRightMotor.setPower(gamepad1.left_stick_x);
        }
    }
}
