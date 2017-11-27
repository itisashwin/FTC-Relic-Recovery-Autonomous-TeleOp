package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by Shreyas on 9/30/17.
 */

@TeleOp

public class ShittyClampTeleop extends LinearOpMode {
    private DcMotor Motor1;
    private DcMotor Motor2;
    private DcMotor Motor3;
    private DcMotor Motor4;
    private DcMotor Winch1;
    private DcMotor RelicArm;
    private Servo RightGrab;
    private Servo LeftGrab;
    private Servo JewelServo;





    @Override
    public void runOpMode() throws InterruptedException {

        Motor1 = hardwareMap.dcMotor.get("Motor1");
        Motor2 = hardwareMap.dcMotor.get("Motor2");
        Motor3 = hardwareMap.dcMotor.get("Motor3");
        Motor4 = hardwareMap.dcMotor.get("Motor4");
        Winch1 = hardwareMap.dcMotor.get("Winch1");
        RelicArm = hardwareMap.dcMotor.get("RelicArm");
        RightGrab = hardwareMap.servo.get("RightGrab");
        LeftGrab = hardwareMap.servo.get("LeftGrab");
        JewelServo = hardwareMap.servo.get("JewelServo");




        ///colorSensor = hardwareMap.colorSensor.get("sensor_color");

        JewelServo.setPosition(0.7);

        boolean meme = true;
        waitForStart();
        while (opModeIsActive()) {




            Motor1.setPower(gamepad1.left_stick_y);
            Motor3.setPower(gamepad1.left_stick_y);
            Motor2.setPower(gamepad1.right_stick_y);
            Motor4.setPower(-gamepad1.right_stick_y);






            if(gamepad2.dpad_up){
                RelicArm.setPower(0.30);
            }
            else if (gamepad2.dpad_down){
                RelicArm.setPower(-0.30);
            }
            else{
                RelicArm.setPower(0);
            }
            if(gamepad2.right_bumper){
                RightGrab.setPosition(0.3);
                LeftGrab.setPosition(0.6);
            }
            else if(gamepad2.left_bumper){
                RightGrab.setPosition(0.62);
                LeftGrab.setPosition(0.3);
            }



            if (gamepad2.dpad_down){
//              Hold tight asneef fvvvjmmmj    nmjmjmmmj fvv
                Winch1.setPower(0.3);
            }
            else if (gamepad2.dpad_up){
//           Big Shaq
                Winch1.setPower(-0.4);
            }
            else{
                Winch1.setPower(-0.03);
            }






        }


    }
}
