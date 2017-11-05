package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Shreyas on 9/30/17.
 */

@TeleOp

public class Teleop extends LinearOpMode {
    private DcMotor Motor1;
    private DcMotor Motor2;
    private DcMotor Motor3;
    private DcMotor Motor4;
    private CRServo LeftServo;
    private CRServo RightServo;
    private ColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        Motor1 = hardwareMap.dcMotor.get("Motor1");
        Motor2 = hardwareMap.dcMotor.get("Motor2");
        Motor3 = hardwareMap.dcMotor.get("Motor3");
        Motor4 = hardwareMap.dcMotor.get("Motor4");
        RightServo = hardwareMap.crservo.get("RightServo");
        LeftServo = hardwareMap.crservo.get("LeftServo");
        ///colorSensor = hardwareMap.colorSensor.get("sensor_color");



        waitForStart();
        while (opModeIsActive()) {

            Motor1.setPower(gamepad1.left_stick_y);
            Motor2.setPower(gamepad1.left_stick_y);
            Motor3.setPower(gamepad1.right_stick_y);
            Motor4.setPower(-gamepad1.right_stick_y);
            if (gamepad1.left_bumper == true){
                RightServo.setPower(1);
                LeftServo.setPower(-1);
            }
            else if(gamepad1.right_bumper == true){
                RightServo.setPower(-1);
                LeftServo.setPower(1);
            }
            else{
                RightServo.setPower(0);
                LeftServo.setPower(0);
            }
//            telemetry.addData("red", colorSensor.red());
//            telemetry.addData("blue", colorSensor.blue());
//            telemetry.addData("green", colorSensor.green());
//            telemetry.update();


        }


    }
}
