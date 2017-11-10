package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by Shreyas on 9/30/17.
 */

@TeleOp

public class Teleop extends LinearOpMode {
    private DcMotor Motor1;
    private DcMotor Motor2;
    private DcMotor Motor3;
    private DcMotor Motor4;
    private DcMotor Winch1;
    private DcMotor Winch2;
    private CRServo LeftServo;
    private CRServo RightServo;
    private Servo GlyphGrabber;
    private Servo RightGrab;
    private Servo LeftGrab;

    public void MoveLift(double distance, double power) {
        Winch1.setMode(RunMode.RESET_ENCODERS);
        Winch2.setMode(RunMode.RESET_ENCODERS);

        Winch1.setTargetPosition(-(int) distance);
        Winch2.setTargetPosition((int) distance);

        Winch1.setMode(RunMode.RUN_TO_POSITION);
        Winch2.setMode(RunMode.RUN_TO_POSITION);

        Winch1.setPower(power);
        Winch2.setPower(power);

        while (Winch1.isBusy() && Winch2.isBusy()) {
        }
    }




    @Override
    public void runOpMode() throws InterruptedException {

        Motor1 = hardwareMap.dcMotor.get("Motor1");
        Motor2 = hardwareMap.dcMotor.get("Motor2");
        Motor3 = hardwareMap.dcMotor.get("Motor3");
        Motor4 = hardwareMap.dcMotor.get("Motor4");
        Winch1 = hardwareMap.dcMotor.get("Winch1");
        Winch2 = hardwareMap.dcMotor.get("Winch2");
        RightServo = hardwareMap.crservo.get("RightServo");
        LeftServo = hardwareMap.crservo.get("LeftServo");
        GlyphGrabber = hardwareMap.servo.get("GlyphGrabber");
        RightGrab = hardwareMap.servo.get("RightGrab");
        LeftGrab = hardwareMap.servo.get("LeftGrab");





        ///colorSensor = hardwareMap.colorSensor.get("sensor_color");




        waitForStart();
        while (opModeIsActive()) {

            Motor1.setPower(gamepad1.left_stick_y);
            Motor3.setPower(gamepad1.left_stick_y);
            Motor2.setPower(gamepad1.right_stick_y);
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
                LeftServo.setPower(-0.02);
            }
            if (gamepad1.dpad_up == true){
//                MoveLift(1000,0.5);
                Winch1.setPower(0.3);
                Winch2.setPower(-0.3);

            }
            else if (gamepad1.dpad_down == true){
//                MoveLift(-1000, 0.5);
                Winch1.setPower(-0.3);
                Winch2.setPower(0.3);
            }
            else{
                Winch1.setPower(0.01);
                Winch2.setPower(-0.01);
            }
            if(gamepad1.y == true){
                if (GlyphGrabber.getPosition() == 0.78 ){
                    GlyphGrabber.setPosition(0.46);
                    sleep(300);
                }
                else{
                    GlyphGrabber.setPosition(0.78);
                    sleep(300);
                }
            }
            if (gamepad1.b == true){
                if(RightGrab.getPosition() == 0.1){
                    LeftGrab.setPosition(0.4);
                    RightGrab.setPosition(0.35);
                    sleep(300);
                }
                else{
                    RightGrab.setPosition(0.8);
                    LeftGrab.setPosition(0.9);
                    sleep(300);
                }
            }



//            telemetry.addData("red", colorSensor.red());
//            telemetry.addData("blue", colorSensor.blue());
//            telemetry.addData("green", colorSensor.green());
//            telemetry.update();


        }


    }
}
