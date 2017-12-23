package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
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

    private DcMotor GlyphFlip;

    private DcMotor Winch1;

    private Servo JewelServo;

    private DcMotor GlyphMotor1;
    private DcMotor GlyphMotor2;

    private Servo Glyphstacker;
    private Servo Glyphstacker2;

    public void Flip(int ticks, double power) {
        GlyphFlip.setMode(DcMotor.RunMode.RESET_ENCODERS);

        GlyphFlip.setTargetPosition(ticks);

        GlyphFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        GlyphFlip.setPower(power);

        while (GlyphFlip.isBusy() ) {
        }

        GlyphFlip.setPower(0);


    }

    @Override
    public void runOpMode() throws InterruptedException {

        Motor1 = hardwareMap.dcMotor.get("Motor1");
        Motor2 = hardwareMap.dcMotor.get("Motor2");
        Motor3 = hardwareMap.dcMotor.get("Motor3");
        Motor4 = hardwareMap.dcMotor.get("Motor4");
        Motor2.setDirection(Direction.REVERSE);


        Winch1 = hardwareMap.dcMotor.get("Winch1");

        JewelServo = hardwareMap.servo.get("JewelServo");

        GlyphMotor1 = hardwareMap.dcMotor.get("GlyphMotor1");
        GlyphMotor2 = hardwareMap.dcMotor.get("GlyphMotor2");

        Glyphstacker = hardwareMap.servo.get("Glyphstacker");
        Glyphstacker2 = hardwareMap.servo.get("Glyphstacker2");
        GlyphFlip = hardwareMap.dcMotor.get("GlyphFlip");
        GlyphFlip.setMode(RunMode.RUN_USING_ENCODER);


        ///colorSensor = hardwareMap.colorSensor.get("sensor_color");

        JewelServo.setPosition(0.4);
        Glyphstacker.setPosition(0.08);

        boolean meme = false;
        waitForStart();
        while (opModeIsActive()) {



            if (meme==true) {

                Motor1.setPower(0.4*gamepad1.left_stick_y);
                Motor3.setPower(0.4*gamepad1.left_stick_y);
                Motor2.setPower(0.4*gamepad1.right_stick_y);
                Motor4.setPower(0.4*-gamepad1.right_stick_y);

            }
            else {
                Motor1.setPower(0.4*-gamepad1.right_stick_y);
                Motor3.setPower(0.4*-gamepad1.right_stick_y);
                Motor2.setPower(0.4*-gamepad1.left_stick_y);
                Motor4.setPower(0.4*gamepad1.left_stick_y);

            }

            if (gamepad1.a == true ) {
                if(meme==true) {
                    meme = false;
                    sleep(100);
                }
                else {
                    meme = true;
                    sleep(100);
                }
            }



            if (gamepad1.left_bumper == true) {
                    GlyphMotor1.setPower(0.75);
                    GlyphMotor2.setPower(-0.75);
                }
            else if (gamepad1.right_bumper == true){
                    GlyphMotor1.setPower(-0.75);
                    GlyphMotor2.setPower(0.75);
                }
            else {
                GlyphMotor1.setPower(0);
                GlyphMotor2.setPower(0);
            }



            if (gamepad2.left_bumper == true) {
//                MoveLift(1000,0.5);
                Winch1.setPower(0.6);
            }
            else if (gamepad2.right_bumper == true) {
//                MoveLift(-1000, 0.5);
                Winch1.setPower(-0.4);
            }
            else {
                Winch1.setPower(0.01);
            }



            //double UpPos = 0.8;
//            if (gamepad1.x == true) {
//                if(Glyphstacker.getPosition() == 0) {
//                    Glyphstacker.setPosition(0.6);
//                    Glyphstacker2.setPosition(0.6);
//                    sleep(300);
//                }
//                else {
//                    Glyphstacker.setPosition(0.05);
//                    Glyphstacker2.setPosition(0.05);
//                    sleep(300);
//                }
//            }


                if (gamepad2.b == true){
                    GlyphFlip.setPower(0.53);


                }
                else if(gamepad2.y == true){
                    GlyphFlip.setPower(-0.53);
                }

                else{
                    GlyphFlip.setPower(0);
                }




//            telemetry.addData("red", colorSensor.red());
//            telemetry.addData("blue", colorSensor.blue());
//            telemetry.addData("green", colorSensor.green());
//            telemetry.update();


        }


    }
}
