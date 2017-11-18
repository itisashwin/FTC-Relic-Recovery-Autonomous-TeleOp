package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

/**
 * Created by Shreyas on 10/27/17.
 */
@Autonomous
public class EncoderTest extends LinearOpMode{

    private DcMotor Motor1;
    private DcMotor Motor2;
    private DcMotor Motor3;
    private DcMotor Motor4;



    public void driveForward(int seconds, double power) {
        Motor1.setMode(RunMode.RESET_ENCODERS);
        Motor2.setMode(RunMode.RESET_ENCODERS);
        Motor3.setMode(RunMode.RESET_ENCODERS);
        Motor4.setMode(RunMode.RESET_ENCODERS);



        Motor1.setMode(RunMode.RUN_USING_ENCODER);
        Motor2.setMode(RunMode.RUN_USING_ENCODER);
        Motor3.setMode(RunMode.RUN_USING_ENCODER);
        Motor4.setMode(RunMode.RUN_USING_ENCODER);


        Motor1.setPower(power);
        Motor2.setPower(power);
        Motor3.setPower(power);
        Motor4.setPower(-power);

        sleep(seconds);

        Motor1.setPower(0);
        Motor2.setPower(0);
        Motor3.setPower(0);
        Motor4.setPower(0);

        while (Motor1.isBusy() & Motor2.isBusy()) {

        }
    }




    @Override
    public void runOpMode() throws InterruptedException {
        Motor1 = hardwareMap.dcMotor.get("Motor1");
        Motor2 = hardwareMap.dcMotor.get("Motor2");
        Motor3 = hardwareMap.dcMotor.get("Motor3");
        Motor4 = hardwareMap.dcMotor.get("Motor4");

        waitForStart();
        driveForward(600,0.8);
        sleep(500);
        driveForward(600,-0.8);
//        Motor1.setPower(0.7);
//        Motor2.setPower(0.7);
//        Motor3.setPower(0.7);
//        Motor4.setPower(-0.7);
//        sleep(2000);
//        Motor1.setPower(0);
//        Motor2.setPower(0);
//        Motor3.setPower(0);
//        Motor4.setPower(0);


        stop();


    }
}
