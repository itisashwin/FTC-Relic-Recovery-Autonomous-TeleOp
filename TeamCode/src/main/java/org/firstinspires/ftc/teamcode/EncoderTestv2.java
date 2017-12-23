package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Shreyas on 12/1/17.
 */
@Autonomous
public class EncoderTestv2 extends EncoderTest{
    private DcMotor Motor1;
    private DcMotor Motor2;
    private DcMotor Motor3;
    private DcMotor Motor4;

    public void driveForward(int ticks, double power) {
        Motor1.setMode(DcMotor.RunMode.RESET_ENCODERS);
        Motor2.setMode(DcMotor.RunMode.RESET_ENCODERS);
        Motor3.setMode(DcMotor.RunMode.RESET_ENCODERS);
        Motor4.setMode(DcMotor.RunMode.RESET_ENCODERS);

        Motor1.setTargetPosition(-ticks);
        Motor2.setTargetPosition(ticks);
        Motor3.setTargetPosition(-ticks);
        Motor4.setTargetPosition(ticks);


        Motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        Motor1.setPower(power);
        Motor2.setPower(power);
        Motor3.setPower(power);
        Motor4.setPower(power);


        while (Motor2.isBusy() && Motor1.isBusy() && Motor3.isBusy() && Motor4.isBusy()) {
        }

        Motor1.setPower(0);
        Motor2.setPower(0);
        Motor3.setPower(0);
        Motor4.setPower(0);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        Motor1 = hardwareMap.dcMotor.get("Motor1");
        Motor2 = hardwareMap.dcMotor.get("Motor2");
        Motor3 = hardwareMap.dcMotor.get("Motor3");
        Motor4 = hardwareMap.dcMotor.get("Motor4");



        //Motor3.setDirection(Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            driveForward(1120, 0.3);
            stop();
        }
    }
}
