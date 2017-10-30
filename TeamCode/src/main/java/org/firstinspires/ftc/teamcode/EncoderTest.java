package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

/**
 * Created by Shreyas on 10/27/17.
 */

public class EncoderTest extends LinearOpMode{

    DcMotor motor1 = null;
    DcMotor motor2 = null;

    public void driveForward(double distance, double power) {
        motor1.setMode(RunMode.RESET_ENCODERS);
        motor2.setMode(RunMode.RESET_ENCODERS);

        motor1.setTargetPosition((int) distance);
        motor2.setTargetPosition((int) distance);

        motor1.setMode(RunMode.RUN_TO_POSITION);
        motor1.setMode(RunMode.RUN_TO_POSITION);

        while (motor1.isBusy() && motor2.isBusy()) {
        }
    }




    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.dcMotor.get("Motor1");
        motor2 = hardwareMap.dcMotor.get("Motor2");
        motor1.setMode(RunMode.RUN_WITHOUT_ENCODERS);
        motor1.setMode(RunMode.RUN_USING_ENCODERS);
        motor1.setMode(RunMode.RUN_TO_POSITION);
        motor1.setMode(RunMode.RESET_ENCODERS);

        waitForStart();

        motor1.getCurrentPosition();
        motor1.setTargetPosition(450);
        motor1.isBusy();

    }
}
