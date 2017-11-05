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

    private DcMotor Winch1;
    private DcMotor Winch2;

    public void driveForward(double distance, double power) {
        Winch1.setMode(RunMode.RESET_ENCODERS);
        Winch2.setMode(RunMode.RESET_ENCODERS);

        Winch1.setTargetPosition(-(int) distance);
        Winch2.setTargetPosition((int) distance);

        Winch1.setMode(RunMode.RUN_TO_POSITION);
        Winch2.setMode(RunMode.RUN_TO_POSITION);

        Winch1.setPower(power);
        Winch2.setPower(-power);

        while (Winch1.isBusy() && Winch2.isBusy()) {
        }
    }




    @Override
    public void runOpMode() throws InterruptedException {
        Winch1 = hardwareMap.dcMotor.get("Winch1");
        Winch2 = hardwareMap.dcMotor.get("Winch2");
        driveForward(1000,0.5);


    }
}
