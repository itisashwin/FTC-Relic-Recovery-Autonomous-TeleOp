package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;


/**
 * Created by Shreyas on 10/27/17.
 */
@Autonomous
public class TurningTest extends LinearOpMode{

    private DcMotor Motor1;
    private DcMotor Motor2;
    private DcMotor Motor3;
    private DcMotor Motor4;


    public void turnRightAngle(double power, double angle) {

        while (SOMETHING < angle) {
            Motor1.setPower(power);
            Motor2.setPower(-power);
            Motor3.setPower(power);
            Motor4.setPower(power);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        turnRightAngle(0.5, 90);


    }
}
