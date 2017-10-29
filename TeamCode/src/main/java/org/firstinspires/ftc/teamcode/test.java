package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Shreyas on 10/15/17.
 */
@TeleOp
public class test extends LinearOpMode {
    private DcMotor Motor1;
    private CRServo RightServo;


    @Override
    public void runOpMode() throws InterruptedException {
        Motor1 = hardwareMap.dcMotor.get("Motor1");
        RightServo = hardwareMap.crservo.get("RightServo");
//        RightServo.setPosition(1);
        waitForStart();

//        RightServo.setPosition(0);
        Motor1.setPower(1);
        sleep(10);


    }
}
