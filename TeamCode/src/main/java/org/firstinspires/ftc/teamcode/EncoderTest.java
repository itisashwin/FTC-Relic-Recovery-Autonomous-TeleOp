package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

/**
 * Created by Shreyas on 10/27/17.
 */
@Autonomous
public class EncoderTest extends LinearOpMode{

    private DcMotor Motor1;
    private DcMotor Motor2;
    private DcMotor Motor3;
    private DcMotor Motor4;
    private BNO055IMU imu;

    public void driveForward(int ticks, double power) {
//        Motor1.setMode(RunMode.RESET_ENCODERS);
        Motor2.setMode(DcMotor.RunMode.RESET_ENCODERS);
//        Motor3.setMode(DcMotor.RunMode.RESET_ENCODERS);
//        Motor4.setMode(RunMode.RESET_ENCODERS);

//        Motor1.setTargetPosition(ticks);
        Motor2.setTargetPosition(ticks);
        //Motor3.setTargetPosition(ticks);




//        Motor1.setMode(RunMode.RUN_TO_POSITION);
        Motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        Motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        Motor4.setMode(RunMode.RUN_TO_POSITION);




//        Motor1.setPower(power);
        Motor2.setPower(power);
//        Motor3.setPower(power);
//        Motor4.setPower(power);


        while (Motor2.isBusy()){
            double meme = Motor2.getPower();
            Motor1.setPower(meme);
            Motor3.setPower(meme);
            Motor4.setPower(meme);


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
        Motor4.setDirection(Direction.REVERSE);
        BNO055IMU.Parameters gyroParam = new BNO055IMU.Parameters();
        gyroParam.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        gyroParam.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParam.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gyroParam.loggingEnabled      = true;
        gyroParam.loggingTag          = "IMU";
        gyroParam.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize, the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(gyroParam);
        waitForStart();

        while (opModeIsActive()){
            driveForward(1000,0.3);
            stop();

        }

//        Motor1.setPower(0.7);
//        Motor2.setPower(0.7);
//        Motor3.setPower(0.7);
//        Motor4.setPower(-0.7);
//        sleep(2000);
//        Motor1.setPower(0);
//        Motor2.setPower(0);
//        Motor3.setPower(0);
//        Motor4.setPower(0);
        /*gyroturn(75, 0.48);
        sleep(500);
        gyroturn(43, 0.48);
        sleep(500);*/
        // driveForward(2000, 0.5);
        //sleep(500);
        //driveForward(3000, 0.5);



    }
}
