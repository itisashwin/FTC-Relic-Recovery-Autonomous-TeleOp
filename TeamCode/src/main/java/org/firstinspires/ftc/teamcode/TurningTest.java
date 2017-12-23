package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * Created by Shreyas on 10/27/17.
 */
@Autonomous
public class TurningTest extends LinearOpMode{

    private DcMotor Motor1;
    private DcMotor Motor2;
    private DcMotor Motor3;
    private DcMotor Motor4;


    private BNO055IMU imu;

    public void gyroturn(double angle, double power, double k){

        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double initval = angles.firstAngle;

        if (angle > 0) {
            while((-angles.firstAngle - initval) < angle){

                double wtf =power*(1-(-angles.firstAngle/(k*angle)));
                telemetry.addData("meme", (angles.firstAngle - initval) );
                telemetry.addData("leg", wtf);
                telemetry.update();

                Motor1.setPower(wtf);
                Motor2.setPower(-wtf);
                Motor3.setPower(-wtf);
                Motor4.setPower(-wtf);
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            }
            Motor1.setPower(0);
            Motor2.setPower(0);
            Motor3.setPower(0);
            Motor4.setPower(0);

        }
        else{
            while((angles.firstAngle - initval) > angle){

                Motor1.setPower(-power*(1-(angles.firstAngle/(k*angle))));
                Motor2.setPower(power*(1-(angles.firstAngle/(k*angle))));
                Motor3.setPower(power*(1-(angles.firstAngle/(k*angle))));
                Motor4.setPower(power*(1-(angles.firstAngle/(k*angle))));
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            }
            Motor1.setPower(0);
            Motor2.setPower(0);
            Motor3.setPower(0);
            Motor4.setPower(0);

        }
    }
    public void gyroAlign(double pastAngle) {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentAngle = angles.firstAngle;
        double power = 0.5;
        if (currentAngle > pastAngle) {
            while((currentAngle - pastAngle) < 0){

                Motor1.setPower(power);
                Motor2.setPower(-power);
                Motor3.setPower(power);
                Motor4.setPower(power);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            }
            Motor1.setPower(0);
            Motor2.setPower(0);
            Motor3.setPower(0);
            Motor4.setPower(0);

        }
        else{
            while((pastAngle - currentAngle) > 0){

                Motor1.setPower(-power);
                Motor2.setPower(power);
                Motor3.setPower(-power);
                Motor4.setPower(power);
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            }
            Motor1.setPower(0);
            Motor2.setPower(0);
            Motor3.setPower(0);
            Motor4.setPower(0);

        }
    }




    @Override
    public void runOpMode() throws InterruptedException {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize, the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
            Motor1 = hardwareMap.dcMotor.get("Motor1");
            Motor2 = hardwareMap.dcMotor.get("Motor2");
            Motor3 = hardwareMap.dcMotor.get("Motor3");
            Motor4 = hardwareMap.dcMotor.get("Motor4");
            Motor1.setDirection(Direction.REVERSE);
            waitForStart();
            while (opModeIsActive()) {
                double meme = imu.getAngularOrientation().firstAngle;
//            Motor1.setPower(-0.48);
//            Motor2.setPower(-0.48);
//            Motor3.setPower(-0.48);
//            Motor4.setPower(0.48);
//            sleep(1500);
//            Motor1.setPower(0);
//            Motor2.setPower(0);
//            Motor3.setPower(0);
//            Motor4.setPower(0);
                //gyroAlign(meme);
                gyroturn(90, 0.6, 1.265);
                stop();


            }

        }
    }

