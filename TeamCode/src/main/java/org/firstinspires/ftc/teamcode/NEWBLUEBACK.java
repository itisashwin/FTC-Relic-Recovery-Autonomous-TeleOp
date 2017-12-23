package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * Created by Ashwin N on 11/5/2017.
 */

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */

@Autonomous

public class NEWBLUEBACK extends LinearOpMode {

    private DcMotor Motor1;
    private DcMotor Motor2;
    private DcMotor Motor3;
    private DcMotor Motor4;

    private DcMotor GlyphFlip;
    private Servo Arm;
    private BNO055IMU imu;



    private ColorSensor colorSensor;


    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    public void Flip(int ticks, double power) {
        GlyphFlip.setMode(RunMode.RESET_ENCODERS);

        GlyphFlip.setTargetPosition(ticks);

        GlyphFlip.setMode(RunMode.RUN_TO_POSITION);

        GlyphFlip.setPower(power);

        while (GlyphFlip.isBusy() ) {
        }

        GlyphFlip.setPower(0);


    }

    public void gyroturn(double angle, double power, double k){

        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double initval = angles.firstAngle;

        Motor1.setMode(RunMode.RUN_USING_ENCODER);
        Motor2.setMode(RunMode.RUN_USING_ENCODER);
        Motor3.setMode(RunMode.RUN_USING_ENCODER);
        Motor4.setMode(RunMode.RUN_USING_ENCODER);

        if (angle > 0) {
            while((angles.firstAngle - initval) < angle){

                double powerNew =power*(1-((angles.firstAngle - initval)/(k*angle)));
                telemetry.addData("meme", (angles.firstAngle - initval) );
                telemetry.addData("leg", powerNew);
                telemetry.update();

                Motor1.setPower(powerNew);
                Motor2.setPower(powerNew);
                Motor3.setPower(powerNew);
                Motor4.setPower(powerNew);
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            }
            Motor1.setPower(0);
            Motor2.setPower(0);
            Motor3.setPower(0);
            Motor4.setPower(0);

            }
        else {
            while((angles.firstAngle - initval) > angle){
                double powerNew =power*(1-((angles.firstAngle-initval)/(k*angle)));
                Motor1.setPower(-powerNew);
                Motor2.setPower(-powerNew);
                Motor3.setPower(-powerNew);
                Motor4.setPower(-powerNew);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            }
            Motor1.setPower(0);
            Motor2.setPower(0);
            Motor3.setPower(0);
            Motor4.setPower(0);

        }
    }
    public void driveForward(int ticks, double power) {
        Motor1.setMode(RunMode.RESET_ENCODERS);
        Motor2.setMode(RunMode.RESET_ENCODERS);
        Motor3.setMode(RunMode.RESET_ENCODERS);
        Motor4.setMode(RunMode.RESET_ENCODERS);

        Motor1.setTargetPosition(-ticks);
        Motor2.setTargetPosition(ticks);
        Motor3.setTargetPosition(-ticks);
        Motor4.setTargetPosition(ticks);


        Motor1.setMode(RunMode.RUN_TO_POSITION);
        Motor2.setMode(RunMode.RUN_TO_POSITION);
        Motor3.setMode(RunMode.RUN_TO_POSITION);
        Motor4.setMode(RunMode.RUN_TO_POSITION);


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

        Motor1.setMode(RunMode.RUN_USING_ENCODER);
        Motor2.setMode(RunMode.RUN_USING_ENCODER);
        Motor3.setMode(RunMode.RUN_USING_ENCODER);
        Motor4.setMode(RunMode.RUN_USING_ENCODER);


    }
    public Image getframe(){
        VuforiaLocalizer.CloseableFrame frame = null;
        try{
            frame = vuforia.getFrameQueue().take();
            long numImages = frame.getNumImages();
            Image rgbImage = null;
            for (int i = 0; i < numImages; i++) {
                Image img = frame.getImage(i);
                int fmt = img.getFormat();
                if (fmt == PIXEL_FORMAT.RGB565) {
                    rgbImage = frame.getImage(i);
                    break;
                }
            }
            return rgbImage;
        }
        catch(InterruptedException exc){
            return null;
        }
        finally{
            if (frame != null) frame.close();
        }

    }


    @Override public void runOpMode() {

        Arm = hardwareMap.servo.get("JewelServo");
        Arm.setPosition(0.38);

        Motor1 = hardwareMap.dcMotor.get("Motor1");
        Motor2 = hardwareMap.dcMotor.get("Motor2");
        Motor3 = hardwareMap.dcMotor.get("Motor3");
        Motor4 = hardwareMap.dcMotor.get("Motor4");
        GlyphFlip = hardwareMap.dcMotor.get("GlyphFlip");




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


        // Set all motors to zero power
        Motor1.setPower(0);
        Motor2.setPower(0);
        Motor3.setPower(0);
        Motor4.setPower(0);

        /* Reference to color sensor object */
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        /*
         * To start
         * up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();


        parameters.vuforiaLicenseKey = "Aegsx6b/////AAAAGZ1XCL5uwk7gp+PMLDLPoOcm5/yyHm4ex0tWMj1G+87mVQHnJ6oK2EdHHthatiYRvKuuvmegcsrLkbjEL7IzSGCh9pjtiavsoCBwMcB1rtOyjwv1X+Veys1noJNxEZF8W7tSXyWDvigaqNmj8y/fIQ+Q03SkEXlytTqMTHgSpcs8l1qbd4o22QrfCik+i/YYrpdOPU82yNY54jmdfPX5r8gEt1zboWugVcwewkh7TL8f00CDz4TgvBXdqZN4k76GLdwxKhXIe9ThEGS/ghb/yyYoXCmZwX6MZN62V3BcAjiIowbZDkUtlozp2eiAJl/7O4/WXfiKhl+g7bMlFT99ID7m7wWZYmSX/7A4zJsVpE+Q";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * irn this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();


        double startang = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        relicTrackables.activate();

        while(opModeIsActive()){
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);


            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            Arm.setPosition(0.95);
            sleep(2000);
            double red = colorSensor.red();
            double blue = colorSensor.blue();


            if (blue - red < 0) {
                telemetry.addLine("RED!");
                telemetry.update();
                gyroturn(11, 0.18, 1);
                sleep(400);
                Arm.setPosition(0.38);
                gyroturn(-11, 0.18, 1);
            } else if (blue - red > 0) {
                telemetry.addLine("BLUE!");
                telemetry.update();
                gyroturn(-11, 0.18, 1);
                sleep(400);
                Arm.setPosition(0.38);
                gyroturn(11, 0.18, 1);
            }
            sleep(2000);





                /* Found an instance of the template.
                   on which VuMark was visible. */

            telemetry.addData("VuMark", "%s visible", vuMark);
            telemetry.update();

                if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    driveForward(-2450,- 0.3);
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    driveForward(-3050, -0.3);
                } else if (vuMark == RelicRecoveryVuMark.LEFT) {
                    driveForward(-4000, -0.3);
                } else if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
                    driveForward(-100,-0.2);
                    vuMark = RelicRecoveryVuMark.from(relicTemplate);
                    if (vuMark == RelicRecoveryVuMark.RIGHT) {
                        driveForward(-2350, -0.3);
                    } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                        driveForward(-2950, -0.3);
                    } else if (vuMark == RelicRecoveryVuMark.LEFT) {
                        driveForward(-3900, -0.3);
                    } else if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
                        driveForward(-3900,-0.2);
                        vuMark = RelicRecoveryVuMark.from(relicTemplate);


                    }

                }
                telemetry.addData("VuMark", "%s visible", vuMark);
                telemetry.update();
                gyroturn(-88, 0.6, 1.265);
                driveForward(400, 0.2);
                Flip(130,0.5);
                driveForward(50,0.2);
                driveForward(-300,-0.2);
                driveForward(200,0.2);
                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));
                stop();




        }
    }








    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}