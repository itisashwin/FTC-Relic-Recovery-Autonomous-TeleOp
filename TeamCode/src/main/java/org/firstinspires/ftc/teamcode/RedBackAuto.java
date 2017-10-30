package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Shreyas on 10/6/17.
 */
@Autonomous
public class RedBackAuto extends LinearOpMode {
    VuforiaLocalizer vuforia;

    /* Public Op-Mode Members. */
    private DcMotor Motor1;
    private DcMotor Motor2;
    private DcMotor Motor3;
    private DcMotor Motor4;
    private Servo Arm;
    public final static double ARM_HOME = 0.0;
    public final static double ARM_MAX_RANGE = 0.50;
    private ColorSensor colorSensor;

    /* Local OpMode members. */

    /*Constructor*/
    public RedBackAuto() {
    }

    /*Methods*/
    public void setMotors (double speed) {
        Motor1.setPower(speed);
        Motor2.setPower(speed);
        Motor3.setPower(speed);
        Motor4.setPower(speed);
    }

    public void checkVuMark (RelicRecoveryVuMark vuMark) {
        if (vuMark == RelicRecoveryVuMark.LEFT) {
                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
            telemetry.addData("VuMark", "%s visible: Left", vuMark);
            return;
        } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
            telemetry.addData("VuMark", "%s visible: Right", vuMark);
            return;
        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            telemetry.addData("VuMark", "%s visible: Center", vuMark);
            return;
        } else {
            telemetry.addData("VuMark", "not visible");
            VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
            VuforiaTrackable relicTemplate = relicTrackables.get(0);
            relicTemplate.setName("relicVuMarkTemplate");
            RelicRecoveryVuMark vuMark1 = RelicRecoveryVuMark.from(relicTemplate);
            checkVuMark(vuMark1);
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {

        Arm = hardwareMap.servo.get("JewelServo");
        Arm.setPosition(ARM_HOME);

        Motor1 = hardwareMap.dcMotor.get("Motor1");
        Motor2 = hardwareMap.dcMotor.get("Motor2");
        Motor3 = hardwareMap.dcMotor.get("Motor3");
        Motor4 = hardwareMap.dcMotor.get("Motor4");

        // Set all motors to zero power
        Motor1.setPower(0);
        Motor2.setPower(0);
        Motor3.setPower(0);
        Motor4.setPower(0);

        /* Reference to color sensor object */
        //colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        /* Initialize VuMark ID */
/*
        telemetry.addData(">", "before vuMark init");
        sleep(500);
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        telemetry.addData(">", "before setting vuforialocalizer");
        sleep(500);
        telemetry.update();
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "Aegsx6b/////AAAAGZ1XCL5uwk7gp+PMLDLPoOcm5/yyHm4ex0tWMj1G+87mVQHnJ6oK2EdHHthatiYRvKuuvmegcsrLkbjEL7IzSGCh9pjtiavsoCBwMcB1rtOyjwv1X+Veys1noJNxEZF8W7tSXyWDvigaqNmj8y/fIQ+Q03SkEXlytTqMTHgSpcs8l1qbd4o22QrfCik+i/YYrpdOPU82yNY54jmdfPX5r8gEt1zboWugVcwewkh7TL8f00CDz4TgvBXdqZN4k76GLdwxKhXIe9ThEGS/ghb/yyYoXCmZwX6MZN62V3BcAjiIowbZDkUtlozp2eiAJl/7O4/WXfiKhl+g7bMlFT99ID7m7wWZYmSX/7A4zJsVpE+Q";

        telemetry.addData(">", "after license key");
        sleep(500);
        telemetry.update();

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        telemetry.addData(">", "after setting which camera");
        sleep(500);
        telemetry.update();
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        telemetry.addData(">", "after trackables");
        sleep(500);
        telemetry.update();
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        telemetry.addData(">", "after relic template instance");
        sleep(500);
        telemetry.update();
        relicTemplate.setName("relicVuMarkTemplateMemez"); // can help in debugging; otherwise not necessary
        telemetry.addData(">", "after setting name");
        sleep(500);
        telemetry.update();

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        */
        waitForStart();

        /*  S h o w  T i m e  */
        //relicTrackables.activate();
        while (opModeIsActive()) {

            //RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            //setMotors(0.5);
            //while (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            //checkVuMark(vuMark);
            //}
            //while (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            // telemetry.addData("VuMark", "%s visible: Unknown", vuMark);
            // }

                Arm.setPosition(ARM_MAX_RANGE);
                //double red = colorSensor.red();
                //double blue = colorSensor.blue();
                if (Math.random()*100<50) {
                    telemetry.addLine("option1");
                    telemetry.update();
                    setMotors(0.8);
                    sleep(500);
                    setMotors(-0.8);
                    sleep(500);
                } else if (Math.random()*100<=50) {
                    telemetry.addLine("option2");
                    telemetry.update();
                    setMotors(-0.8);
                    sleep(500);
                    setMotors(0.8);
                    sleep(500);
                }
                Arm.setPosition(ARM_HOME);
                setMotors(1);
                sleep(2000);
            /*if (vuMark == RelicRecoveryVuMark.LEFT) {
                //setMotors(0.5);
            } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                //setMotors(1);
            } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                //setMotors(0.25);*/
            }
        }
    }
