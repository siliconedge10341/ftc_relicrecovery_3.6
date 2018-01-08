package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
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

import org.firstinspires.ftc.teamcode.classes.AdafruitIMU;
import org.firstinspires.ftc.teamcode.classes.Mecanum;


/**
 * Created by vatty on 9/15/2017.
 */
@Autonomous(name="Red Auto 1", group="Pushbot")
public class RedAuto1 extends LinearOpMode {

    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBR;
    private DcMotor motorBL;

    //Mecanum
    Mecanum bot = new Mecanum();

    //Camera initialize
    VuforiaLocalizer vuforia;

    //Gyro Initialize
    AdafruitIMU imu = new AdafruitIMU();

    //Color sensor
    ColorSensor sensorColor;

    //Servos
    Servo jewelHitter;

    //Timer
    ElapsedTime timer = new ElapsedTime();

    private static final Double ticks_per_inch = 19.9;
    private static final Double CORRECTION = .04;
    private static final Double THRESHOLD = 2.0;
    Double driveDistance;
    double red;
    double blue;

    public void runOpMode(){
        //motors
        motorFL = hardwareMap.dcMotor.get("fl");
        motorFR = hardwareMap.dcMotor.get("fr");
        motorBL = hardwareMap.dcMotor.get("bl");
        motorBR = hardwareMap.dcMotor.get("br");

        //Camera setup
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AQXLr5v/////AAAAGXJCbi1ut0+SmeEkH1vkZG0NpkUylv0BWeZ4GaNc7LTEne9mZKpKMgWbCuVD61ge5I21IinkJ2L4JTKau7uw5jlPtkvH/PVDQ2EYv9UGJ6d0ml/iI2pWhuv4wDNbuOwWtB3/kuepar8zRVCOI0Ec05z766KFRLyb6ldCuMdQ04hQOn/02RYZRv43IBcMhOJiY3gs0oEiTwb+I4yVBa7qp8bQTGEjdysybOicD9JnswAzF4i0qSVh9WRGbkY8rFlkA+THheK72syEnD9iVCfXjgKaPuUZ95XpR4V7eUl/LnSm3uQ/FrXjRkyYU0TsaWROhRrdpMNSZSt17A/RDL7lE0K6iil9wMEo/7UT/jO5cqOn";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        //Vuforia
        VuforiaTrackables relicTrackables =  this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        //Mecanum
        bot = new Mecanum(motorFR,motorFL,motorBR,motorBL);

        //IMU
        imu = new AdafruitIMU(hardwareMap.get(BNO055IMU.class, "imu"));

        //Color Sensor
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        //Servo
        jewelHitter = hardwareMap.servo.get("servo_hitter");
        jewelHitter.setPosition(0.5);

        //Timer
        timer = new ElapsedTime();

        driveDistance = 15.0;


        waitForStart();

//////////////////////////////////////////////////////////////////////////play!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        imu.start();
        //STATE ONE: MOVE FORWARD
        //encoderDrive(2,"forward",.5);

        //STATE TWO: DETECT BALLS
        hitballOff();

        //STATE THREE: SCAN VUMARK
        //encoderDrive(2.0,"left",.3);

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        telemetry.addData("VuMark", "%s visible", vuMark);

        telemetry.update();
        if (vuMark == RelicRecoveryVuMark.LEFT){
            driveDistance = 46.0;
        }else if (vuMark == RelicRecoveryVuMark.CENTER){
            driveDistance = 34.0;
        }else if (vuMark == RelicRecoveryVuMark.RIGHT){
            driveDistance = 22.0;
        }else{
            driveDistance = 36.0;
        }

        //STATE FIVE: MOVE LEFT
        encoderDrive(driveDistance,"forward",.3);

        //STATE SIX: TURN 90 degrees
        gyroTurnRight(90,"oof",0.3);
    }

    public void encoderDrive(double inches, String direction , double power ) {
        int encoderval;
        //
        // Sets the encoders
        //
        bot.reset_encoders();
        encoderval = ticks_per_inch.intValue() * (int) inches;

        //
        // Uses the encoders and motors to set the specific position
        //
        bot.setPosition(encoderval,encoderval,encoderval,encoderval);
        bot.run_using_encoders();
        //
        // Sets the power and direction
        //
        bot.setPowerD(power);
        if (direction == "forward"){
            bot.run_forward();
        } else if(direction == "backward"){
            bot.run_backward();
        } else if (direction == "left"){
            bot.run_left();
        } else if (direction == "right"){
            bot.run_right();
        } else if (direction == "diagonal_left_up"){
            bot.run_diagonal_left_up();
        }

        while (bot.testDistance(motorFL) != 1 && opModeIsActive()) {
            telemetry.addData("Pos ", motorFL.getCurrentPosition());
            telemetry.update();
        }

        bot.brake();


    }


    public void gyroTurnRight(double angle, String direction, double power){
        double aheading = imu.getHeading() + angle;
        if(aheading>360){
            aheading=aheading-360;
        }
        boolean gua = false;
        bot.run_without_encoders();
        bot.setPowerD(power);

        while(opModeIsActive() && gua==false) {
            //aheading = Math.abs(imu.getHeading()) + angle;
            bot.turn_right();

            telemetry.addData("Heading", imu.getHeading());
            telemetry.addData("Target Angle", aheading);
            telemetry.update();
            if (imu.getHeading() >= (aheading - THRESHOLD) && (imu.getHeading() <= (aheading + THRESHOLD))) {
                bot.brake();
                gua=true;
            }

        }

        bot.brake();

    }

    public void gyroTurnLeft(double angle, String direction, double power){
        double aheading = imu.getHeading() - angle;
        boolean gua = false;
        bot.run_without_encoders();
        bot.setPowerD(power);

        while(opModeIsActive() && gua==false) {
            //aheading = Math.abs(imu.getHeading()) + angle;
            bot.turn_left();

            telemetry.addData("Heading", imu.getHeading());
            telemetry.addData("Target Angle", aheading);
            telemetry.update();
            if (imu.getHeading() >= (aheading - THRESHOLD) && (imu.getHeading() <= (aheading + THRESHOLD))) {
                bot.brake();
                gua=true;
            }

        }

        bot.brake();

    }

    public void pauseAuto(double time){
        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        while(timer.seconds()<time && opModeIsActive()){

        }
        timer.reset();
    }


    public void hitballOff(){
        gyroTurnRight(9,"oof",.3);

        jewelHitter.setPosition(.75);

        pauseAuto(1.0);

        timer.reset();
        timer.startTime();
        while(timer.seconds()<1 && opModeIsActive()){
            red += sensorColor.red();
            blue += sensorColor.blue();
        }

        telemetry.addData("Red" , red);
        telemetry.addData("Blue" , blue);
        telemetry.update();

        if(blue>red){

            gyroTurnLeft(10,"oof",.3);
            jewelHitter.setPosition(0.0);
            pauseAuto(.5);
            gyroTurnRight(180,"og",.3);
        }else{

            gyroTurnRight(10,"oof",.3);
            jewelHitter.setPosition(0.0);
            pauseAuto(.5);
            gyroTurnLeft(180,"og",.3);
        }


        telemetry.update();
    }
}
