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
@Autonomous(name="Red Auto 2", group="Pushbot")
public class RedAuto2 extends LinearOpMode {

    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBR;
    private DcMotor motorBL;

    private Servo armServoL1;
    private Servo armServoL2;
    private Servo armServoR1;
    private Servo armServoR2;

    double ServoposL = 1;
    double ServoposL1 = 1;
    double ServoposR = .7;
    double ServoposR1 = .7;

    double closeposL1 = 0.29;
    double closeposR1 = 0.28;
    double closeposL2 = 0.79;
    double closeposR2 = 0.22;

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
    Servo jewelHitter2;

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

        //Servo
        armServoL1 = hardwareMap.servo.get("arm_servoL1");
        armServoL2 = hardwareMap.servo.get("arm_servoL2");
        armServoR2 = hardwareMap.servo.get("arm_servoR2");
        armServoR1 = hardwareMap.servo.get("arm_servoR1");

        armServoL1.setPosition(.05);
        armServoL2.setPosition(.45);    //actually r2
        armServoR1.setPosition(.95);
        armServoR2.setPosition(.05);

        //Camera setup
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AXrSE4L/////AAAAmRrcbhbRtktYuoFNH6SYXsg3DAoskyFpeMJmWumuwvdJQ8vU6duKJ8TX2fFqU/SmaMtFGSxY/CaiRHVIS2CMcInOkmDXgoglSTo7lB8m1V5gUkaPwHLS6PGnyG6JECNotb/ait+fmG1SkkZD3+588MjDUOWRV+E3xG3LB1rqyjM+yO/jjgYpfTNoxGFHhbmjE0qxD/fiftVDdewEcntlTeTPCml9f5AUv0+TRhS4zILyI8J3OKwtfjGG7Cx2A8RiosLq6TsPh6okqZKF3YLOSqiPyMDeHCE4FxFeam4WVHccHTkPmMG7FrgxZOYNwI9eDlrC83qdNMzkjpSqTVfF2H9CNE2wvzl07zfXFgV6PRVI";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        //Vuforia
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        //Mecanum
        bot = new Mecanum(motorFR,motorFL,motorBR,motorBL);

        //IMU
        imu = new AdafruitIMU(hardwareMap.get(BNO055IMU.class, "imu"));
        imu.init();

        //Color Sensor
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        //Servo
        jewelHitter = hardwareMap.servo.get("servo_hitter");
        jewelHitter.setPosition(0);

        jewelHitter2 = hardwareMap.servo.get("servo_hitter2");

        //Timer
        timer = new ElapsedTime();

        driveDistance = 15.0;

        waitForStart();

//////////////////////////////////////////////////////////////////////////play!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        imu.start();

        jewelHitter2.setPosition(0.0);
        pauseAuto(1.0);
        //STATE ONE: MOVE FORWARD
        hitballOff();

        pauseAuto(2.0);

        //STATE THREE: SCAN VUMARK
        encoderDrive(28.0,"forward" , .3);
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        telemetry.addData("VuMark", "%s visible", vuMark);

        telemetry.update();
        pauseAuto(1.0);

        relicTrackables.activate();
        if (vuMark == RelicRecoveryVuMark.LEFT){
            driveDistance = 24.0;
        }else if (vuMark == RelicRecoveryVuMark.CENTER){
            driveDistance = 16.0;
        }else if (vuMark == RelicRecoveryVuMark.RIGHT){
            driveDistance = 13.0;
        }else{
            driveDistance = 23.0;
        }
        pauseAuto(1.0);
        //STATE FIVE: MOVE BACK
        gyroTurnLeft(89,"oof" , .27);
        pauseAuto(1.0);
        encoderDrive(driveDistance,"forward",.3);
        pauseAuto(1.0);
        gyroTurnRight(90,"oof" , .27);



        //STATE SIX: STACK BLOCK
        encoderDrive(5.0,"forward" , .27);


        armServoL1.setPosition(.15+.2);
        armServoL2.setPosition(.4-.2);
        armServoR2.setPosition(.9-.2);
        armServoR1.setPosition(.15+.2);

        encoderDrive(3.0,"backward" , .3);
        pauseAuto(1.0);

        encoderDrive(5.0,"forward" , .5);


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
        if (direction == "forward"|| direction =="forward_scacn"){
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
            if(direction =="forward_scan"){

            }
        }

        bot.brake();


    }



    public void gyroTurnRight(double angle, String direction, double power){
        double aheading = imu.getHeading() - angle;
        if(direction=="og"){
            aheading = 180;
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
        double aheading = imu.getHeading() + angle;
        boolean gua = false;
        bot.run_without_encoders();
        bot.setPowerD(power);

        if(direction=="og"){
            aheading = 180;
        }

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
        gyroTurnRight(5,"fatfuck",.3);

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

        if(blue<red){

            gyroTurnLeft(10,"oof",.3);
            jewelHitter.setPosition(0.0);
            pauseAuto(.5);
            gyroTurnRight(5,"oof",.26);
        }else{

            gyroTurnRight(10,"oof",.3);
            jewelHitter.setPosition(0.0);
            pauseAuto(.5);
            gyroTurnLeft(15,"oof",.26);
        }


        telemetry.update();
    }

}
