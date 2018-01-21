package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.classes.AdafruitIMU;
import org.firstinspires.ftc.teamcode.classes.Mecanum;

@Autonomous(name="Blue Auto 2", group="Pushbot")
public class BlueAuto2 extends LinearOpMode {

    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBR;
    private DcMotor motorBL;

    //Servo
    private Servo armServoTop;
    private Servo armServoBot;
    private Servo armServoRot;

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
    ElapsedTime timer;

    private static final Double ticks_per_inch = 19.9;
    private static final Double CORRECTION = .04;
    private static final Double THRESHOLD = 2.0;

    Double driveDistance;
    int red;
    int blue;

    double topPosOpen = 1.0;    //Open for arm
    double botPosOpen = 1.0;

    double topPosClose = 0.0;
    double botPosClose = 0.0;   //Close for arm

    double rotPos = 0.0;

    public void runOpMode() {
        //motors
        motorFL = hardwareMap.dcMotor.get("fl");
        motorFR = hardwareMap.dcMotor.get("fr");
        motorBL = hardwareMap.dcMotor.get("bl");
        motorBR = hardwareMap.dcMotor.get("br");

        //Servo
        armServoTop = hardwareMap.servo.get("arm_servoT");
        armServoTop.setPosition(topPosClose);

        armServoBot = hardwareMap.servo.get("arm_servoB");
        armServoBot.setPosition(botPosClose);

        armServoRot = hardwareMap.servo.get("arm_servoR");
        armServoRot.setPosition(rotPos);

        //Camera setup
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AXrSE4L/////AAAAmRrcbhbRtktYuoFNH6SYXsg3DAoskyFpeMJmWumuwvdJQ8vU6duKJ8TX2fFqU/SmaMtFGSxY/CaiRHVIS2CMcInOkmDXgoglSTo7lB8m1V5gUkaPwHLS6PGnyG6JECNotb/ait+fmG1SkkZD3+588MjDUOWRV+E3xG3LB1rqyjM+yO/jjgYpfTNoxGFHhbmjE0qxD/fiftVDdewEcntlTeTPCml9f5AUv0+TRhS4zILyI8J3OKwtfjGG7Cx2A8RiosLq6TsPh6okqZKF3YLOSqiPyMDeHCE4FxFeam4WVHccHTkPmMG7FrgxZOYNwI9eDlrC83qdNMzkjpSqTVfF2H9CNE2wvzl07zfXFgV6PRVI";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        //Vuforia
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary


        //Mecanum
        bot = new Mecanum(motorFR, motorFL, motorBR, motorBL);

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
        timer  = new ElapsedTime();

        driveDistance = 15.0;
        blue = 0;
        red = 0;

        waitForStart();

        jewelHitter2.setPosition(0.0);
        pauseAuto(1.0);

//////////////////////////////////////////////////////////////////////////play!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        imu.start();

        //STATE TWO: DETECT BALLS
        hitballOff();

        //STATE THREE: SCAN VUMARK
       // encoderDrive(0.5, "forward", .2);

        pauseAuto(1.0);

        relicTrackables.activate();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        telemetry.addData("VuMark", "%s visible", vuMark);

        telemetry.update();
        if (vuMark == RelicRecoveryVuMark.LEFT) {
            driveDistance = 2.0;
        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            driveDistance = 12.0;
        } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
            driveDistance = 24.0;
        } else {
            driveDistance = 19.0;
        }
        pauseAuto(2.0);

        //STATE FOUR: MOVE BACK
        encoderDrive(35.0, "backward", .2);
        pauseAuto(1.0);
        //STATE: TURN 180
        gyroTurnLeft(90,"oof" , .27);
        pauseAuto(1.0);

        //STATE FIVE: MOVE RIGHT
        encoderDrive(driveDistance, "forward", .27);
        pauseAuto(1.0);

        //STATE: TURN 90
        gyroTurnLeft(90,"oof" , .27);

        encoderDrive(10.0,"forward" , .27);

        armServoBot.setPosition(botPosOpen);
        armServoTop.setPosition(topPosOpen);

        encoderDrive(3.0,"forward" , .5);

        jewelHitter.setPosition(0.0);

    }

    public void encoderDrive(double inches, String direction, double power) {
        int encoderval;
        //
        // Sets the encoders
        //
        bot.reset_encoders();
        encoderval = ticks_per_inch.intValue() * (int) inches;

        //
        // Uses the encoders and motors to set the specific position
        //
        bot.setPosition(encoderval, encoderval, encoderval, encoderval);
        bot.run_using_encoders();
        //
        // Sets the power and direction
        //
        bot.setPowerD(power);
        if (direction == "forward") {
            bot.run_forward();
        } else if (direction == "backward") {
            bot.run_backward();
        } else if (direction == "left") {
            bot.run_left();
        } else if (direction == "right") {
            bot.run_right();
        } else if (direction == "diagonal_left_up") {
            bot.run_diagonal_left_up();
        }

        while (bot.testDistance(motorFL) != 1 && opModeIsActive()) {
            telemetry.addData("Pos ", motorFL.getCurrentPosition());
            telemetry.update();
        }

        bot.brake();


    }


    public void gyroTurnRight(double angle, String direction, double power) {
        double aheading = imu.getHeading() - angle;
        if (direction == "og") {
            aheading = 180;
        }
        boolean gua = false;
        bot.run_without_encoders();
        bot.setPowerD(power);

        while (opModeIsActive() && gua == false) {
            //aheading = Math.abs(imu.getHeading()) + angle;
            bot.turn_right();

            telemetry.addData("Heading", imu.getHeading());
            telemetry.addData("Target Angle", aheading);
            telemetry.update();
            if (imu.getHeading() >= (aheading - THRESHOLD) && (imu.getHeading() <= (aheading + THRESHOLD))) {
                bot.brake();
                gua = true;
            }

        }

        bot.brake();

    }

    public void gyroTurnLeft(double angle, String direction, double power) {
        double aheading = imu.getHeading() + angle;
        boolean gua = false;
        bot.run_without_encoders();
        bot.setPowerD(power);

        if (direction == "og") {
            aheading = 180;
        }

        while (opModeIsActive() && gua == false) {
            //aheading = Math.abs(imu.getHeading()) + angle;
            bot.turn_left();

            telemetry.addData("Heading", imu.getHeading());
            telemetry.addData("Target Angle", aheading);
            telemetry.update();
            if (imu.getHeading() >= (aheading - THRESHOLD) && (imu.getHeading() <= (aheading + THRESHOLD))) {
                bot.brake();
                gua = true;
            }

        }

        bot.brake();

    }

    public void pauseAuto(double time) {
        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        while (timer.seconds() < time && opModeIsActive()) {

        }
        timer.reset();
    }

    public void hitballOff() {
        gyroTurnRight(5, "oof", .3);

        jewelHitter.setPosition(.75);

        pauseAuto(1.0);

        timer.reset();
        timer.startTime();
        while (timer.seconds() < 1 && opModeIsActive()) {
            red += sensorColor.red();
            blue += sensorColor.blue();
        }

        telemetry.addData("Red", red);
        telemetry.addData("Blue", blue);
        telemetry.update();

        if (blue > red) {

            gyroTurnLeft(10, "oof", .3);
            jewelHitter.setPosition(0.0);
            pauseAuto(.5);
            gyroTurnRight(5, "oof", .26);
        } else {

            gyroTurnRight(10, "oof", .3);
            jewelHitter.setPosition(0.0);
            pauseAuto(.5);
            gyroTurnLeft(15, "oof", .26);
        }


        telemetry.update();
    }
}