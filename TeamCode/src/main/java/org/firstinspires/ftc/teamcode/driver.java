package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.classes.AdafruitIMU;


@TeleOp(name = "MecanumDrive", group = "Drive")
public class driver extends OpMode{
    // instance variables
    // private variables
    // Motors

    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBR;
    private DcMotor motorBL;

    private DcMotor motorVertical1;
    private DcMotor motorVertical2;

    private DcMotor motorHorizontal;


    private Servo armServoL1;
    private Servo armServoL2;
    private Servo armServoR1;
    private Servo armServoR2;

    double ServoposL = 1;
    double ServoposL1 = 1;
    double ServoposR = .7;
    double ServoposR1 = .7;

    //AdafruitIMU imu;

    double Ch1;
    double Ch3;
    double Ch4 ;
    double accel;
    double speedv = 2;
    int endtime = 0;
    boolean pressed;

    double speedcoef;

    double closeposL1 = 0.29;
    double closeposR1 = 0.28;
    double closeposL2 = 0.79;
    double closeposR2 = 0.22;

    // Servos

    // constructors
    public driver() {
        // default constructor

    }

    @Override
    public void init() {
        //
        // Initialize everything
        //
        // Motors
        motorFL = hardwareMap.dcMotor.get("fl");
        motorFR = hardwareMap.dcMotor.get("fr");
        motorBL = hardwareMap.dcMotor.get("bl");
        motorBR = hardwareMap.dcMotor.get("br");



        motorVertical1 = hardwareMap.dcMotor.get("motor_vertical1");
        motorVertical1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorVertical2 = hardwareMap.dcMotor.get("motor_vertical2");
        motorVertical2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorHorizontal = hardwareMap.dcMotor.get("motor_horizontal");

        armServoL1 = hardwareMap.servo.get("arm_servoL1");
        armServoL1.setPosition(closeposL1);

        armServoL2 = hardwareMap.servo.get("arm_servoL2");
        armServoL2.setPosition(closeposL2);

        armServoR2 = hardwareMap.servo.get("arm_servoR2");
        armServoR2.setPosition(closeposR1);

        armServoR1 = hardwareMap.servo.get("arm_servoR1");
        armServoR1.setPosition(closeposR2);

        armServoL1.setPosition(.15);
        armServoL2.setPosition(.4);
        armServoR1.setPosition(.9);
        armServoR2.setPosition(.15);

       // imu = new AdafruitIMU(hardwareMap.get(BNO055IMU.class, "imu"));
       // imu.init();

        pressed = false;
        endtime = 0;
        speedcoef = .5;

        accel = 0;

    }

    @Override
    public void start() {
        //imu.start();`
    }

    // loop
    @Override
    public void loop() {
        //accel = Math.sqrt(imu.getAccelX()*imu.getAccelX() + imu.getAccelZ()*imu.getAccelZ() + imu.getAccelY()*imu.getAccelY());


        if (gamepad1.right_trigger>.5){
            speedv = 2;                   //fast
        }
        else{
            speedv = 1;
        }

        Ch1 = gamepad1.right_stick_x;
        Ch3 = gamepad1.left_stick_y;
        Ch4 = gamepad1.left_stick_x;

        motorFR.setPower( -speedcoef* -(Ch3 - Ch1 - Ch4));
        motorFL.setPower( -speedcoef * (Ch3 + Ch1 + Ch4));


        motorBR.setPower(-speedcoef * -(Ch3 - Ch1 + Ch4));
        motorBL.setPower(-speedcoef * (Ch3 + Ch1 - Ch4));

        if(gamepad2.left_bumper){
            armServoL1.setPosition(.15+.2);
            armServoL2.setPosition(.4-.2);
            armServoR2.setPosition(.9-.2);
            armServoR1.setPosition(.15+.2);
        }else if(gamepad2.right_bumper){
            armServoL1.setPosition(.05);
            armServoL2.setPosition(.45);    //actually r2
            armServoR1.setPosition(.95);
            armServoR2.setPosition(.05);;
        }else if(gamepad2.right_trigger>.5){
            armServoL1.setPosition(.15-.08);
            armServoL2.setPosition(.4+.08);    //actually r2
            armServoR1.setPosition(.9+.08);
            armServoR2.setPosition(.15-.08);
        }


        if (speedv == 1){
            speedcoef = .6;
        }
        if (speedv == 2){
            speedcoef = .5;
        }

        if(gamepad2.dpad_up){
            motorVertical1.setPower(1.0);
            motorVertical2.setPower(-1.0);
        }else if(gamepad2.dpad_down){
            motorVertical1.setPower(-1.0);
            motorVertical2.setPower(1.0);
        }else{
            motorVertical1.setPower(0.0);
            motorVertical2.setPower(0.0);
        }

        if(gamepad1.dpad_left){
            motorHorizontal.setPower(1.0);
        }else if(gamepad1.dpad_right){
            motorHorizontal.setPower(-1.0);
        }else{
            motorHorizontal.setPower(0.0);
        }

        telemetry.addData("Servopos Left:" , ServoposL);
        telemetry.addData("Servopos Right: " , ServoposR);
        telemetry.addData("Speed coeff" , speedcoef);
        //telemetry.addData("Acceleration" , accel);
        telemetry.update();

        // Runs the collector
        //



    }

    // functions
    @Override
    public void stop() {

        // set to zero so the power doesn't influence any motion or rotation in the robot

    }

}

