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


@TeleOp(name = "MecanumDriveGood", group = "Drive")
public class driver extends OpMode{
    // instance variables
    // private variables
    // Motors

    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBR;
    private DcMotor motorBL;

    private DcMotor motorVertical1;

    private DcMotor motorHorizontal;

    private Servo armServoTop;
    private Servo armServoBot;
    private Servo armServoRot;

    private Servo relicservohook;
    private Servo relicservoarm;

    //increasing servo value -> clockwise

    double openposLB = .1;
    double openposLT = .1;
    double openposRB = .9;
    double openposRT = .9;
    double openrelichook = .9;
    double downrelicarm = 1.0;

    double closeposLB = 0.65;
    double closeposLT = 0.9;
    double closeposRB = 0.35;
    double closeposRT = .1;
    double closerelichook = 0.0;
    double uprelicarm = .4;

    double topPosOpen = 1.0;
    double botPosOpen = 1.0;

    double topPosClose = 0.0;
    double botPosClose = 0.0;

    double rotPos = 0.0;

    //AdafruitIMU imu;

    double Ch1;
    double Ch3;
    double Ch4 ;
    double accel;
    int endtime = 0;
    boolean pressed;

    double speedcoef;

    boolean changedRot = false; //Outside of loop()
    boolean changed = false; //Outside of loop()
    boolean changedup = false, on = false; //Outside of loop()

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

        motorHorizontal = hardwareMap.dcMotor.get("motor_horizontal");

        armServoTop = hardwareMap.servo.get("arm_servoT");
        armServoTop.setPosition(topPosOpen);

        armServoBot = hardwareMap.servo.get("arm_servoB");
        armServoBot.setPosition(botPosOpen);

        armServoRot = hardwareMap.servo.get("arm_servoR");
        armServoRot.setPosition(rotPos);


        relicservohook = hardwareMap.servo.get("relic_servohook");
        relicservohook.setPosition(1.0);

        relicservoarm = hardwareMap.servo.get("relic_servoarm");
        relicservoarm.setPosition(uprelicarm);


        // imu = new AdafruitIMU(hardwareMap.get(BNO055IMU.class, "imu"));
        // imu.init();

        pressed = false;
        endtime = 0;
        speedcoef = .7;

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


        if (gamepad1.left_trigger>.5){
            speedcoef = 1.0;
        }else if(gamepad1.right_trigger >.5){
            speedcoef = .3;
        }
        else{
            speedcoef = .7;
        }

        //meccanum drive
        Ch1 = -gamepad1.right_stick_x;
        Ch3 = gamepad1.left_stick_y;
        Ch4 = gamepad1.left_stick_x;

        motorFR.setPower( -speedcoef* -(Ch3 - Ch1 - Ch4));
        motorFL.setPower( -speedcoef * (Ch3 + Ch1 + Ch4));

        motorBR.setPower(-speedcoef * -(Ch3 - Ch1 + Ch4));
        motorBL.setPower(-speedcoef * (Ch3 + Ch1 - Ch4));

        //Glyph open/close
        if(gamepad1.right_trigger>.5){ 		//Glyph Open
            armServoBot.setPosition(botPosOpen);
            armServoTop.setPosition(topPosOpen);
        }else if(gamepad1.right_bumper){ 	//Glyph Close
            armServoBot.setPosition(topPosClose);
            armServoTop.setPosition(botPosClose);
        }

        //Glpy mechanism rotate

        if(gamepad2.x && !changedRot) {
            if(armServoRot.getPosition() == 0) armServoRot.setPosition(1);
            else armServoRot.setPosition(0);
            changedRot = true;
        } else if(!gamepad1.x) changedRot = false;

        //relic open/close

        if(gamepad2.right_trigger>.5){ 		//Relic Close
            relicservohook.setPosition(closerelichook);
        }else if(gamepad2.left_trigger>.5){ 		//Relic Open
            relicservohook.setPosition(openrelichook);
        }

        //relic up/down



        if(gamepad2.right_bumper && !changedup) {
            if(relicservoarm.getPosition() == uprelicarm) relicservoarm.setPosition(downrelicarm);
            else relicservoarm.setPosition(uprelicarm);
            changedup = true;
        } else if(!gamepad2.right_bumper) changedup = false;

       /* if(gamepad2.left_bumper){ 		//Relic arm Up
            relicservoarm.setPosition(uprelicarm);
        }else if(gamepad2.right_bumper){ 		//Relic arm Down
            relicservoarm.setPosition(downrelicarm);
        }*/

        //glyph extend
        if(gamepad2.dpad_up){			//up
            motorVertical1.setPower(1.0);
        }else if(gamepad2.dpad_down){		//down
            motorVertical1.setPower(-1.0);
        }else{					//default
            motorVertical1.setPower(0.0);
        }

        //relic extend
        if(gamepad2.dpad_left){
            motorHorizontal.setPower(.7);
        }else if(gamepad2.dpad_right){
            motorHorizontal.setPower(-.7);
        }else{
            motorHorizontal.setPower(0.0);
        }

        //telemetry
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

    public void flip(){

    }

}

