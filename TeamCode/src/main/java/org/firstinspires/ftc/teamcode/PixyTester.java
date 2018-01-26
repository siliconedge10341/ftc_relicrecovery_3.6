package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.apache.commons.math3.stat.descriptive.DescriptiveStatistics;


@Autonomous(name="PixtTester")
public class PixyTester extends LinearOpMode {
    I2cDeviceSynch pixyCam;
    Servo servo_tilt;

    double x, y, width, height, numObjects;
    double gainx = -.0005;
    double xpos = 0.5;

    byte[] pixyData;

    DescriptiveStatistics stat;



    @Override
    public void runOpMode() throws InterruptedException {

        pixyCam = hardwareMap.i2cDeviceSynch.get("pixy");

        servo_tilt = hardwareMap.servo.get("servo_pan");
        servo_tilt.setPosition(.14);

        stat=new DescriptiveStatistics();


        waitForStart();

        while(opModeIsActive()){
            pixyCam.engage();
            pixyData = pixyCam.read(0x51, 5);

            x = pixyData[1];
            y = pixyData[2];
            width = pixyData[3];
            height = pixyData[4];
            numObjects = pixyData[0];

            servo_tilt.setPosition(xpos);


            if(numObjects>0){
                stat.addValue(x);
                xpos += x*gainx;
                if(x>255/2 + 3){
                    xpos += .01;
                }

                if(xpos >= Servo.MAX_POSITION){
                    xpos = Servo.MAX_POSITION;
                }
                if(xpos<=Servo.MIN_POSITION){
                    xpos=Servo.MIN_POSITION;
                }
                if(xpos!=0) {
                    servo_tilt.setPosition(xpos);
                }
            }

            servo_tilt.setPosition(1-xpos);



            telemetry.addData("num obj 0", 0xff&pixyData[0]);
            telemetry.addData("x: 1", 0xff&pixyData[1]);
            telemetry.addData("y: 2", 0xff&pixyData[2]);
            telemetry.addData("width: 3", 0xff&pixyData[3]);
            telemetry.addData("height: 4", 0xff&pixyData[4]);
            telemetry.addData("stats" ,stat.toString());
            //telemetry.addData("Length", pixyData.length);
            telemetry.update();
            sleep (50);
        }

    }


}