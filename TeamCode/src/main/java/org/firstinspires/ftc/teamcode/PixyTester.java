package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.teamcode.classes.AdafruitIMU;
import org.firstinspires.ftc.teamcode.classes.Pixy;

/**
 * Created by vatty on 9/16/2017.
 */

@Autonomous(name = "pixyTester")
public class PixyTester extends LinearOpMode{
    I2cDeviceSynch pixy;

    Pixy cam;


    public void runOpMode(){
        pixy = hardwareMap.i2cDeviceSynch.get("pixy");

        cam = new Pixy(pixy);

        waitForStart();

        cam.engage();

        while (opModeIsActive()){
            telemetry.addData("Data 1",cam.getX());
            telemetry.addData("Data 2", cam.getY());
            telemetry.addData("Data 0", cam.numobjects());
            telemetry.update();

        }
    }
}
