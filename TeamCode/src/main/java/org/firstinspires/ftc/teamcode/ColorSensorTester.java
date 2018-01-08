package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.classes.AdafruitIMU;

/**
 * Created by vatty on 9/16/2017.
 */

@Autonomous(name = "colorTester")
@Disabled
public class ColorSensorTester extends LinearOpMode{
    ColorSensor sensorColor;

    public void runOpMode(){

        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("Hu" , sensorColor.argb());

            telemetry.update();
        }
    }
}
