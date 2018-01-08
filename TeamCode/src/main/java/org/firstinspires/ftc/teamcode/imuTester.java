package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.classes.AdafruitIMU;

/**
 * Created by vatty on 9/16/2017.
 */

@Autonomous(name = "imuTester gyro")
public class imuTester extends LinearOpMode{
    AdafruitIMU imu = new AdafruitIMU();

    double accel;
    double vel;

    public void runOpMode(){
        imu = new AdafruitIMU(hardwareMap.get(BNO055IMU.class, "imu"));

        imu.init();
        accel = 0;

        waitForStart();
        imu.start();

        while (opModeIsActive()){

            accel = Math.sqrt(imu.getAccelX()*imu.getAccelX() + imu.getAccelZ()*imu.getAccelZ() + imu.getAccelY()*imu.getAccelY());
            //vel = Math.sqrt(imu.getVelocityX()*imu.getAccelX()+imu.getVelocityY()*imu.getVelocityY()+imu.getVelocityZ()*imu.getVelocityZ());

            telemetry.addData("Heading: ", imu.getHeading());
            //telemetry.addData("Velocity " , vel);

            telemetry.addData("Acceleration" , accel);
            telemetry.addData("Gravity" , imu.getGravity());
            telemetry.update();
        }
    }
}
