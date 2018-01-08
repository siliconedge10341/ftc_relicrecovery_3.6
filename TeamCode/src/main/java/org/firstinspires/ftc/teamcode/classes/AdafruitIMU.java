package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by Durga,
 */


public class AdafruitIMU {
    BNO055IMU imu;
    Orientation angles;
    public AdafruitIMU(){


    }

    public AdafruitIMU(BNO055IMU hardwaremap){

        imu = hardwaremap;

    }
    public void init(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
       // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
    }

    public void start(){
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

    }
//Durga is your lover

    public double getHeading(){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle + 180;
    }
    public double getPitch(){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.thirdAngle;
    }
    public double getRoll(){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.secondAngle;
    }
    public double getVelocityX(){
        Velocity v = imu.getVelocity();
        return v.xVeloc;
    }
    public double getVelocityY(){
        Velocity v = imu.getVelocity();
        return v.yVeloc;
    }
    public double getVelocityZ(){
        Velocity v = imu.getVelocity();
        return v.zVeloc;
    }
    public double getAccelX(){
        return  imu.getAcceleration().xAccel;
    }
    public double getAccelZ(){
        return  imu.getAcceleration().zAccel;
    }
    public double getAccelY(){
        return  imu.getAcceleration().yAccel;
    }
    public String getGravity(){
        return imu.getGravity().toString();
    }





}
