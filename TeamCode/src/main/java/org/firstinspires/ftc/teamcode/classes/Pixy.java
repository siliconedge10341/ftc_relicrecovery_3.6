package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
/**
 * Created by vatty on 9/23/2017.
 */

public class Pixy {
    I2cDeviceSynch pixy;

    public Pixy(I2cDeviceSynch hardwaremap){
        pixy = hardwaremap;
        pixy.setI2cAddress(I2cAddr.create7bit(0x54));
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow (1, 26, I2cDeviceSynch.ReadMode.REPEAT);
        pixy.setReadWindow(readWindow);
    }

    public void engage(){
        pixy.engage();
    }

    public double getX(){
        return (pixy.read8(1));
    }

    public double getY(){
        return (pixy.read8(2) );
    }

    public int numobjects(){
       return (pixy.read8(0));
    }


}
/*
Bytes    16-bit word    Description
        ----------------------------------------------------------------
        0, 1     y              sync: 0xaa55=normal object, 0xaa56=color code object
        2, 3     y              checksum (sum of all 16-bit words 2-6, that is, bytes 4-13)
        4, 5     y              signature number
        6, 7     y              x center of object
        8, 9     y              y center of object
        10, 11   y              width of object
        12, 13   y              height of object
        */
