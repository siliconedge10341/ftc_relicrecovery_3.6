/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="motorConfigure rrt", group="Pushbot")
@Disabled
public class ConfigTester extends LinearOpMode {

    /* Declare OpMode members. */

    private ElapsedTime     runtime = new ElapsedTime();
    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBR;
    private DcMotor motorBL;


    @Override
    public void runOpMode() {
        motorFL = hardwareMap.dcMotor.get("fl");
        motorFR = hardwareMap.dcMotor.get("fr");
        motorBL = hardwareMap.dcMotor.get("bl");
        motorBR = hardwareMap.dcMotor.get("br");
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        waitForStart();


        runtime.reset();

        motorFR.setPower(.2);
        while (runtime.seconds()<=1){
            telemetry.addLine("Front Right");
        }
        motorFR.setPower(0);

        runtime.reset();
        motorFL.setPower(.2);
        while (runtime.seconds()<=1){
            telemetry.addLine("Front LeFT");

        }
        motorFL.setPower(0);

        runtime.reset();
        motorBR.setPower(.2);
        while (runtime.seconds()<=1){
            telemetry.addLine("Back Right");
        }
        motorBR.setPower(0);

        runtime.reset();
        motorBL.setPower(.2);
        while (runtime.seconds()<=1){
            telemetry.addLine("Back LeFT");

        }
        motorBL.setPower(0);



        motorFL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);

        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFR.setTargetPosition(720);
        motorFL.setTargetPosition(720);
        motorBR.setTargetPosition(720);
        motorBL.setTargetPosition(720);

        motorFR.setPower(.4);
        motorFL.setPower(.4);
        motorBR.setPower(.4);
        motorBL.setPower(.4);

        while (motorFR.isBusy() && motorFL.isBusy()&& motorBL.isBusy()&& motorBR.isBusy() && opModeIsActive()){
            telemetry.addData("FR" , motorFR.getCurrentPosition());
            telemetry.addData("FL" , motorFL.getCurrentPosition());
            telemetry.addData("BR" , motorBR.getCurrentPosition());
            telemetry.addData("BL" , motorBL.getCurrentPosition());
            telemetry.update();
        }

        motorBL.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);




        sleep(1000);

    }
}
