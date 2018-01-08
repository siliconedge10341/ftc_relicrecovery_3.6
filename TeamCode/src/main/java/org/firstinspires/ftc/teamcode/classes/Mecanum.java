package org.firstinspires.ftc.teamcode.classes;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;



/**

 * Created by the lord and savior Arko Chaterjee, Richard the genius, and Krimps on 9/7/2016.

 */

public class Mecanum{

    // instance variables
    // private variables
    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBR;
    private DcMotor motorBL;
    // constants

    private final double pi = 3.1415926;

    private final double wheel_diameter = 4 * (pi);
    // powers

    private double FRpower = 0;

    private double FLpower = 0;

    private double BRpower = 0;

    private double BLpower = 0;

    private double BasePower = 0;


    // constructors

    public Mecanum() {
        // default constructors
        FRpower = 0;
        FLpower = 0;
        BRpower = 0;
        BLpower = 0;
        BasePower = .5;

    }
    public Mecanum(DcMotor motorFR,DcMotor motorFL,DcMotor motorBR,DcMotor motorBL){
        this.motorFR = motorFR;
        this.motorFL = motorFL;
        this.motorBR = motorBR;
        this.motorBL = motorBL;
    }


    // sets

    public void set_Power(float rjoystick_x, float ljoystick_y, float ljoystick_x, boolean percision_flag) {

        float ch1 = rjoystick_x;

        float ch3 = ljoystick_x;

        float ch4 = ljoystick_y;

        double midfl, midbl, midfr, midbr;

        boolean FL = ch4 > ch3;

        boolean FR = ch4 > -ch3;

        if (ch1 < -0.4 || ch1 > 0.4) {

            midfl = ch1;

            midbl = ch1;

            midfr = ch1;

            midbr = ch1;

        } else if (FL && FR) {

            midfl = ch4;

            midbl = ch4;

            midfr = -ch4;

            midbr = -ch4;

        } else if (FL || FR) {

            if (FL) {

                //

                // Will be adjusted

                //

                midfl = ch3; //0.7

                midbl = -ch3;  //0.95

                midfr = ch3; //0.78

                midbr = -ch3;  //1

            } else {

                midfl = ch3; //.78

                midbl = -ch3;  //1

                midfr = ch3; //0.7

                midbr = -ch3;  //0.95

            }

        } else {

            midfl = ch4;

            midbl = ch4;

            midfr = -ch4;

            midbr = -ch4;

        }

        //

        // Power has to be over a certain amount. We found this to work because the amount of

        // power that is below 1 together doesn't move the robot anyways. So, in order to protect

        // the motors, we did this in case.

        //

        if (Math.abs(midfl) + Math.abs(midbl) + Math.abs(midfr) + Math.abs(midbr) < 1)

        {

            midfl = 0;

            midbl = 0;

            midfr = 0;

            midbr = 0;

        }


        FLpower = midfl;

        BLpower = midbl;

        FRpower = midfr;

        BRpower = midbr;


        if (Math.abs(FLpower) + Math.abs(BLpower) + Math.abs(FRpower) + Math.abs(BRpower) < 0.5)

        {

            FLpower = 0;

            BLpower = 0;

            FRpower = 0;

            BRpower = 0;

        }


    }

    public void setPosition(int fr, int fl, int br, int bl){
        // Sets the motor position to something and the runs it.

        motorFR.setTargetPosition(fr);

        motorFL.setTargetPosition(fl);

        motorBR.setTargetPosition(br);

        motorBL.setTargetPosition(bl);


    }

    public void setPowerD(double power) {

        BasePower = power;

    }


    // gets

    public double get_wheel_rotations() {

        double distance = 0;

        double wheel_rotation = 0;

        wheel_rotation = distance / wheel_diameter;


        return wheel_rotation;

    }


    // methods

    public void run_motor() {

        motorFR.setPower(FRpower);

        motorBR.setPower(BRpower);

        motorFL.setPower(FLpower);

        motorBL.setPower(BLpower);

    }


    // testers


    // directions for the robot to run


    // methods

    //

    // These are the functions for the specific direction

    //

    public void run_left() {

        FLpower = -BasePower;
        BLpower = BasePower;
        FRpower = -BasePower;
        BRpower = BasePower;

        // The positive and negative inputs only mean direction, not speed.

        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower(FLpower);
        motorBL.setPower(BLpower);

    }

    public void run_right() {

        FLpower = BasePower;

        BLpower = -BasePower;

        FRpower = BasePower;

        BRpower = -BasePower;

        //

        // The positive and negative inputs only mean direction, not speed.

        //

        motorFR.setPower(FRpower);

        motorBR.setPower(BRpower);

        motorFL.setPower(FLpower);

        motorBL.setPower(BLpower);

        //

        // This sets the motors that go into DemoAutonomous.

        //

    }

    public void run_forward() {

        FLpower = BasePower;

        BLpower = BasePower;

        FRpower = -BasePower;

        BRpower = -BasePower;


        motorFR.setPower(FRpower);

        motorBR.setPower(BRpower);

        motorFL.setPower(FLpower);

        motorBL.setPower(BLpower);


    }

    public void run_backward() {

        FLpower = -BasePower;

        BLpower = -BasePower;

        FRpower = BasePower;

        BRpower = BasePower;

        //

        // The positive and negative inputs only mean direction, not speed.

        //

        motorFR.setPower(FRpower);

        motorBR.setPower(BRpower);

        motorFL.setPower(FLpower);

        motorBL.setPower(BLpower);

        //

        // This sets the motors that go into DemoAutonomous.

        //

    }

    public void run_diagonal_right_up() {

        FLpower = BasePower;

        BLpower = 0;

        FRpower = 0;

        BRpower = BasePower;

        //

        // The positive and negative inputs only mean direction, not speed.

        //

        motorFR.setPower(-FRpower);

        motorBR.setPower(-BRpower);

        motorFL.setPower((FLpower));

        motorBL.setPower(BLpower);

        //

        // This sets the motors that go into DemoAutonomous.

        //

    }

    public void run_diagonal_left_up() {

        FLpower = BasePower;

        BLpower = 0;

        FRpower = 0;

        BRpower = -BasePower;

        //

        // The positive and negative inputs only mean direction, not speed.

        //

        motorFR.setPower(FRpower);

        motorBR.setPower(BRpower);

        motorFL.setPower(FLpower);

        motorBL.setPower(BLpower);

        //

        // This sets the motors that go into DemoAutonomous.

        //

    }

    public void run_diagonal_left_down() {

        FLpower = -BasePower;

        BLpower = 0;

        FRpower = 0;

        BRpower = -BasePower;

        //

        // The positive and negative inputs only mean direction, not speed.

        //

        motorFR.setPower(-FRpower);

        motorBR.setPower(-BRpower);

        motorFL.setPower((FLpower));

        motorBL.setPower(BLpower);

        //

        // This sets the motors that go into DemoAutonomous.

        //

    }

    public void run_diagonal_right_down() {

        FLpower = 0;

        BLpower = -BasePower;

        FRpower = -BasePower;

        BRpower = 0;

        //

        // The positive and negative inputs only mean direction, not speed.

        //

        motorFR.setPower(-FRpower);

        motorBR.setPower(-BRpower);

        motorFL.setPower((FLpower));

        motorBL.setPower(BLpower);

        //

        // This sets the motors that go into DemoAutonomous.

        //

    }

    public void turn_right() {

        FLpower = BasePower;
        BLpower = BasePower;
        FRpower = BasePower;
        BRpower = BasePower;

        // The positive and negative inputs only mean direction, not speed.

        // This sets the motors that go into DemoAutonomous.

        motorFR.setPower(FRpower);

        motorBR.setPower(BRpower);

        motorFL.setPower(FLpower);

        motorBL.setPower(BLpower);

    }

    public void turn_left() {

        FLpower = -BasePower;

        BLpower = -BasePower;

        FRpower = -BasePower;

        BRpower = -BasePower;

        //

        // The positive and negative inputs only mean direction, not speed.

        // This sets the motors that go into DemoAutonomous.

        //

        motorFR.setPower(FRpower);

        motorBR.setPower(BRpower);

        motorFL.setPower(FLpower);

        motorBL.setPower(BLpower);

    }

    public void brake() {

        FLpower = 0;

        BLpower = 0;

        FRpower = 0;

        BRpower = 0;

        //

        // The positive and negative inputs only mean direction, not speed.

        //

        motorFR.setPower(FRpower);

        motorBR.setPower(BRpower);

        motorFL.setPower(FLpower);

        motorBL.setPower(BLpower);

        //

        // This sets the motors that go into DemoAutonomous.

        //

    }


    // resets

    public void reset_encoders() {

        //

        // The positive and negative inputs only mean direction, not speed.

        //

        motorFR.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motorBR.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motorFL.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motorBL.setMode(DcMotor.RunMode.RESET_ENCODERS);

        //

        // This sets the motors that go into DemoAutonomous.

        //

    }

    public void run_to_position() {

        //

        // The positive and negative inputs only mean direction, not speed.

        //

        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //

        // This sets the motors that go into DemoAutonomous.

        //

    }

    public void run_using_encoders() {

        //

        // The positive and negative inputs only mean direction, not speed.

        //

        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //

        // This sets the motors that go into DemoAutonomous.

        //

    }

    public void run_without_encoders() {

        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void drive_forward_gyro(double lpower, double rpower){
        FLpower = lpower;
        BLpower = lpower;
        FRpower = -rpower;
        BRpower = -rpower;
    }

    public double getSpeed() {
        return BasePower;
    }

    public Boolean isBusy(){
        if (motorFR.isBusy() || motorBL.isBusy()|| motorBR.isBusy()|| motorFL.isBusy()){
            return true;
        }else{
            return false;
        }
    }

    public int testDistance(DcMotor A_Motor) {
        // This takes any motor and tests the distance that that motor has traveled
        // We only do one because all have traveled the same amount. Knowing Turns
        // Do not count.

        if (Math.abs(A_Motor.getCurrentPosition()) >= Math.abs(A_Motor.getTargetPosition())){
            return 1;
        }else{
            return 0;
        }
    }


}