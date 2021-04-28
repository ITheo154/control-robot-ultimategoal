package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "UltimateGoalAutonomous", group = "FTC")
public class autonom extends LinearOpMode {

    private BNO055IMU imu;

    // movement motors
    DcMotor stangafata;
    DcMotor dreaptafata;
    DcMotor stangaspate;
    DcMotor dreaptaspate;

    //servos
    Servo servo_brat;
    Servo servo_rampa;

    //hex motors
    DcMotor brat;
    DcMotor rampa;
    DcMotor lansator;


    private boolean didFunctionRun = false;

    private double width = 16.0; //inches
    private int cpr = 28; //counts per rotation
    private int gearratio = 40;
    private double diameter = 4.125;
    private double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    private double bias = 0.8;//default 0.8
    private double meccyBias = 0.9;//change to adjust only strafing movement
    //
    private double conversion = cpi * bias;
    private boolean exit = false;
    //
    private Orientation angles;
    private Acceleration gravity;

    @Override
    public void runOpMode() {
        initGyro();

        stangafata = hardwareMap.dcMotor.get("stangafata");
        dreaptafata = hardwareMap.dcMotor.get("dreaptafata");
        stangaspate = hardwareMap.dcMotor.get("stangaspate");
        dreaptaspate = hardwareMap.dcMotor.get("dreaptaspate");
        rampa = hardwareMap.dcMotor.get("rampa");
        lansator = hardwareMap.dcMotor.get("lansator");





        stangafata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreaptafata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stangaspate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreaptaspate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        reversePolarity();


        //CALIBRARE GYRO START
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }


        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("IMU Calibration Status :", imu.getCalibrationStatus().toString());
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            DoAutonomusStuff(didFunctionRun);
        }
    }

    private void DoAutonomusStuff(boolean didFunctionRun){
        if(!didFunctionRun){
            // position 1(A) pos 2(B) pos 3(C)
            deliverWobble();
            shootPowershot();
            navigateToStart();
            
            didFunctionRun = true;

        }
    }


    /*
    This function's purpose is simply to drive forward or backward.
    To drive backward, simply make the centimeter input negative.
     */
    public void moveToPosition(double cm, double speed){
        //
        double inches = cm / 2.54;
        int move = (int)(Math.round(inches*conversion));
        //
        stangaspate.setTargetPosition(stangaspate.getCurrentPosition() + move);
        stangafata.setTargetPosition(stangafata.getCurrentPosition() + move);
        dreaptaspate.setTargetPosition(dreaptaspate.getCurrentPosition() + move);
        dreaptafata.setTargetPosition(dreaptaspate.getCurrentPosition() + move);
        //
        stangafata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptafata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stangaspate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaspate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        stangafata.setPower(speed);
        stangaspate.setPower(speed);
        dreaptafata.setPower(speed);
        dreaptaspate.setPower(speed);
        //
        while (stangafata.isBusy() && dreaptafata.isBusy() && stangaspate.isBusy() && dreaptaspate.isBusy()){
            if (exit){
                dreaptafata.setPower(0);
                stangafata.setPower(0);
                dreaptaspate.setPower(0);
                stangaspate.setPower(0);
                return;
            }
        }
        dreaptafata.setPower(0);
        stangafata.setPower(0);
        dreaptaspate.setPower(0);
        stangaspate.setPower(0);
        return;
    }
    //
    /*
    This function uses the Expansion Hub IMU Integrated Gyro to turn a precise number of degrees (+/- 5).
    Degrees should always be positive, make speedDirection negative to turn left.
     */
    public void turnWithGyro(double degrees, double speedDirection){
        //<editor-fold desc="Initialize">
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;//make this negative
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();
        //
        telemetry.addData("stuff", speedDirection);
        telemetry.update();
        //
        double first;
        double second;
        //</editor-fold>
        //
        if (speedDirection > 0){//set target positions
            //<editor-fold desc="turn right">
            if (degrees > 10){
                first = (degrees - 10) + devertify(yaw);
                second = degrees + devertify(yaw);
            }else{
                first = devertify(yaw);
                second = degrees + devertify(yaw);
            }
            //</editor-fold>
        }else{
            //<editor-fold desc="turn left">
            if (degrees > 10){
                first = devertify(-(degrees - 10) + devertify(yaw));
                second = devertify(-degrees + devertify(yaw));
            }else{
                first = devertify(yaw);
                second = devertify(-degrees + devertify(yaw));
            }
            //
            //</editor-fold>
        }
        //
        //<editor-fold desc="Go to position">
        Double firsta = convertify(first - 5);//175
        Double firstb = convertify(first + 5);//-175
        //
        turnWithEncoder(speedDirection);
        //
        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }else{
            //
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }
        //
        Double seconda = convertify(second - 5);//175
        Double secondb = convertify(second + 5);//-175
        //
        turnWithEncoder(speedDirection / 3);
        //
        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            stangafata.setPower(0);
            dreaptafata.setPower(0);
            stangaspate.setPower(0);
            dreaptaspate.setPower(0);
        }
        //</editor-fold>
        //
        stangafata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptafata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangaspate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaspate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        stangafata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptafata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stangaspate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaspate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //
    /*
    This function uses the encoders to strafe left or right.
    Negative input for inches results in left strafing.
     */
    public void strafeToPosition(double cm, double speed){
        //
        double inches = cm / 2.54;
        int move = (int)(Math.round(inches * cpi * meccyBias));
        //

        stangafata.setTargetPosition(stangafata.getCurrentPosition() + move);
        dreaptafata.setTargetPosition(dreaptafata.getCurrentPosition() - move);
        stangaspate.setTargetPosition(stangaspate.getCurrentPosition() + move);
        dreaptaspate.setTargetPosition(dreaptaspate.getCurrentPosition() - move);

        //
        stangafata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptafata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stangaspate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaspate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //
        stangafata.setPower(speed);
        dreaptafata.setPower(speed);
        stangaspate.setPower(speed);
        dreaptaspate.setPower(speed);
        //
        while (stangafata.isBusy() && dreaptafata.isBusy() && stangaspate.isBusy() && dreaptaspate.isBusy()){}
        stangafata.setPower(0);
        dreaptafata.setPower(0);
        stangaspate.setPower(0);
        dreaptaspate.setPower(0);

        return;
    }
    //
    /*
    A tradition within the Thunder Pengwins code, we always start programs with waitForStartify,
    our way of adding personality to our programs.
     */
    public void waitForStartify(){
        waitForStart();
    }
    //
    /*
    These functions are used in the turnWithGyro function to ensure inputs
    are interpreted properly.
     */
    public double devertify(double degrees){
        if (degrees < 0){
            degrees = degrees + 360;
        }
        return degrees;
    }
    public double convertify(double degrees){
        if (degrees > 179){
            degrees = -(360 - degrees);
        } else if(degrees < -180){
            degrees = 360 + degrees;
        } else if(degrees > 360){
            degrees = degrees - 360;
        }
        return degrees;
    }


    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }


    public void turnWithEncoder(double input){
        stangafata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dreaptafata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stangaspate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dreaptaspate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        stangafata.setPower(input);
        dreaptafata.setPower(-input);
        stangaspate.setPower(input);
        dreaptaspate.setPower(-input);
    }

    private void reversePolarity(){
        stangafata.setDirection(DcMotorSimple.Direction.REVERSE);
        dreaptafata.setDirection(DcMotorSimple.Direction.FORWARD);
        stangaspate.setDirection(DcMotorSimple.Direction.REVERSE);
        dreaptaspate.setDirection(DcMotorSimple.Direction.FORWARD);


    }

    public void deliverWobble(int position){
        servo1.setPosition(0.65);
        if (position == 1){
            moveToPosition(180, 1);
            strafeToPosition(50, 0.8);
            servo1.setPosition(0);
            brat.setPower(1);
            sleep(100);
            
            // go to ring area 
            moveToPosition(-88, 1);
            strafeToPosition(87, 0.8);
            moveToPosition(83, 0.75);
        }
        else if (position == 2){
            moveToPosition(230, 1);
            strafeToPosition(-20, 0.8);
            servo1.setPosition(0);
            brat.setPower(1);
            sleep(100);

            // go to ring area
            strafeToPosition(56, 0.8);
            moveToPosition(-116, 1);
            strafeToPosition(86, 0.8);
            moveToPosition(84, 1);

        }
        else if (position == 3){
            moveToPosition(320, 1);
            strafeToPosition(50,0.8);
            servo1.setPosition(0);
            brat.setPower(1);
            sleep(100);
        }


    }

    public void shootHighGoal(){
        rampa.setPower(1);
        moveToPosition(65, 1);
        moveToPosition(85, 1);
        for (int i = 0; i<=3; i++) {
            lansator.setPower(1);
            sleep(20);
        }

    }

    public void shootPowershot(){
        rampa.setPower(1);

        for (int j = 0; j<=3; j++) {
            lansator.setPower(1);
            strafeToPosition(10, 0.8);
            sleep(10);
        }

    }

    public void navigateToStart(){
        moveToPosition(202, 1);
        strafeToPosition(10, 0.8);
        
    }






}