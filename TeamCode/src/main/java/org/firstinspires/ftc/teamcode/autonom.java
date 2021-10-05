package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "UltimateGoalAutonomous", group = "FTC")
public class autonom extends LinearOpMode {

    //private static final String TFOD_MODEL_ASSET = "model_unquant.tflite";
    private static final String LABEL_A = "0A";
    private static final String LABEL_B = "1B";
    private static final String LABEL_C = "4C";

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "AexRVhH/////AAABmRwlvZsGx0Oor/vwJ7jQe7w0CCm9dj4XqZzZM+GKL0bAOBWbJZCukHVq80UOiV4X6fZipT53Y/ekerVZ4Y73NnXBy3fxFkz11J6LweNoe5HZNQEXbeCuTGGc4XhidpQPDhXGjwQW302VtF6gK4z9Sru7Lqyu+eYSeSfy8UhVs2VYLlCuP8vO8gJCbFG8dptNQGn/NVZP7BTugsioepH2DnoKmkj1kwMdbiQGZkAOLYrI/RqPVdR1qOyqY2dX4s2N3LPWkN39fh6VVMm7A353UAE4OYDPgj9Id4wWBlKUL0inI5TgbMFRTkPcvykUDS1N29aZ6tmBfIixe/RRWQXh4WAteCeZ34wMnL/bts8EDy4g";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private BNO055IMU imu;

    // movement motors
    DcMotor stangafata;
    DcMotor dreaptafata;
    DcMotor stangaspate;
    DcMotor dreaptaspate;

    //servos
    Servo servo_brat;
    Servo servo_rampa;
    Servo servo_ghidaj;

    //hex motors
    DcMotor brat;
    DcMotor intake;
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
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);
        }

        stangafata = hardwareMap.dcMotor.get("stangafata");
        dreaptafata = hardwareMap.dcMotor.get("dreaptafata");
        stangaspate = hardwareMap.dcMotor.get("stangaspate");
        dreaptaspate = hardwareMap.dcMotor.get("dreaptaspate");
        intake = hardwareMap.dcMotor.get("intake");
        lansator = hardwareMap.dcMotor.get("lansator");

        brat= hardwareMap.dcMotor.get("brat");

        servo_brat = hardwareMap.servo.get("servobrat");
        servo_ghidaj = hardwareMap.servo.get("servoghidaj");
        servo_rampa = hardwareMap.servo.get("servorampa");

        dreaptafata.setDirection(DcMotorSimple.Direction.FORWARD);
        dreaptaspate.setDirection(DcMotorSimple.Direction.FORWARD);
        stangafata.setDirection(DcMotorSimple.Direction.REVERSE);
        stangaspate.setDirection(DcMotorSimple.Direction.REVERSE);

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
            if (!didFunctionRun)
                doSmartStuff();
            else requestOpModeStop();
        }
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void cazA() {
        servo_brat.setPosition(0);
        strafeToPosition(-30, 0.5);
        moveToPosition(-170, 1);
        sleep(50);
        strafeToPosition(30, 0.5);
        turnWithGyro(-35, -0.5);
        moveToPosition(-20, 0.5);
        dropWobble();
        moveToPosition(20, 0.5);
        turnWithGyro(-35, 0.5);
        strafeToPosition(40, 0.5);
        turnWithGyro(-180, -0.6);
        moveToPosition(-20, 0.5);
        strafeToPosition(40, 0.5);
        // -90, -178, 0
        //lansator.setPower();
    }

    private void cazB() {
        servo_brat.setPosition(0);
        strafeToPosition(-30, 0.5);
        moveToPosition(-260, 1);
        sleep(50);
        strafeToPosition(80, 0.5);
        turnWithGyro(-10, -0.5);
        //moveToPosition(-20, 0.5);
        dropWobble();
        moveToPosition(50, 0.5);
        strafeToPosition(40, 0.5);

    }

    private void cazC() {
        servo_brat.setPosition(0);
        strafeToPosition(-30, 0.5);
        moveToPosition(-350, 1);
        sleep(50);
        strafeToPosition(30, 0.7);
        turnWithGyro(-27, -0.7);
        dropWobble();
        turnWithGyro(-27, 1);
        strafeToPosition(35, 0.7);
        moveToPosition(165, 1);
        turnWithGyro(-170, -1);
        strafeToPosition(40, 0.5);
        turnWithGyro(-10, -0.5);
        dropBridge();
        launch();
    }

    private void launch() {
        intake.setPower(1);
        servo_ghidaj.setPosition(0);
        sleep(3500);
    }

    private void dropBridge() {
        lansator.setPower(0.61);
        servo_rampa.setPosition(1);
        sleep(2600);
        servo_rampa.setPosition(0.5);
    }

    private void dropWobble() {
        sleep(50);
        moveMotorToPosition(-70, 1);
        moveMotorToPosition(brat.getCurrentPosition() - 20, 0.5);
        sleep(500);
        servo_brat.setPosition(0.65);
        sleep(50);
        moveMotorToPosition(10, .85);
        servo_brat.setPosition(0);
    }

    private boolean gotEm = false;
    private void doSmartStuff() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            sleep(750);
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null && !gotEm) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                gotEm = true;
                if (updatedRecognitions.size() == 1) {
                    Recognition recognition = updatedRecognitions.get(0);
                    telemetry.addData("Recon", recognition.getLabel());
                    telemetry.update();
                    if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                        cazB();
                    } else
                        cazC();
                } else {
                    cazA();
                }
                didFunctionRun = true;
            } else requestOpModeStop();
        }
    }

    private void DoAutonomusStuff(boolean didFunctionRun){
        if(!didFunctionRun){
            // position 1(A) pos 2(B) pos 3(C)
            doSmartStuff();

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
        dreaptafata.setPower(speed);
        stangaspate.setPower(speed);
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

    public void moveMotorToPosition(int position, double speed) {
        brat.setTargetPosition(position);
        brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brat.setPower(speed);
        while (brat.isBusy()) {}
        brat.setPower(0);
    }

    /*
    This function uses the Expansion Hub IMU Integrated Gyro to turn a precise number of degrees (+/- 5).
    Degrees should always be positive, make speedDirection negative to turn left.
     */
    public void turnWithGyro(double degrees, double speedDirection){
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
        if (speedDirection > 0){//set target positions
            if (degrees > 10){
                first = (degrees - 10) + devertify(yaw);
            } else {
                first = devertify(yaw);
            }
            second = degrees + devertify(yaw);
        } else {
            if (degrees > 10){
                first = devertify(-(degrees - 10) + devertify(yaw));
            } else {
                first = devertify(yaw);
            }
            second = devertify(-degrees + devertify(yaw));
        }
        //
        double firsta = convertify(first - 5);//175
        double firstb = convertify(first + 5);//-175
        turnWithEncoder(speedDirection);
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
        double seconda = convertify(second - 5);//175
        double secondb = convertify(second + 5);//-175
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
        stangaspate.setTargetPosition(stangaspate.getCurrentPosition() - move);
        dreaptaspate.setTargetPosition(dreaptaspate.getCurrentPosition() + move);

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
        parameters.loggingEnabled      = false;
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

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }






}