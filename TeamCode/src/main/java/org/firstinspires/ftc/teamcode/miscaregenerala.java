package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp
public class miscaregenerala extends LinearOpMode {
    DcMotor brat;
    Servo servo_brat;

    private BNO055IMU imu;

    Servo servo_rampa;

    Servo servo_ghidaj;

    DcMotor lansator;

    DcMotor stangafata;
    DcMotor dreaptafata;
    DcMotor stangaspate;
    DcMotor dreaptaspate;
    DcMotor intake;

    @Override
    public void runOpMode() {
        initGyro();
        stangafata = hardwareMap.dcMotor.get("stangafata");         // hub 1 port 0
        dreaptafata = hardwareMap.dcMotor.get("dreaptafata");       // hub 1 port 1
        stangaspate = hardwareMap.dcMotor.get("stangaspate");       // hub 1 port 2
        dreaptaspate = hardwareMap.dcMotor.get("dreaptaspate");     // hub 1 port 3

        brat = hardwareMap.dcMotor.get("brat");
        servo_brat = hardwareMap.servo.get("servobrat");
        servo_rampa = hardwareMap.servo.get("servorampa");
        intake = hardwareMap.dcMotor.get("intake");
        servo_ghidaj = hardwareMap.servo.get("servoghidaj");

        lansator = hardwareMap.dcMotor.get("lansator");

        dreaptafata.setDirection(DcMotorSimple.Direction.FORWARD);
        dreaptaspate.setDirection(DcMotorSimple.Direction.FORWARD);
        stangafata.setDirection(DcMotorSimple.Direction.REVERSE);
        stangaspate.setDirection(DcMotorSimple.Direction.REVERSE);

        stangafata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreaptafata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stangaspate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreaptaspate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //CALIBRARE GYRO START
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        waitForStart();
        double slower = 1;
        boolean fullIntake = true;
        while (opModeIsActive()) {
            /**
             * Gamepad 1 -----------------------------------------------
             */

            /**
             * Miscare robot
             */
            double x = -gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;

            if (gamepad1.right_bumper) {
                if (slower == 1) {
                    slower = 2;
                } else {
                    slower = 1;
                }
            }
            mecanum(x, y, turn);

            /**
             * Servo Brat
             */
            if (gamepad2.right_trigger > 0)
                servo_brat.setPosition(0.65);
            if (gamepad2.left_trigger > 0)
                servo_brat.setPosition(0);

            /**
             * ---------------------------------------------------------
             */


            /**
             * Gamepad 2 -----------------------------------------------
             */

            /**
             * Lansator
             */
            if (gamepad2.x) {
                //if (lansator.getPower() == 0) {
                lansator.setPower(0.61);

                //} else if (lansator.getPower() <= 0.7) {
                //    lansator.setPower(lansator.getPower() + 0.1);
                //}
            }
            if (gamepad2.b) {
                //if (lansator.getPower() >= 0.1) {
                //   lansator.setPower(lansator.getPower() - 0.1);
                //}
                lansator.setPower(0);
            }
            if (lansator.getPower() < 0)
                lansator.setPower(0);


            /**
             * Brat Wobble
             */
            if (gamepad2.left_bumper) {
                brat.setPower(-1);
            } else if (gamepad2.right_bumper) {
                brat.setPower(1);
            } else if (gamepad2.a) {
                brat.setPower(0.5);
            } else if (gamepad2.y) {
                brat.setPower(-0.3);
            } else {
                brat.setPower(0);
            }

            telemetry.addData("Brat", brat.getCurrentPosition());

            /**
             * Rampa
             */
            if (gamepad2.dpad_up) {
                servo_rampa.setPosition(1);
            } else if (gamepad2.dpad_down) {
                servo_rampa.setPosition(0);
            } else {
                servo_rampa.setPosition(0.5);
            }

            /**
             * Intake
             */
            if (gamepad2.right_stick_button) {
                fullIntake = !fullIntake;
            }
            if (gamepad2.dpad_left) { //intake forward
                intake.setPower(1);
                //if (fullIntake)
                    servo_ghidaj.setPosition(0);
            }
            else if (gamepad2.dpad_right) { //intake reverse
                intake.setPower(-1);
                //if (fullIntake)
                    servo_ghidaj.setPosition(1);
            } else {
                intake.setPower(0);
                servo_ghidaj.setPosition(0.5);
            }
            /**
             * ---------------------------------------------------------
             */
            telemetry.addData("X", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle);
            telemetry.addData("Z", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("Y", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle);
            telemetry.update();
        }
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

    /**
     * Control a mecanum drive base with three double inputs
     *
     * @param Strafe  is the first double X value which represents how the base should strafe
     * @param Forward is the only double Y value which represents how the base should drive forward
     * @param Turn    is the second double X value which represents how the base should turn
     */
    public void mecanum(double Strafe, double Forward, double Turn) {
        //Find the magnitude of the controller's input
        double r = Math.hypot(Strafe, Forward);

        //returns point from +X axis to point (forward, strafe)
        double robotAngle = Math.atan2(Forward, Strafe) - Math.PI / 4;

        //Quantity to turn by (turn)
        double rightX = Turn;

        //double vX represents the velocities sent to each motor
        final double v1 = (r * Math.cos(robotAngle)) + rightX;
        final double v2 = (r * Math.sin(robotAngle)) - rightX;
        final double v3 = (r * Math.sin(robotAngle)) + rightX;
        final double v4 = (r * Math.cos(robotAngle)) - rightX;

        stangafata.setPower(v1);
        dreaptafata.setPower(v2);
        stangaspate.setPower(v3);
        dreaptaspate.setPower(v4);
    }

}


