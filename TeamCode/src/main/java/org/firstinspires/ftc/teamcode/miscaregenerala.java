package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class miscaregenerala extends LinearOpMode {
    DcMotor brat;
    Servo servo_brat;


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

        brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        stangafata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreaptafata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);;
        stangaspate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);;
        dreaptaspate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);;

        waitForStart();

        while (opModeIsActive()) {

            //control robot
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x; // turn
            double turn = -gamepad1.right_stick_x;  //strafe

            mecanum(x, y, turn);

            lansator.setPower(gamepad1.right_trigger);

            //control brat
            if (gamepad1.left_bumper) {
                brat.setPower(-0.9);
            } else if (gamepad1.right_bumper) {
                brat.setPower(0.9);
            } else if (gamepad1.x) {
                brat.setPower(0.3);
            } else if (gamepad1.y) {
                brat.setPower(-0.3);
            } else {
                brat.setPower(0);
            }

            //control servo v2
            if (gamepad1.a) {
                servo_brat.setPosition(0);
            } else if (gamepad1.b) {
                servo_brat.setPosition(0.5);
            }

            if (gamepad1.dpad_up){
                servo_rampa.setPosition(1);
                sleep(2420);
            }
            else if (gamepad1.dpad_down){
                servo_rampa.setPosition(0);
                sleep(2420);
            }
            else {
                servo_rampa.setPosition(0.5);
            }
            if (gamepad1.left_trigger > 0){
                servo_ghidaj.setPosition(1);
                intake.setPower(1);
            }
            else {
                servo_ghidaj.setPosition(0.5);
                intake.setPower(0);
            }
            if (gamepad1.dpad_left) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }
        }
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
        dreaptafata.setPower(-v2);
        stangaspate.setPower(v3);
        dreaptaspate.setPower(-v4);
    }

}


