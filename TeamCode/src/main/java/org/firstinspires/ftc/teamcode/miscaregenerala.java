package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.nio.channels.ShutdownChannelGroupException;

@TeleOp
public class miscaregenerala extends LinearOpMode {
    DcMotor brat;
    Servo servo_brat;
    Servo servo_rampa;

    DcMotor lansator;

    DcMotor stangafata;
    DcMotor dreaptafata;
    DcMotor stangaspate;
    DcMotor dreaptaspate;
    DcMotor hex_rampa;

    @Override
    public void runOpMode() {
        stangafata = hardwareMap.dcMotor.get("stangafata");
        dreaptafata = hardwareMap.dcMotor.get("dreaptafata");
        stangaspate = hardwareMap.dcMotor.get("stangaspate");
        dreaptaspate = hardwareMap.dcMotor.get("dreaptaspate");
        brat = hardwareMap.dcMotor.get("brat");
        servo_brat = hardwareMap.servo.get("servobrat");
        servo_rampa = hardwareMap.servo.get("servorampa");
        hex_rampa = hardwareMap.dcMotor.get("rampa");

        lansator = hardwareMap.dcMotor.get("lansator");

        brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // hub 1
        // hub 2

        waitForStart();

        while (opModeIsActive()) {

            //control robot
            double forward = -gamepad1.left_stick_y;
            double turn = -gamepad1.left_stick_x; // turn
            double strafe = gamepad1.right_stick_x;  //strafe
              //Forward

            mecanum(strafe, forward, turn);

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
                servo_rampa.setPosition(0);
            }
            else if (gamepad1.dpad_down){
                servo_rampa.setPosition(1);
            }




            if (gamepad1.dpad_left) {
                hex_rampa.setPower(1);
            } else {
                hex_rampa.setPower(0);
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