package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.nio.channels.ShutdownChannelGroupException;

@TeleOp
public class miscaregenerala extends LinearOpMode {
    DcMotor brat;
    Servo servo1;

    DcMotor stangafata;
    DcMotor dreaptafata;
    DcMotor stangaspate;
    DcMotor dreaptaspate;

    DcMotor yeeter;
    @Override
    public void runOpMode() throws InterruptedException {
       stangafata = hardwareMap.dcMotor.get("stangafata");
       dreaptafata = hardwareMap.dcMotor.get("dreaptafata");
       stangaspate = hardwareMap.dcMotor.get("stangaspate");
       dreaptaspate = hardwareMap.dcMotor.get("dreaptaspate");
       brat = hardwareMap.dcMotor.get("brat");
       servo1 = hardwareMap.servo.get("servobrat");
       yeeter = =hardwareMap.dcMotor.get("yeeter");

       // hub 1 
       // hub 2

        waitForStart();

        while(opModeIsActive()){
            //control robot
            double x = -gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            mecanum(x, y, turn);

            //control brat
            if (gamepad1.left_bumper){
                brat.setPower(-0.9);
            }
            else if(gamepad1.right_bumper){
                brat.setPower(0.9);
            }
            else if (gamepad1.x){
                brat.setPower(0.3);
                brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else if (gamepad1.y){
                brat.setPower(-0.3);
                brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else {
                brat.setPower(0);
            }

            //control servo v2

            if (gamepad1.a){
                servo1.setPosition(0);
            }
            else if(gamepad1.b){
                servo1.setPosition(0.6);
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
        dreaptafata.setPower(v2);
        stangaspate.setPower(v3);
        dreaptaspate.setPower(v4);
    }



}
