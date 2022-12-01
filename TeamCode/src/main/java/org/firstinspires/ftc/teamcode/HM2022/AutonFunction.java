package org.firstinspires.ftc.teamcode.HM2022;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
public class AutonFunction {
    ElapsedTime runtime;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    Servo claw;

    public AutonFunction(DcMotor leftFront, DcMotor rightFront,
                         DcMotor leftBack, DcMotor rightBack, Servo claw,
                         ElapsedTime runtime) {

        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        this.runtime = runtime;
        this.claw = claw;
    }
    public void goForward(double inches) throws InterruptedException {
        runtime.reset();
        leftFront.setPower(1);
        leftBack.setPower(1);
        rightFront.setPower(1);
        rightBack.setPower(1);
        double seconds = (0.787 / 24) * inches;
        while (runtime.seconds() < seconds) {

        }
        stop();
    }

    public void goBack(double inches) throws InterruptedException {
        runtime.reset();
        leftFront.setPower(-1);
        leftBack.setPower(-1);
        rightFront.setPower(-1);
        rightBack.setPower(-1);
        double seconds = (0.787 / 24) * inches;
        while (runtime.seconds() < seconds) {

        }
        stop();
    }

    public void goLeft(double inches) throws InterruptedException {
        runtime.reset();
        leftFront.setPower(-1);
        leftBack.setPower(1);
        rightFront.setPower(1);
        rightBack.setPower(-1);
        double seconds = (0.787 / 24) * inches;
        while (runtime.seconds() < seconds) {

        }
        stop();
    }

    public void goRight(double inches) throws InterruptedException {
        runtime.reset();
        leftFront.setPower(1);
        leftBack.setPower(-1);
        rightFront.setPower(-1);
        rightBack.setPower(1);
        double seconds = (0.787 / 24) * inches;
        while (runtime.seconds() < seconds) {

        }
        stop();
    }

    public void turnRight(double degrees) throws InterruptedException {
        runtime.reset();
        leftFront.setPower(-1);
        leftBack.setPower(-1);
        rightFront.setPower(1);
        rightBack.setPower(1);
        double seconds = 0.010837037037037 * degrees;
        while (runtime.seconds() < seconds) {

        }
        stop();
    }

    public void turnLeft(double degrees) throws InterruptedException {
        runtime.reset();
        leftFront.setPower(1);
        leftBack.setPower(1);
        rightFront.setPower(-1);
        rightBack.setPower(-1);
        double seconds = 0.010837037037037 * degrees;
        while (runtime.seconds() < seconds) {

        }
        stop();
    }
    public void claw(String command, double seconds) {
        if (command.equals("Open") || command.equals("open")) {
            claw.setPosition(10);
            runtime.reset();
            while (runtime.seconds() < seconds) {

            }
            stop();
        } else if (command.equals("Close") || command.equals("close")) {
            claw.setPosition(0);
            runtime.reset();
            while (runtime.seconds() < seconds) {

            }
            stop();
        }
        stop();
    }
    public void wait(double seconds) {
        runtime.reset();
        while (runtime.seconds() < seconds) {

        }
        stop();
    }

    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}