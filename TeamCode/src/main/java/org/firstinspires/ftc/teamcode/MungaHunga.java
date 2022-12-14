package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name="Munga Hunga", group="Iterative Opmode")

public class MungaHunga extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor frontRight = null;
    DcMotor frontLeft = null;
    DcMotor backRight = null;
    DcMotor backLeft = null;
    DcMotor lift = null;

    @Override
    public void init() {
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        lift = hardwareMap.get(DcMotor.class, "lift");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        telemetry.addData("Status", "initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        boolean high = gamepad1.x;
        boolean medium = gamepad1.a;
        boolean low = gamepad1.b;
        boolean base = gamepad1.y;

        double liftPos = lift.getCurrentPosition();

        double frontRightPower = drive + strafe + turn;
        double frontLeftPower = drive - strafe - turn;
        double backRightPower = drive - strafe + turn;
        double backLeftPower = drive + strafe - turn;

        frontRight.setPower(frontRightPower);
        frontLeft.setPower(frontLeftPower);
        backRight.setPower(backRightPower);
        backLeft.setPower(backLeftPower);

//        if (high == true && medium == base == low == false) {
//            lift.setTargetPosition();
//            lift.setPower(1);
//        } else if (medium == true && high ==low == base == false) {
//            lift.setTargetPosition();
//            lift.setPower(1);
//        } else if (low == true && high == medium == base == false) {
//            lift.setTargetPosition();
//            lift.setPower(1);
//        } else if (base == true && high == medium == low == false) {
//            lift.setTargetPosition(2);
//        }

        telemetry.addData("Status", "Running");
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
