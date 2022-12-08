package org.firstinspires.ftc.teamcode.HM2022;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "HMTeleOp",
        group = "Iterative Opmode")
public class HMTeleOp extends OpMode {

    // Run time instantiation
    private ElapsedTime runtime = new ElapsedTime();

    // Motor Initialization & Instantiation to null
    DcMotor frontLeft,
            frontRight,
            backLeft,
            backRight = null;

    // Viper Motors
    DcMotor viperSlide = null;
    double vippow = 0;
    double targetPos = 0;

    // Sero Initialization & Instantiation to null
    Servo claw = null;

    // Gamepad Connections

    float drive = 0;
    float strafe = 0;
    float turn = 0;
    double modifier = 0.3;
    double clawPos = 1;

    // Motor Power
    double frontRightPower;
    double frontLeftPower;
    double backRightPower;
    double backLeftPower;

    @Override
    public void init() {

        // Motor instantiation to hardwareMap

        //Drive Motors

        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Lift motor to hardwareMap
        //Viper Slide Motor
        viperSlide = hardwareMap.get(DcMotor.class, "viperSlide");

        viperSlide.setDirection(DcMotor.Direction.FORWARD);

        viperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperSlide.setMode(DcMotor.RunMode.RESET_ENCODERS);

        viperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servo to hardwareMap
        claw = hardwareMap.get(Servo.class, "claw");

    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override

    public void loop() {

        drive = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;

        // Viper buttons
        boolean high = gamepad1.y;
        boolean medium = gamepad1.x;
        boolean low = gamepad1.a;
        boolean base = gamepad1.b;

        boolean stack5 = gamepad1.dpad_up;
        boolean stack4 = gamepad1.dpad_left;
        boolean stack3 = gamepad1.dpad_right;
        boolean stack2 = gamepad1.dpad_down;

        frontRightPower = drive + strafe + turn;
        frontLeftPower = drive - strafe - turn;
        backRightPower = drive - strafe + turn;
        backLeftPower = drive + strafe - turn;

        // Viper slide variables
        double height = viperSlide.getCurrentPosition();

        // Viper Slide button functions
        if (high && !(medium == low == base)) {
            targetPos = 3119; // 3080
        } else if (medium && !(high == low == base)) {
            targetPos = 2167; // 2325
        } else if (low && !(high == medium == base)) {
            targetPos = 1324;
        } else if (base && !(high == medium == low)) {
            targetPos = 60;
            claw.setPosition(0.32);
        } else if (gamepad1.dpad_up) {
            targetPos = 646;
        } else if (gamepad1.dpad_left) {
            targetPos = 448;
        } else if (gamepad1.dpad_right) {
            targetPos = 323;
        } else if (gamepad1.dpad_down) {
            targetPos = 200;
        }

        if (height > 3000) {
            modifier = 0.3;
        } else {
            modifier = 0.5;
        }

        // Set Power based on Current position V.S. targEt position
        if (height < targetPos) {
            vippow = 1.2;
        } else if (height > targetPos) {
            vippow = -1.2;
        } else if (height == targetPos) {
            vippow = 0;
        }

        if (gamepad1.right_trigger > 0) {
            targetPos += 23;
        } else if (gamepad1.left_trigger > 0 && targetPos > 0) {
            targetPos -= 23;
        }

        frontRight.setPower(frontRightPower * modifier);
        frontLeft.setPower(frontLeftPower * modifier);
        backRight.setPower(backRightPower * modifier);
        backLeft.setPower(backLeftPower * modifier);

        if (gamepad1.left_bumper) {
            clawPos = 0.26;
        } else {
            clawPos = 1;
        }

        // Viper Slide directions
        viperSlide.setTargetPosition((int) targetPos);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(vippow);

        claw.setPosition(clawPos);

        telemetry.addData("Value", height);
        telemetry.addData("Status", "Running");
        telemetry.update();
    }

    @Override
    public void stop() {}
}