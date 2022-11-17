package org.firstinspires.ftc.teamcode.HM2022;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
@Disabled
public class HMAuton extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    // Motor initialization
    DcMotor frontLeft,
            frontRight,
            backLeft,
            backRight = null;

    // Motor Power initialization
    double  frontRightPower,
            frontLeftPower,
            backRightPower,
            backLeftPower;

        public void runOpMode() throws InterruptedException {
            telemetry.addData("Status", "Initialized");
            telemetry.update();

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

            waitForStart();
            runtime.reset();
            AutonFunction functions = new AutonFunction(frontLeft, frontRight, backLeft, backRight, runtime);
            while (opModeIsActive()) {
                functions.turnLeft(90);
                functions.goForward(36);
                functions.turnRight(90);
            }

        }
    }
