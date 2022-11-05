import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOpr", group = "Iterative Opmode")
public class TeleOpr extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Drivetrain Motors
    DcMotor frontRight = null;
    DcMotor frontLeft = null;
    DcMotor backRight = null;
    DcMotor backLeft = null;

    // Viper Motors
    DcMotor viperSlide = null;
    double vippow = 0;

    //Servo
    Servo claw = null;

    @Override
    public void init() {

        // Motor initialization
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

        //Viper Slide Motor
        viperSlide = hardwareMap.get(DcMotor.class, "viperSlide");

        viperSlide.setDirection(DcMotor.Direction.FORWARD);

        viperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperSlide.setMode(DcMotor.RunMode.RESET_ENCODERS);

        viperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Servo
        claw = hardwareMap.get(Servo.class, "claw");

        claw.setDirection(Servo.Direction.REVERSE);

        telemetry.addData("Status", "initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        // - - Gamepad Buttons - - //
        // Drive buttons (yes, buttons)
        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        // Viper buttons
        boolean high = gamepad1.y;
        boolean medium = gamepad1.x;
        boolean low = gamepad1.a;
        boolean base = gamepad1.b;

        boolean vipup = gamepad1.right_bumper;
        float vipdow = gamepad1.right_trigger;

        // Servo buttons
        boolean clawOpen = gamepad1.dpad_left;
        boolean clawClose = gamepad1.dpad_right;

        //Drive Power
        double frontRightPower = drive - strafe + turn;
        double frontLeftPower = drive + strafe - turn;
        double backRightPower = drive + strafe + turn;
        double backLeftPower = drive - strafe - turn;

        // Viper slide variables
        double height = viperSlide.getCurrentPosition();
        double targetPos = 0;

        // Servo Position variable
        double clawPos = 0;


        // Viper Slide button functions
        if (high && !(medium == low == base) ) {
            targetPos = 1800;
        } else if (medium && !(high == low == base) ) {
            targetPos = 1000;
        } else if (low && !(high == medium == base) ) {
            targetPos = 200;
        } else if (base && !(high == medium == low) ) {
            targetPos = 10;
        }

        if (vipup && vipdow == 0 && height < 2730) {
            vippow = 1;
        } else if (vipdow > 0 && !vipup && height > 0) {
            vippow = -1;
        }

        // Set Power based on Current position V.S. targEt position
        if (height < targetPos) {
            vippow = 1;
        } else if (height > targetPos) {
            vippow = -1;
        } else if (height == targetPos) {
            vippow = 0;
        }

        // Servo code
        if (clawOpen && !clawClose && clawPos < 1) {
            clawPos += 0.02;
        } else if (clawClose && !clawOpen && clawPos > -1) {
            clawPos -= 0.02;
        }

        // Drive power
        frontRight.setPower(frontRightPower);
        frontLeft.setPower(frontLeftPower);
        backRight.setPower(backRightPower);
        backLeft.setPower(backLeftPower);

        // Viper Slide directions
        viperSlide.setTargetPosition((int)targetPos);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(vippow);

        //Servo movement
        claw.setPosition(clawPos);

        telemetry.addData("Status", "Running");
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
