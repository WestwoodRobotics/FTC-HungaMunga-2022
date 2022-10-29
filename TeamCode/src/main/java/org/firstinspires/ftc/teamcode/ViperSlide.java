package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Teleop (name="Viper Slide", group="Iterative Opmode")

public class ViperSlide extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor viperSlide = null;

    @Override
    public void init() {
        viperSlide = hardwareMap.get(DcMotor.class, "viperSlide");

        viperSlide.getDirection(DcMotor.Direction.REVERSE);

        viperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        viperSlide.setMode(DcMotor.RunMode.RESET_ENCODERS);

        viperSlide.setZeroPowerBehavior(DcMotor.setZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        boolean high = gamepad1.x;
        boolean medium = gamepad1.a;
        boolean low = gamepad1.b;
        boolean base = gamepad1.y;
        boolean vipup = gamepad1.right_bumper;
        boolean vipdow = gamepad1.right_trigger;
        //make variable for motor position ;-;

        double vippow = 0;
        double vippos = viperSlide.getCurrentPosition();

        if (high == true && medium == low == base == false && vippos != ) {
            // need to put in encoder value for all 3
            viperSlide.setTargetPosition();
            vippow = 1;
        } else if (medium == true && high == low == base == false && vippos != ) {
            viperSlide.setTargetPosition();
            vippow = 1;
        } else if (low == true && high == medium == base == false && vippos !=) {
            viperSlide.setTargetPosition();
            vippow = 1;
        } else if (base == true && high == medium == low == false && vippos !=) {
            viperSlide.setTargetPosition(0);
            vippow = 1;
        }

        if (vipup == true && vipdow == false) {

        }

        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        viperSlide.setPower(vippow);

        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}
