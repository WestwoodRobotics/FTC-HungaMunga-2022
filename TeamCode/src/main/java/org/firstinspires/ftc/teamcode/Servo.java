package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Servo", group ="Iterative Opmode")

public class ServoCode extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    Servo claw = null;

    @Override
    public void init() {
        claw = hardwareMap.get(Servo.class, "claw");

        claw.setDirection(Servo.setDirection.FORWARD);

        telemetry.addData("Status","initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double clawPos = 0;

        boolean clawOpen = gamepad1.dpad_left;
        boolean clawClose = gamepad1.dpad_right;

        if (clawOpen == true && clawClose == false && clawPos <= 1) {
            clawPos += 0.02;
        } else if (clawClose == true && clawOpen == false && clawPos >= -1) {
            clawPos -= 0.02;
        }

        claw.setPosition(clawPos);
    }
}
