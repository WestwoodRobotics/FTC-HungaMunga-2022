package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class EleTest extends OpMode {

    public DcMotorEx elevator;



    @Override
    public void init() {
         elevator = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
    }

    @Override
    public void loop() {
        telemetry.addData("Value", elevator.getCurrentPosition());
    }
}
