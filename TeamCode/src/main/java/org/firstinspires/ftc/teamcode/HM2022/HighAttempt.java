
/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous(name = "HighAttempt")
public class HighAttempt extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    String parking = "middle";

    Servo claw = null;

    DcMotor viperSlide = null;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        claw = hardwareMap.get(Servo.class, "claw");

        viperSlide = hardwareMap.get(DcMotor.class, "viperSlide");

        viperSlide.setDirection(DcMotor.Direction.FORWARD);

        viperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperSlide.setMode(DcMotor.RunMode.RESET_ENCODERS);

        viperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory leftTrajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(36)
                .build();

        Trajectory rightTrajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(36)
                .build();

        Trajectory toHigh = drive.trajectoryBuilder(new Pose2d(36, -65.5), Math.toRadians(0))
                .splineTo(new Vector2d(-34, 1), Math.toRadians(0))
                .build();

        Trajectory toStack = drive.trajectoryBuilder(toHigh.end())
                .splineTo(new Vector2d(-60, -15), Math.toRadians(180))
                .build();

        Trajectory fromStackToHigh = drive.trajectoryBuilder(toStack.end())
                .splineTo(new Vector2d(-34, 1), Math.toRadians(0))
                .build();

        Trajectory actuallyToHigh = drive.trajectoryBuilder(fromStackToHigh.end())
                .forward(5)
                .build();

        Trajectory backFromActuallyToHigh = drive.trajectoryBuilder(actuallyToHigh.end())
                .back(5)
                .build();

        Trajectory backToStack = drive.trajectoryBuilder(backFromActuallyToHigh.end())
                .splineTo(new Vector2d(-60, -15), Math.toRadians(180))
                .build();



        Trajectory parkingPosition = drive.trajectoryBuilder(backFromActuallyToHigh.end())
                .splineTo(new Vector2d(-36, -36), Math.toRadians(90))
                .build();

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null || tagOfInterest.id == LEFT) {
            parking = "Left";
        } else if (tagOfInterest.id == MIDDLE) {
            parking = "Middle";
        } else {
            parking = "Right";
        }

        /* picking up pre-placed cone */
        claw.setPosition(1);
        viperSlide.setTargetPosition(60);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(1);
        drive.followTrajectory(toHigh);
        viperSlide.setPower(0);

        /* move to high goal and attempt to score */
        drive.followTrajectory(actuallyToHigh);
        viperSlide.setTargetPosition(3120);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(1);
        sleep(2800);
        viperSlide.setPower(0);
        drive.followTrajectory(actuallyToHigh);
        claw.setPosition(0.7);

        /* return to cone stack and pick up 5th cone */
        drive.followTrajectory(backFromActuallyToHigh);
        viperSlide.setTargetPosition(690);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(-1);
        sleep(1800);
        viperSlide.setPower(0);
        drive.followTrajectory(backToStack);
        viperSlide.setTargetPosition(640);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(-1);
        claw.setPosition(1);
        viperSlide.setTargetPosition(770);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(1);
        sleep(600);
        viperSlide.setPower(0);

        /* drive to high pole and attempt to score */
        drive.followTrajectory(fromStackToHigh);
        viperSlide.setTargetPosition(3120);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(1);
        sleep(3500);
        viperSlide.setPower(0);
        drive.followTrajectory(actuallyToHigh);
        claw.setPosition(0.7);

        /* return to cone stack and pick up 4th cone */
        drive.followTrajectory(backFromActuallyToHigh);
        viperSlide.setTargetPosition(500);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(-1);
        sleep(2000);
        viperSlide.setPower(0);
        drive.followTrajectory(backToStack);
        viperSlide.setTargetPosition(440);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(-1);
        claw.setPosition(1);
        viperSlide.setTargetPosition(600);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(1);
        sleep(600);
        viperSlide.setPower(0);

        /* drive to high pole and attempt to score */
        drive.followTrajectory(fromStackToHigh);
        viperSlide.setTargetPosition(3120);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(1);
        sleep(3500);
        viperSlide.setPower(0);
        drive.followTrajectory(actuallyToHigh);
        claw.setPosition(0.7);
        drive.followTrajectory(backFromActuallyToHigh);

        /* return to cone stack and pick up 3rd cone */
        drive.followTrajectory(backFromActuallyToHigh);
        viperSlide.setTargetPosition(400);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(-1);
        sleep(2000);
        viperSlide.setPower(0);
        drive.followTrajectory(backToStack);
        viperSlide.setTargetPosition(320);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(-1);
        claw.setPosition(1);
        viperSlide.setTargetPosition(450);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(1);
        sleep(600);
        viperSlide.setPower(0);

        /* drive to high pole and attempt to score */
        drive.followTrajectory(fromStackToHigh);
        viperSlide.setTargetPosition(3120);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(1);
        sleep(3500);
        viperSlide.setPower(0);
        drive.followTrajectory(actuallyToHigh);
        claw.setPosition(0.7);
        drive.followTrajectory(backFromActuallyToHigh);

        /* return to cone stack and pick up 2nd cone */
        drive.followTrajectory(backFromActuallyToHigh);
        viperSlide.setTargetPosition(250);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(-1);
        sleep(2000);
        viperSlide.setPower(0);
        drive.followTrajectory(backToStack);
        viperSlide.setTargetPosition(190);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(-1);
        claw.setPosition(1);
        viperSlide.setTargetPosition(300);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(1);
        sleep(600);
        viperSlide.setPower(0);

        /* drive to high pole and attempt to score */
        drive.followTrajectory(fromStackToHigh);
        viperSlide.setTargetPosition(3120);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(1);
        sleep(3500);
        viperSlide.setPower(0);
        drive.followTrajectory(actuallyToHigh);
        claw.setPosition(0.7);
        drive.followTrajectory(backFromActuallyToHigh);

        /* return to cone stack and pick up last cone */
        drive.followTrajectory(backFromActuallyToHigh);
        viperSlide.setTargetPosition(50);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(-1);
        sleep(2000);
        viperSlide.setPower(0);
        drive.followTrajectory(backToStack);
        viperSlide.setTargetPosition(10);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(-1);
        claw.setPosition(1);
        viperSlide.setTargetPosition(60);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(1);
        sleep(300);
        viperSlide.setPower(0);

        /* drive to high pole and attempt to score */
        drive.followTrajectory(fromStackToHigh);
        viperSlide.setTargetPosition(3120);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(1);
        sleep(3500);
        viperSlide.setPower(0);
        drive.followTrajectory(actuallyToHigh);
        claw.setPosition(0.7);
        drive.followTrajectory(backFromActuallyToHigh);

        /* return to center of parking positions */
        drive.followTrajectory(parkingPosition);
        if (parking == "Left") {
            drive.followTrajectory(leftTrajectory);
        } else if (parking == "Right") {
            drive.followTrajectory(rightTrajectory);
        }



        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}