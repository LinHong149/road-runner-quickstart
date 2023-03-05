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


// -----------------------------------------------------Example Code
//package org.firstinspires.ftc.teamcode.drive.auton;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.openftc.apriltag.AprilTagDetection;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//
//import java.util.ArrayList;


// ------------------------------------------------------------Blue Right Auton Code
package org.firstinspires.ftc.teamcode.drive.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.ServoImpl;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;
import java.util.Vector;


@Autonomous
public class autontest extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // -------------------------------------------Blue Right Auton Code
    public DcMotorEx lift;
    private Servo ocServo;

    double slowervel = 15;
    int MotorTimer = 1;
    int MotorWait = 2;
    int TJunction = 3222; //was 3250
    int Firstcone = 520; //setting the right value for these +20
    int Secondcone = 440; //357 didnt add 20 yet
    int Thirdcone = 340; //243
    int Fourthcone = 320; //110
    int Fifthcone = 260; //-39


    // UNITS ARE METERS
    double tagsize = 0.166;


    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        ocServo = hardwareMap.get(Servo.class, "openClose");
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
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
        while (!isStarted() && !isStopRequested())
        {
            ocServo.setPosition(0.8);
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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

//        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Pose2d startPose = new Pose2d(-35, 64, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        /* Actually do something useful */
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        if(tagOfInterest == null || tagOfInterest.id == LEFT) {
            Trajectory T1 = drive.trajectoryBuilder(startPose) //getting to the tallest junction with pre-loaded cone
                    // Claw is closed
                    .addTemporalMarker(0.1, ()->{ // Goes to Tall
                        lift.setTargetPosition(TJunction);
                        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })

                    .lineToLinearHeading(new Pose2d(-35, 20, Math.toRadians(270)))
                    .splineToSplineHeading(new Pose2d(-29, 6, Math.toRadians(320)), Math.toRadians(320))
                    .build();


            Trajectory T2 = drive.trajectoryBuilder(T1.end()) //going to the cones
                    .addTemporalMarker(0.01, ()-> { // Opens
                        ocServo.setPosition(1);
                    })

                    .addTemporalMarker(0.5, ()->{ // Goes to First
                        lift.setTargetPosition(Firstcone);
                        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })

                    .addTemporalMarker(2.5, ()-> { // Closes
                        ocServo.setPosition(0.8);

                    })

                    .addTemporalMarker(3, ()-> { // Goes to Tall
                        lift.setTargetPosition(TJunction);
                        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })

                    .lineToSplineHeading(new Pose2d(-38, 12, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-60,12), Math.toRadians(180))                   //.lineToLinearHeading(new Pose2d(-60, 12), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)) )
                    .build();


            Trajectory T3 = drive.trajectoryBuilder(T2.end()) //going back to the tallest junction


                    .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
                    .splineToSplineHeading(new Pose2d(-27, 5, Math.toRadians(320)), Math.toRadians(320))
                    .build();


            Trajectory T4 = drive.trajectoryBuilder(T3.end()) //going to the cones Second cone
                    .addTemporalMarker(0.01, ()-> { // Opens
                        ocServo.setPosition(1);
                    })

                    .addTemporalMarker(0.5, ()->{ // Goes to Second
                        lift.setTargetPosition(Secondcone);
                        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })

                    .addTemporalMarker(2.5, ()-> { // Closes
                        ocServo.setPosition(0.8);

                    })

                    .addTemporalMarker(3, ()-> { // Goes to Tall
                        lift.setTargetPosition(TJunction);
                        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })

                    .lineToSplineHeading(new Pose2d(-38, 12, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-60,12), Math.toRadians(180))                   //.lineToLinearHeading(new Pose2d(-60, 12), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)) )
                    .build();

            Trajectory T5 = drive.trajectoryBuilder(T4.end()) //going back to the tallest junction

                    .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
                    .splineToSplineHeading(new Pose2d(-27, 5, Math.toRadians(320)), Math.toRadians(320))
                    .build();

            Trajectory T6 = drive.trajectoryBuilder(T5.end()) //going to the cones third cone

                    .addTemporalMarker(0.01, ()-> { // Opens
                        ocServo.setPosition(1);
                    })
                    .addTemporalMarker(0.5, ()->{ // Goes Third
                        lift.setTargetPosition(Thirdcone);
                        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })

                    .addTemporalMarker(2.1, ()-> { // Closes
                        ocServo.setPosition(0.8);

                    })

                    .addTemporalMarker(3, ()-> { // Goes to Tall
                        lift.setTargetPosition(TJunction);
                        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })

                    .lineToSplineHeading(new Pose2d(-38, 12, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-60,12), Math.toRadians(180))                   //.lineToLinearHeading(new Pose2d(-60, 12), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)) )
                    .build();

            Trajectory T7 = drive.trajectoryBuilder(T6.end()) //going back to the tallest junction

                    .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
                    .splineToSplineHeading(new Pose2d(-27, 5, Math.toRadians(320)), Math.toRadians(320))
                    .build();

            Trajectory Parking = drive.trajectoryBuilder(T7.end()) //parking left

                    .addTemporalMarker(0.2, ()->{ // Goes to 0
                        lift.setTargetPosition(0);
                        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })


                    .addTemporalMarker(0.01, ()-> { // Opens
                        ocServo.setPosition(1);
                    })

                    .lineToSplineHeading(new Pose2d(-35,17, Math.toRadians(270)))


                    .build();
            Trajectory Parking1 = drive.trajectoryBuilder(T7.end()) //parking left

                    .addTemporalMarker(0.2, ()->{ // Goes to 0
                        lift.setTargetPosition(0);
                        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })


                    .addTemporalMarker(0.01, ()-> { // Opens
                        ocServo.setPosition(1);
                    })


                    .lineToLinearHeading(new Pose2d(-12, 17, Math.toRadians(0)))
                    .build();





            if(isStopRequested()) return;


            drive.followTrajectory(T1); // goes to tallest junction
            drive.followTrajectory(T2); // First cone
            drive.followTrajectory(T3); // TJ
            drive.followTrajectory(T4); // SC
            drive.followTrajectory(T5); // TJ
            drive.followTrajectory(T6); // TC
            drive.followTrajectory(T7); // TJ
            drive.followTrajectory(Parking); // Parking
            drive.followTrajectory(Parking1);


        }else if(tagOfInterest.id == MIDDLE) {

            Trajectory T1 = drive.trajectoryBuilder(startPose) //getting to the tallest junction with pre-loaded cone
                    // Claw is closed
                    .addTemporalMarker(0.1, ()->{ // Goes to Tall
                        lift.setTargetPosition(TJunction);
                        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })

                    .lineToLinearHeading(new Pose2d(-35, 20, Math.toRadians(270)))
                    .splineToSplineHeading(new Pose2d(-28, 6, Math.toRadians(320)), Math.toRadians(320))
                    .build();


            Trajectory T2 = drive.trajectoryBuilder(T1.end()) //going to the cones
                    .addTemporalMarker(0.01, ()-> { // Opens
                        ocServo.setPosition(1);
                    })

                    .addTemporalMarker(0.5, ()->{ // Goes to First
                        lift.setTargetPosition(Firstcone);
                        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })

                    .addTemporalMarker(2.5, ()-> { // Closes
                        ocServo.setPosition(0.8);

                    })

                    .addTemporalMarker(3, ()-> { // Goes to Tall
                        lift.setTargetPosition(TJunction);
                        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })

                    .lineToSplineHeading(new Pose2d(-38, 12, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-60,12), Math.toRadians(180))                   //.lineToLinearHeading(new Pose2d(-60, 12), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)) )
                    .build();


            Trajectory T3 = drive.trajectoryBuilder(T2.end()) //going back to the tallest junction


                    .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
                    .splineToSplineHeading(new Pose2d(-27, 5, Math.toRadians(320)), Math.toRadians(320))
                    .build();


            Trajectory T4 = drive.trajectoryBuilder(T3.end()) //going to the cones Second cone
                    .addTemporalMarker(0.01, ()-> { // Opens
                        ocServo.setPosition(1);
                    })

                    .addTemporalMarker(0.5, ()->{ // Goes to Second
                        lift.setTargetPosition(Secondcone);
                        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })

                    .addTemporalMarker(2.5, ()-> { // Closes
                        ocServo.setPosition(0.8);

                    })

                    .addTemporalMarker(3, ()-> { // Goes to Tall
                        lift.setTargetPosition(TJunction);
                        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })

                    .lineToSplineHeading(new Pose2d(-38, 12, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-60,12), Math.toRadians(180))                   //.lineToLinearHeading(new Pose2d(-60, 12), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)) )
                    .build();

            Trajectory T5 = drive.trajectoryBuilder(T4.end()) //going back to the tallest junction

                    .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
                    .splineToSplineHeading(new Pose2d(-27, 5, Math.toRadians(320)), Math.toRadians(320))
                    .build();

            Trajectory T6 = drive.trajectoryBuilder(T5.end()) //going to the cones third cone

                    .addTemporalMarker(0.01, ()-> { // Opens
                        ocServo.setPosition(1);
                    })
                    .addTemporalMarker(0.5, ()->{ // Goes Third
                        lift.setTargetPosition(Thirdcone);
                        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })

                    .addTemporalMarker(2.6, ()-> { // Closes
                        ocServo.setPosition(0.8);

                    })

                    .addTemporalMarker(3, ()-> { // Goes to Tall
                        lift.setTargetPosition(TJunction);
                        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })

                    .lineToSplineHeading(new Pose2d(-38, 12, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-60,12), Math.toRadians(180))                   //.lineToLinearHeading(new Pose2d(-60, 12), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)) )
                    .build();

            Trajectory T7 = drive.trajectoryBuilder(T6.end()) //going back to the tallest junction

                    .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
                    .splineToSplineHeading(new Pose2d(-27, 5, Math.toRadians(320)), Math.toRadians(320))
                    .build();

            Trajectory Parking = drive.trajectoryBuilder(T7.end()) //parking mid

                    .addTemporalMarker(0.2, ()->{ // Goes to 0
                        lift.setTargetPosition(0);
                        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })

                    .addTemporalMarker(0.01, ()-> { // Opens
                        ocServo.setPosition(1);
                    })

                    .lineToSplineHeading(new Pose2d(-34,12))
                    .build();






            if(isStopRequested()) return;


            drive.followTrajectory(T1); // goes to tallest junction
            drive.followTrajectory(T2); // First cone
            drive.followTrajectory(T3); // TJ
            drive.followTrajectory(T4); // SC
            drive.followTrajectory(T5); // TJ
            drive.followTrajectory(T6); // TC
            drive.followTrajectory(T7); // TJ
            drive.followTrajectory(Parking); // Parking



        }
        else {

            Trajectory T1 = drive.trajectoryBuilder(startPose) //getting to the tallest junction with pre-loaded cone
                    // Claw is closed
                    .addTemporalMarker(0.1, ()->{ // Goes to Tall
                        lift.setTargetPosition(TJunction);
                        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })

                    .lineToLinearHeading(new Pose2d(-35, 20, Math.toRadians(270)))
                    .splineToSplineHeading(new Pose2d(-28, 6, Math.toRadians(320)), Math.toRadians(320))
                    .build();


            Trajectory T2 = drive.trajectoryBuilder(T1.end()) //going to the cones
                    .addTemporalMarker(0.01, ()-> { // Opens
                        ocServo.setPosition(1);
                    })

                    .addTemporalMarker(0.5, ()->{ // Goes to First
                        lift.setTargetPosition(Firstcone);
                        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })

                    .addTemporalMarker(2.5, ()-> { // Closes
                        ocServo.setPosition(0.8);

                    })

                    .addTemporalMarker(3, ()-> { // Goes to Tall
                        lift.setTargetPosition(TJunction);
                        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })

                    .lineToSplineHeading(new Pose2d(-38, 12, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-60,12), Math.toRadians(180))                   //.lineToLinearHeading(new Pose2d(-60, 12), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)) )
                    .build();


            Trajectory T3 = drive.trajectoryBuilder(T2.end()) //going back to the tallest junction


                    .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
                    .splineToSplineHeading(new Pose2d(-27, 5, Math.toRadians(320)), Math.toRadians(320))
                    .build();


            Trajectory T4 = drive.trajectoryBuilder(T3.end()) //going to the cones Second cone
                    .addTemporalMarker(0.01, ()-> { // Opens
                        ocServo.setPosition(1);
                    })

                    .addTemporalMarker(0.5, ()->{ // Goes to Second
                        lift.setTargetPosition(Secondcone);
                        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })

                    .addTemporalMarker(2.5, ()-> { // Closes
                        ocServo.setPosition(0.8);

                    })

                    .addTemporalMarker(3, ()-> { // Goes to Tall
                        lift.setTargetPosition(TJunction);
                        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })

                    .lineToSplineHeading(new Pose2d(-38, 12, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-60,12), Math.toRadians(180))                   //.lineToLinearHeading(new Pose2d(-60, 12), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)) )
                    .build();

            Trajectory T5 = drive.trajectoryBuilder(T4.end()) //going back to the tallest junction

                    .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
                    .splineToSplineHeading(new Pose2d(-27, 5, Math.toRadians(320)), Math.toRadians(320))
                    .build();

            Trajectory T6 = drive.trajectoryBuilder(T5.end()) //going to the cones third cone

                    .addTemporalMarker(0.01, ()-> { // Opens
                        ocServo.setPosition(1);
                    })
                    .addTemporalMarker(0.5, ()->{ // Goes Third
                        lift.setTargetPosition(Thirdcone);
                        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })

                    .addTemporalMarker(2.1, ()-> { // Closes
                        ocServo.setPosition(0.8);

                    })

                    .addTemporalMarker(3, ()-> { // Goes to Tall
                        lift.setTargetPosition(TJunction);
                        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })

                    .lineToSplineHeading(new Pose2d(-38, 12, Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(-60,12), Math.toRadians(180))                   //.lineToLinearHeading(new Pose2d(-60, 12), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)) )
                    .build();

            Trajectory T7 = drive.trajectoryBuilder(T6.end()) //going back to the tallest junction

                    .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
                    .splineToSplineHeading(new Pose2d(-27, 5, Math.toRadians(320)), Math.toRadians(320))
                    .build();


            Trajectory Parking = drive.trajectoryBuilder(T7.end()) //parking right

                    .addTemporalMarker(0.2, ()->{ // Goes to 0
                        lift.setTargetPosition(0);
                        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })

                    .addTemporalMarker(0.01, ()-> { // Opens
                        ocServo.setPosition(1);
                    })

                    .lineToSplineHeading(new Pose2d(-35,10))

//                    .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
//                    .splineToSplineHeading(new Pose2d(-30, 8, Math.toRadians(320)), Math.toRadians(320))
                    .build();
            Trajectory Parking1 = drive.trajectoryBuilder(T7.end()) //parking left

                    .addTemporalMarker(0.2, ()->{ // Goes to 0
                        lift.setTargetPosition(0);
                        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    })


                    .addTemporalMarker(0.01, ()-> { // Opens
                        ocServo.setPosition(1);
                    })


                    .lineToConstantHeading(new Vector2d(-58,12))
                    .build();




            if(isStopRequested()) return;


            drive.followTrajectory(T1); // goes to tallest junction
            drive.followTrajectory(T2); // First cone
            drive.followTrajectory(T3); // TJ
            drive.followTrajectory(T4); // SC
            drive.followTrajectory(T5); // TJ
            drive.followTrajectory(T6); // TC
            drive.followTrajectory(T7); // TJ
            drive.followTrajectory(Parking); // Parking
            drive.followTrajectory(Parking1); // Parking


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