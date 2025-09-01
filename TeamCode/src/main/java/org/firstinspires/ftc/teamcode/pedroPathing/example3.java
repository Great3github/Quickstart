package org.firstinspires.ftc.teamcode.pedroPathing;

import static java.lang.Thread.sleep;

import android.provider.Settings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.constants.OTOSConstants;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous (name="Mattp2-1", group="Examples")
public class example3 extends OpMode {
    PathChain path;
    int pathnum = 1;
    SparkFunOTOS.Pose2D newpose = OTOSConstants.offset;
    long time = System.currentTimeMillis();
    boolean hasDoneServothing = false;
    boolean var2 = false;
    boolean path2started = false;
//    HardwareMap hardwareMap;
    private PathChain builder;
    private PathChain builder2;
    private Follower follower;
    private Telemetry telemetryA;
    CRServo servo;
    @Override
    public void init() {

        servo = hardwareMap.get(CRServo.class, "servo1");
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose(12, 60, Math.toRadians(180)).getAsFTCStandardCoordinates());
        // ^^ remember to set starting pose ^^
        //path = new MattPath().GeneratedPath3();
        new MattPath().startNewPath(pathnum);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addData("XValue", newpose.x);
        telemetryA.addData("YValue", newpose.y);
        telemetryA.update();
    }
    public void start() {
        //follower.startTeleopDrive();
    }
    @Override
    public void loop() {
        follower.update();
        follower.telemetryDebug(telemetryA);
        //follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        if (pathnum == 2 && !hasDoneServothing) {

            if (System.currentTimeMillis() >= time + 2000) {
                servo.setPower(0);
                hasDoneServothing = true;
            }
        }
        if (!follower.isBusy() && !path2started) {
            pathnum +=1;
            new MattPath().startNewPath(pathnum);
            path2started = true;
            if (!var2) {time = System.currentTimeMillis(); var2= true;}
            servo.setPower(-1);
        } //else if (!follower.isBusy() && !path2started)
    }
    public class MattPath {
        public void startNewPath(int pathnum) {
            if (pathnum == 1) {
                follower.followPath(new MattPath().GeneratedPath4(), 0.3, true);
            } else if (pathnum == 2) {


                follower.followPath(new MattPath().GeneratedPath4_2(), 0.3, true);



            } else if (pathnum == 3) {
                servo.setPower(0);

            }
        }
        public PathChain followerPath() {
//            PathBuilder builder = new PathBuilder();
            builder = follower.pathBuilder()


                    .addPath(
                            // Line 1
                            new BezierCurve(
                                    new Point(10.000, 60.000, Point.CARTESIAN),
                                    new Point(37.000, 24.000, Point.CARTESIAN),
                                    new Point(84.000, 24.000, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(
                            // Line 2
                            new BezierCurve(
                                    new Point(84.000, 24.000, Point.CARTESIAN),
                                    new Point(143.000, 49.000, Point.CARTESIAN),
                                    new Point(110.000, 71.913, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                    .build();

            return builder;
        }
        public PathChain GeneratedPath2() {
            builder = follower.pathBuilder()


                    .addPath(
                            // Line 1
                            new BezierCurve(
                                    new Point(new Pose(12.5, 85, 0).getAsFTCStandardCoordinates()),
                                    new Point(new Pose(0.5, 60).getAsFTCStandardCoordinates()),
                                    new Point(new Pose(48.5, 60).getAsFTCStandardCoordinates()),
                                    new Point(new Pose(35, 85, 90).getAsFTCStandardCoordinates())
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();
            
            return builder;
        }
        public PathChain GeneratedPath3() {
            builder = follower.pathBuilder()


                    .addPath(
                            // Line 1
                            new BezierCurve(
                                    new Point(new Pose(9.757, 84.983, 0).getAsFTCStandardCoordinates()),
                                    new Point(new Pose(9.863, 19.009, 0).getAsFTCStandardCoordinates()),
                                    new Point(new Pose(76.214, 13.270, 0).getAsFTCStandardCoordinates())
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            // Line 2
                            new BezierCurve(
                                    new Point(new Pose(76.214, 13.270, 0).getAsFTCStandardCoordinates()),
                                    new Point(new Pose(143.283, 13.629, 0).getAsFTCStandardCoordinates()),
                                    new Point(new Pose(119.791, 75.318, 0).getAsFTCStandardCoordinates())
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            // Line 3
                            new BezierCurve(
                                    new Point(new Pose(119.791, 75.318, 0).getAsFTCStandardCoordinates()),
                                    new Point(new Pose(138.441, 132.702, 0).getAsFTCStandardCoordinates()),
                                    new Point(new Pose(72.807, 126.964, 0).getAsFTCStandardCoordinates())
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            // Line 4
                            new BezierCurve(
                                    new Point(new Pose(72.807, 126.964, 0).getAsFTCStandardCoordinates()),
                                    new Point(new Pose(10.042, 134.496, 0).getAsFTCStandardCoordinates()),
                                    new Point(new Pose(12.912, 88.767, 0).getAsFTCStandardCoordinates())
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
            return builder;
        }
        public PathChain GeneratedPath4() {
            builder = follower.pathBuilder()


                    .addPath(
                            // Line 1
                            new BezierCurve(
                                    new Point(new Pose(12.000, 60.000, 180).getAsFTCStandardCoordinates()),
                                    new Point(new Pose(12.000, 85.000, 180).getAsFTCStandardCoordinates()),
                                    new Point(new Pose(36.000, 83.500, 180).getAsFTCStandardCoordinates())
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            return builder;
        }
        public PathChain GeneratedPath4_2() {
            return follower.pathBuilder()
                    .addPath(
                            // Line 2
                            new BezierCurve(
                                    new Point(new Pose(36.000, 83.500, 180).getAsFTCStandardCoordinates()),
                                    new Point(new Pose(12.000, 85.000, 180).getAsFTCStandardCoordinates()),
                                    new Point(new Pose(12.000, 107.500, 180).getAsFTCStandardCoordinates())
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

        }
    }
}
