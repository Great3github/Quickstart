package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous (name="Mattp2-1", group="Examples")
public class example3 extends OpMode {
    PathChain path;
//    HardwareMap hardwareMap;
    private PathChain builder;
    private Follower follower;
    @Override
    public void init() {


        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose(12.5, 85, Math.toRadians(0)).getAsFTCStandardCoordinates());
        // ^^ remember to set starting pose ^^
        path = new MattPath().GeneratedPath2();
        follower.followPath(path, true);
    }

    @Override
    public void loop() {
        follower.update();
    }
    public class MattPath {

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
                                    new Point(new Pose(12.5, 85, true)),
                                    new Point(new Pose(0.5, 60, true)),
                                    new Point(new Pose(48.5, 60, true)),
                                    new Point(new Pose(35, 85, true))
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();
            return builder;
        }
    }
}
