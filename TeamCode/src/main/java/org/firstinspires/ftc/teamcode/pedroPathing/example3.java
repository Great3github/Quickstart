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

@Autonomous (name="Mattp2", group="Examples")
public class example3 extends OpMode {
    PathChain path = new MattPath().followerPath();
    HardwareMap hardwareMap;
    Follower follower;
    @Override
    public void init() {


        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose(10, 60, Math.toRadians(0)).getAsFTCStandardCoordinates());
        follower.followPath(path, true);
    }

    @Override
    public void loop() {
        follower.update();
    }
    public static class MattPath {

        public PathChain followerPath() {
            PathBuilder builder = new PathBuilder();

            builder
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
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180));

            return builder.build();
        }
    }
}
