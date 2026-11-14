package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;
import static com.pedropathing.ivy.groups.Groups.parallel;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static com.pedropathing.ivy.groups.Groups.race;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous
public class Blue_Up extends LinearOpMode {

    private Follower follower;
    private AutonomousSubsystem autoSub;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(86, 133, 0);
    private final Pose point1 = poseFactory.of(133, 133, 0);
    private final Pose point2 = poseFactory.of(115, 121, 0);
    private final Pose point3 = poseFactory.of(115, 30, 90);
    private final Pose point4Start = poseFactory.of(115, 30, 90);
    private final Pose point4 = poseFactory.of(95, 20, 110);
    private final Pose point5Start = poseFactory.of(95, 20, 110);
    private final Pose point5 = poseFactory.of(95, 20.1, 270);
    private final Pose point6 = poseFactory.of(95, 13, -90);
    private final Pose point7Start = poseFactory.of(95, 13, 270);
    private final Pose point7 = poseFactory.of(90, 15, 100);
    private final Pose point8 = poseFactory.of(130, 24, 13);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                waitMs(300),
                race(follow(follower, path1()), waitMs(5000)),

//                parallel(setShooter(10,10)), follow(follower, path2()),

                follow(follower, path2()),
                setShooter(10,10),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6()),
                follow(follower, path7()),
                follow(follower, path8())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        autoSub = new AutonomousSubsystem(hardwareMap);

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.line(start, point1).tangent()
//                .with(Constants.foresightConfig.maxPathSpeed.at(0.5)); // 30 inches per second cap

                .with(Constants.foresightConfig.maxVelocityConstraint.at(30.0)); // 30 inches per second cap

    }


    public Path path2() {
        return Paths.line(point1, point2).constant(point2);
    }

    public Path path3() {
        return Paths.line(point2, point3).constant(point3);
    }

    public Path path4() {
        return Paths.line(point4Start, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point5, point6).tangent();
    }

    public Path path7() {
        return Paths.line(point7Start, point7).linear(point7Start, point7);
    }

    public Path path8() {
        return Paths.line(point7, point8).tangent();
    }
    public Command setShooter(double pVel, double nVel){
        return instant(() -> autoSub.setShooterVelocities(pVel, nVel));
    }

}
