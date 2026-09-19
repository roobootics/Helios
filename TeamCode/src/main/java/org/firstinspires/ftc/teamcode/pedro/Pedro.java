package org.firstinspires.ftc.teamcode.pedro;

import static org.firstinspires.ftc.teamcode.base.Components.timer;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.api.Paths;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.base.Commands;
import org.firstinspires.ftc.teamcode.base.Components;

import java.util.function.Supplier;

public abstract class Pedro {
    public static Follower follower;
    public static void createFollower(Pose startingPose){
        follower=Constants.create(Components.getHardwareMap());
        follower.setPose(startingPose);
    }
    public static Commands.RunResettingLoop updateCommand(){
        return new Commands.RunResettingLoop(new Commands.InstantCommand(follower::update));
    }
    public static class PedroCommand extends Commands.PathCommand<Path>{
        private double startTime;
        private double timeout = Double.POSITIVE_INFINITY;
        public PedroCommand(Supplier<Path> buildPath) {
            super(buildPath);
        }
        public PedroCommand setTimeout(double timeout){this.timeout=timeout; return this;}
        @Override
        public boolean followPath(){
            if (isStart()){
                startTime = timer.time();
                follower.follow(getPath());
                return true;
            }
            return follower.isBusy() && (timer.time()-startTime<timeout);
        }
        @Override
        public void stopProcedure(){
            follower.stop();
        }
    }
    public static class PedroLinearCommand extends PedroCommand{
        public PedroLinearCommand(Pose pose){ //Goes to the position indicated by the inputs
            super(
                    ()-> Paths.line(follower.pose(),pose)
                            .linear(follower.pose().heading(),pose.heading())
            );
        }
    }
    public static class PedroLinearChainCommand extends PedroCommand{
        public PedroLinearChainCommand(Pose...poses){ //In a path chain, goes to all positions provided one at a time
            super(
                ()->{
                    Path[] paths = new Path[poses.length];
                    for (int i=0;i<poses.length;i++) {
                        paths[i] = Paths.line(follower.pose(), poses[i]);
                        if (i==0){
                            paths[i].linear(follower.pose().heading(), poses[i].heading());}
                        else{
                            paths[i].linear(poses[i-1].heading(), poses[i].heading());}
                    }
                    return Paths.path(paths);
                }
            );
        }
    }
    public static class PedroLinearTransformCommand extends PedroCommand{
        public PedroLinearTransformCommand(double x, double y, double heading, boolean holdEnd){ //Transforms from current position by the given inputs
            super(
                    ()-> Paths.line(follower.pose(),new Pose(follower.pose().x()+x,follower.pose().y()+y))
                            .linear(follower.pose().heading(),follower.pose().heading()+heading)
            );
        }
    }
    public static class PedroCurveCommand extends PedroCommand{
        public PedroCurveCommand(double heading, Pose...poses) {
            super(
                    ()-> Paths.curve(poses)
                            .linear(follower.pose().heading(),heading)
            );
        }
    }
    public static class PedroInstantCommand extends Commands.InstantCommand { //This action ends instantly and does not wait for the follower to reach its goal
        public PedroInstantCommand(Supplier<Path> buildPath) {
            super(()->follower.follow(buildPath.get()));
        }
    }
    public static class PedroInstantLinearCommand extends PedroInstantCommand{
        public PedroInstantLinearCommand(Pose pose, boolean holdEnd) {
            super(
                    ()-> Paths.line(follower.pose(),pose)
                    .linear(follower.pose().heading(),pose.heading())
            );
        }
    }
    public static class PedroInstantLinearChainCommand extends PedroInstantCommand{
        public PedroInstantLinearChainCommand(boolean holdEnd, Pose...poses){ //In a path chain, goes to all positions provided one at a time
            super(
                    ()->{
                        Path[] paths = new Path[poses.length];
                        for (int i=0;i<poses.length;i++) {
                            paths[i] = Paths.line(follower.pose(), poses[i]);
                            if (i==0){
                                paths[i].linear(follower.pose().heading(), poses[i].heading());}
                            else{
                                paths[i].linear(poses[i-1].heading(), poses[i].heading());}
                        }
                        return Paths.path(paths);
                    }
            );
        }
    }
    public static class PedroInstantLinearTransformCommand extends PedroInstantCommand{
        public PedroInstantLinearTransformCommand(boolean holdEnd, double x, double y, double heading) {
            super(
                    ()-> Paths.line(follower.pose(),new Pose(follower.pose().x()+x,follower.pose().y()+y))
                            .linear(follower.pose().heading(),follower.pose().heading()+heading)
            );
        }
    }
    public static class PedroInstantCurveCommand extends PedroInstantCommand{
        public PedroInstantCurveCommand(double tension, boolean holdEnd, double heading, Pose...poses) {
            super(
                    ()-> Paths.curve(poses)
                            .linear(follower.pose().heading(),heading)
            );
        }
    }
}
