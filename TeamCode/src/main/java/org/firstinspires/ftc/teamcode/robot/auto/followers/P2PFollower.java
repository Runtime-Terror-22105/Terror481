package org.firstinspires.ftc.teamcode.robot.auto.followers;

import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

import java.util.ArrayList;
import java.util.Stack;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Predicate;
import java.util.function.Supplier;

public class P2PFollower {

    public static class Builder {
        private final Drivetrain drivetrain;
        private final Stack<Task> tasks = new Stack<>();
        private final RobotHardware hardware;

        public Builder(Drivetrain drivetrain, RobotHardware hardware) {
            this.drivetrain = drivetrain;
            this.hardware = hardware;
        }

        /**
         * Adds an instruction to drive to a point.
         * @param point The point to drive to.
         * @param tolerance The tolerance for how much error there can be on the x,y,h.
         * @param reachedTime How long the robot needs to stay at its destination.
         * @param timeLimit How long the robot can take to drive (ms) before it is considered to have stalled.
         * @return The builder object, to allow for chaining.
         */
        public Builder addPoint(Pose2d point, Pose2d tolerance, double reachedTime, double timeLimit) {
            Task.Context context = new Task.Context(drivetrain);
            context.setGoal(point, tolerance, reachedTime);
            Task task = new Task(
                    context,
                    (Task.Context ctx) -> {
                        Pose2d powers = ctx.p2p.calculatePower(ctx.getCurrentPos());
                        return ctx.p2p.driveToDestination(ctx.getDrivetrain(), powers, ctx.getCurrentPos());
                    },
                    Task.Type.DRIVING,
                    timeLimit
            );
            tasks.add(task);

            return this;
        }

        /**
         * Execute some action once.
         * @param action The function to run.
         * @param timeLimit How long the action can take to run (ms), before it is considered to have stalled.
         * @return The builder object, to allow for chaining.
         */
        public Builder executeActionOnce(Consumer<Task.Context> action, double timeLimit) {
            Task.Context context = new Task.Context(drivetrain);
            Task task = new Task(
                    context,
                    (Task.Context ctx) -> {
                        action.accept(ctx);
                        return true;
                    },
                    Task.Type.ACTION,
                    timeLimit
            );
            tasks.add(task);
            return this;
        }

        /**
         * Execute some action repeatedly (async) until some condition is true.
         * @param condition The condition.
         * @param action The function to run.
         * @param timeLimit How long the action can take to run (ms), before it is considered to have stalled.
         * @return The builder object, to allow for chaining.
         */
        public Builder executeUntilTrue(Predicate<Task.Context> condition,
                                        Consumer<Task.Context> action,
                                        double timeLimit) {
            Task.Context context = new Task.Context(drivetrain);
            Task task = new Task(
                    context,
                    (Task.Context ctx) -> {
                        action.accept(ctx);
                        return condition.test(ctx);
                    },
                    Task.Type.ACTION,
                    timeLimit
            );
            tasks.add(task);
            return this;
        }

        public P2PFollower build() {
            return new P2PFollower(tasks, hardware);
        }
    }

    private final Stack<Task> pendingTasks;
    private final ArrayList<Task> runningTasks = new ArrayList<>();
    private final RobotHardware hardware;

    private P2PFollower(Stack<Task> tasks, RobotHardware hardware) {
        this.pendingTasks = tasks;
        this.hardware = hardware;
    }

    public void follow(BooleanSupplier opModeIsActive, Supplier<Pose2d> currentPos) {
        boolean canContinue = true;

        while (!(pendingTasks.isEmpty() || runningTasks.isEmpty()) && opModeIsActive.getAsBoolean()) {
            // clear bulk cache (assume manual is being used)
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            for (int i = runningTasks.size()-1; i >= 0; i--) {
                Task task = runningTasks.get(i);
                Task.Context ctx = task.getContext();
                ctx.setCurrentPos(currentPos.get());

                if (task.execute(ctx)) { // run the task
                    // if the task finished, remove it from the list
                    runningTasks.remove(i);
                }
            }

            if (runningTasks.isEmpty()) {
                canContinue = true;
            }

            Task newTask;
            while (canContinue && !pendingTasks.isEmpty()) {
                newTask = pendingTasks.pop();
                newTask.getContext().startTimer();
                runningTasks.add(newTask);

                if (newTask.getTaskType().equals(Task.Type.DRIVING)) {
                    canContinue = false;
                }
            }

            hardware.write();
        }
    }
}

