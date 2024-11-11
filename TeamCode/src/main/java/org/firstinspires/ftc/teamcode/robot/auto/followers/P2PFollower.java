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

        public Builder addPoint(Pose2d point, Pose2d tolerance, double reachedTime) {
            Task.Context context = new Task.Context(drivetrain);
            context.setGoal(point, tolerance, reachedTime);
            Task task = new Task(
                    context,
                    (Task.Context ctx) -> {
                        Pose2d powers = ctx.p2p.calculatePower(ctx.getCurrentPos());
                        return ctx.p2p.driveToDestination(ctx.getDrivetrain(), powers, ctx.getCurrentPos());
                    },
                    Task.TaskType.DRIVING
            );
            tasks.add(task);

            return this;
        }

        public Builder addPoint(Pose2d point, double tolerance, double reachedTime) {
            return addPoint(point, new Pose2d(tolerance, tolerance, tolerance), reachedTime);
        }

        public Builder executeActionOnce(Consumer<Task.Context> action) {
            Task.Context context = new Task.Context(drivetrain);
            Task task = new Task(
                    context,
                    (Task.Context ctx) -> {
                        action.accept(ctx);
                        return true;
                    },
                    Task.TaskType.ACTION
            );
            tasks.add(task);
            return this;
        }

        public Builder executeUntilTrue(Predicate<Task.Context> condition,
                                        Consumer<Task.Context> action) {
            Task.Context context = new Task.Context(drivetrain);
            Task task = new Task(
                    context,
                    (Task.Context ctx) -> {
                        action.accept(ctx);
                        return condition.test(ctx);
                    },
                    Task.TaskType.ACTION
            );
            tasks.add(task);
            return this;
        }

        public Builder finishActions() {
            Task.Context context = new Task.Context(drivetrain);
            Task task = new Task(
                    context,
                    (Task.Context ctx) -> false,
                    Task.TaskType.FINISH_ACTIONS
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
        while (!(pendingTasks.isEmpty() || runningTasks.isEmpty()) && opModeIsActive.getAsBoolean()) {
            // clear bulk cache (assume manual is being used)
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            boolean addNewTask = true;
            for (int i = runningTasks.size()-1; i >= 0; i--) {
                Task task = runningTasks.get(i);
                Task.Context ctx = task.context;
                ctx.setCurrentPos(currentPos.get());

                if (task.execute(ctx)) { // run the task
                    // if the task finished, remove it from the list
                    runningTasks.remove(i);
                } else {
                    // we don't want to start new tasks if we're driving
                    if (task.taskType.equals(Task.TaskType.DRIVING) ||
                        task.taskType.equals(Task.TaskType.FINISH_ACTIONS)) {
                        addNewTask = false;
                    }
                }
            }

            if (addNewTask) {
                Task newTask;
                do {
                    newTask = pendingTasks.pop();
                    runningTasks.add(newTask);
                } while (newTask.taskType.equals(Task.TaskType.DRIVING) ||
                         newTask.taskType.equals(Task.TaskType.FINISH_ACTIONS));
            }

            hardware.write();
        }
    }
}

