package org.firstinspires.ftc.teamcode.robot.auto.followers;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.drive.Drivetrain;

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

        public Builder(Drivetrain drivetrain) {
            this.drivetrain = drivetrain;
        }

        public Builder addPoint(Pose2d point, Pose2d tolerance) {
            Task.Context context = new Task.Context(drivetrain);
            context.setGoal(point, tolerance);
            Task task = new Task(
                    context,
                    (Task.Context ctx) -> {
                        return ctx.p2p.driveToDestination(ctx.getDrivetrain(), ctx.getCurrentPos());
                    },
                    Task.TaskType.DRIVING
            );
            tasks.add(task);

            return this;
        }

        public Builder addPoint(Pose2d point, double tolerance) {
            return addPoint(point, new Pose2d(tolerance, tolerance, tolerance));
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
            return new P2PFollower(tasks);
        }
    }

    private final Stack<Task> tasks;
    private final ArrayList<Task> currentTasks = new ArrayList<>();

    private P2PFollower(Stack<Task> tasks) {
        this.tasks = tasks;
    }

    public void follow(BooleanSupplier opModeIsActive, Supplier<Pose2d> currentPos) {
        while (!tasks.isEmpty() && opModeIsActive.getAsBoolean()) {
            boolean addNewTask = true;
            for (int i = currentTasks.size()-1; i >= 0; i--) {
                Task task = currentTasks.get(i);
                Task.Context ctx = task.context;
                ctx.setCurrentPos(currentPos.get());

                if (task.execute(ctx)) { // run the task
                    // if the task finished, remove it from the list
                    currentTasks.remove(i);
                } else {
                    // we don't want to start new tasks if we're driving
                    if (task.taskType.equals(Task.TaskType.DRIVING) ||
                        task.taskType.equals(Task.TaskType.FINISH_ACTIONS)) {
                        addNewTask = false;
                    }
                }
            }

            if (addNewTask) {
                currentTasks.add(tasks.pop());
            }
        }
    }
}

