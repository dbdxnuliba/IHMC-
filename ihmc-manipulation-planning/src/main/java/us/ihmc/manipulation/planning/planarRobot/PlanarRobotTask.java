package us.ihmc.manipulation.planning.planarRobot;

import gnu.trove.list.array.TDoubleArrayList;

public abstract class PlanarRobotTask
{
   private final String taskName;
   protected final PlanarRobot robot;

   public PlanarRobotTask(String taskName, PlanarRobot robot)
   {
      this.taskName = taskName;
      this.robot = robot;
   }

   public String getTaskName()
   {
      return taskName;
   }

   public abstract double getTask(TDoubleArrayList jointConfiguration);
}
