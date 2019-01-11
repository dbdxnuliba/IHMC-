package us.ihmc.manipulation.planning.planarRobot.tenJointsSimulation;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.manipulation.planning.planarRobot.PlanarRobot;
import us.ihmc.manipulation.planning.planarRobot.PlanarRobotTask;
import us.ihmc.manipulation.planning.planarRobot.PlanarRobotVisualizer;

public class TenJointsSimulation
{
   private final PlanarRobot robot;
   private final PlanarRobotVisualizer visualizer;

   public TenJointsSimulation()
   {
      robot = new PlanarRobot();

      int numberOfJoints = 9;

      robot.addBaseJoint(new Point2D(), 1.0, Math.PI * 10 / 180);

      for (int i = 0; i < numberOfJoints; i++)
         robot.addJoint(1.0, Math.PI * 10 / 180);

      robot.updateRobot();

      visualizer = new PlanarRobotVisualizer(robot, -15.0, 15.0);
      visualizer.open("initial configuration", robot.getJointConfiguration());

      defineTasks();
      
      TDoubleArrayList taskConfigurationOne = robot.getTaskConfiguration();
      for (int i = 0; i < taskConfigurationOne.size(); i++)
         System.out.println("" + taskConfigurationOne.get(i));
   }

   public static void main(String[] args)
   {
      TenJointsSimulation test = new TenJointsSimulation();

      System.out.println("all tests are completed");
   }

   private void defineTasks()
   {
      PlanarRobotTask endEffectorTaskX = new PlanarRobotTask("endX", robot)
      {
         @Override
         public double getTask(TDoubleArrayList jointConfiguration)
         {
            for (int i = 0; i < robot.getJointDimension(); i++)
               robot.getJoint(i).setJointAngle(jointConfiguration.get(i));
            robot.updateRobot();

            return robot.getLastJoint().getEndTip().getX();
         }
      };

      PlanarRobotTask endEffectorTaskY = new PlanarRobotTask("endY", robot)
      {
         @Override
         public double getTask(TDoubleArrayList jointConfiguration)
         {
            for (int i = 0; i < robot.getJointDimension(); i++)
               robot.getJoint(i).setJointAngle(jointConfiguration.get(i));
            robot.updateRobot();

            return robot.getLastJoint().getEndTip().getY();
         }
      };

      PlanarRobotTask endEffectorTaskYaw = new PlanarRobotTask("endYaw", robot)
      {
         @Override
         public double getTask(TDoubleArrayList jointConfiguration)
         {
            for (int i = 0; i < robot.getJointDimension(); i++)
               robot.getJoint(i).setJointAngle(jointConfiguration.get(i));
            robot.updateRobot();

            return robot.getLastJoint().getEndTipAngle();
         }
      };

      robot.addTask(endEffectorTaskX);
      robot.addTask(endEffectorTaskY);
      robot.addTask(endEffectorTaskYaw);
   }
}
