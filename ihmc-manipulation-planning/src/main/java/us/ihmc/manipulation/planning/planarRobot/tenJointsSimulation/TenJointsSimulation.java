package us.ihmc.manipulation.planning.planarRobot.tenJointsSimulation;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.log.LogTools;
import us.ihmc.manipulation.planning.planarRobot.PlanarRobot;
import us.ihmc.manipulation.planning.planarRobot.PlanarRobotInverseKinematicsSolver;
import us.ihmc.manipulation.planning.planarRobot.PlanarRobotTask;
import us.ihmc.manipulation.planning.planarRobot.PlanarRobotVisualizer;

public class TenJointsSimulation
{
   private static final int numberOfJoints = 10;
   private final PlanarRobot robot;
   private final PlanarRobotVisualizer visualizer;
   private final PlanarRobotInverseKinematicsSolver ikSolver;

   public TenJointsSimulation()
   {
      robot = new PlanarRobot();

      robot.addBaseJoint(new Point2D(), 1.0, Math.PI * 10 / 180);
      for (int i = 0; i < numberOfJoints - 1; i++)
         robot.addJoint(1.0, Math.PI * 10 / 180);

      robot.updateRobot();

      visualizer = new PlanarRobotVisualizer(robot, -15.0, 15.0);
      visualizer.open("initial configuration", robot.getJointConfiguration());

      defineTasks();
      ikSolver = new PlanarRobotInverseKinematicsSolver(robot);

      TDoubleArrayList taskConfigurationOne = robot.getTaskConfiguration();
      for (int i = 0; i < taskConfigurationOne.size(); i++)
         System.out.println("" + i + " " + taskConfigurationOne.get(i));
   }

   public void testInverseKinematics()
   {
      TDoubleArrayList desiredTask = new TDoubleArrayList();
      desiredTask.add(5.0);
      desiredTask.add(5.0);
      desiredTask.add(0.0);

      TDoubleArrayList initialConfiguration = new TDoubleArrayList();
      initialConfiguration.addAll(robot.getJointConfiguration());

      ikSolver.setInitialConfiguration(initialConfiguration);
      ikSolver.setDesiredTaskConfiguration(desiredTask);
      
      long start = System.nanoTime();
      
      LogTools.info(""+ikSolver.solve());
      
      long end = System.nanoTime();
      
      LogTools.info("computing time is " + Conversions.nanosecondsToSeconds(end - start));

      visualizer.open("ik test solution", ikSolver.getSolution());
   }

   public static void main(String[] args)
   {
      TenJointsSimulation test = new TenJointsSimulation();

      test.testInverseKinematics();
      

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
