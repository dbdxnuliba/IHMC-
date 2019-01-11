package us.ihmc.manipulation.planning.planarRobot;

import gnu.trove.list.array.TDoubleArrayList;

public class PlanarRobotInverseKinematicsSolver implements InverseKinematicsInterface
{
   private static final int maximumNumberOfIteration = 100;

   private final PlanarRobot robot;

   public PlanarRobotInverseKinematicsSolver(PlanarRobot robot)
   {
      this.robot = new PlanarRobot(robot);
   }

   @Override
   public boolean solve()
   {
      return true;
   }

   @Override
   public void setInitialConfiguration(TDoubleArrayList jointConfiguration)
   {
      robot.setJointConfiguration(jointConfiguration);
   }

   @Override
   public void setDesiredTaskConfiguration(TDoubleArrayList taskConfiguration)
   {

   }

   @Override
   public TDoubleArrayList getSolution()
   {
      return null;
   }

}
