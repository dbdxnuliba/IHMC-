package us.ihmc.manipulation.planning.planarRobot;

import java.util.ArrayList;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.tuple2D.Point2D;

public class PlanarRobot
{
   private final ArrayList<PlanarRobotJoint> robotJoints;

   private final TDoubleArrayList robotTaskConfiguration;
   private final TDoubleArrayList robotJointConfiguration;

   private final ArrayList<PlanarRobotTask> robotTasks;

   public PlanarRobot()
   {
      robotJoints = new ArrayList<PlanarRobotJoint>();
      robotTaskConfiguration = new TDoubleArrayList();
      robotJointConfiguration = new TDoubleArrayList();
      robotTasks = new ArrayList<PlanarRobotTask>();
   }

   public void addBaseJoint(Point2D basePoint, double linkLength, double jointAngle)
   {
      PlanarRobotJoint baseJoint = new PlanarRobotJoint(basePoint, linkLength, jointAngle);
      robotJoints.clear();
      robotJoints.add(baseJoint);
   }

   public void addJoint(double linkLength, double jointAngle)
   {
      PlanarRobotJoint joint = new PlanarRobotJoint(getLastJoint(), linkLength, jointAngle);
      robotJoints.add(joint);
   }

   public void addTask(PlanarRobotTask taskToAdd)
   {
      robotTasks.add(taskToAdd);
   }

   public void setJointConfiguration(TDoubleArrayList jointConfiguration)
   {
      if (jointConfiguration.size() != getJointDimension())
         System.out.println("the joint configuration has different size with robot joint dimension.");
      else
      {
         robotJointConfiguration.clear();
         robotJointConfiguration.addAll(jointConfiguration);
         for (int i = 0; i < getJointDimension(); i++)
            robotJoints.get(i).setJointAngle(jointConfiguration.get(i));
      }
      updateRobot();
   }

   public void updateRobot()
   {
      for (int i = 0; i < robotJoints.size(); i++)
         robotJoints.get(i).updateJoint();
   }

   public int getJointDimension()
   {
      return robotJoints.size();
   }

   public int getTaskDimension()
   {
      return robotTasks.size();
   }

   public TDoubleArrayList getTaskConfiguration()
   {
      robotTaskConfiguration.clear();
      for (int i = 0; i < getTaskDimension(); i++)
         robotTaskConfiguration.add(robotTasks.get(i).getTaskConfiguration(robotJointConfiguration));

      return robotTaskConfiguration;
   }

   public PlanarRobotJoint getLastJoint()
   {
      return robotJoints.get(robotJoints.size() - 1);
   }

   public PlanarRobotJoint getJoint(int i)
   {
      return robotJoints.get(i);
   }
}
