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

   public PlanarRobot(PlanarRobot other)
   {
      this();

      for (int i = 0; i < other.getJointDimension(); i++)
      {
         PlanarRobotJoint jointToAdd = other.robotJoints.get(i);
         if (jointToAdd.getParentJoint() == null)
            addBaseJoint(jointToAdd.getJointPoint(), jointToAdd.getLinkLength(), 0.0);
         else
            addJoint(jointToAdd.getLinkLength(), 0.0);
      }
      robotTasks.addAll(other.robotTasks);
   }

   public void addBaseJoint(Point2D basePoint, double linkLength, double jointAngle)
   {
      PlanarRobotJoint baseJoint = new PlanarRobotJoint(basePoint, linkLength, jointAngle);
      robotJoints.clear();
      robotJoints.add(baseJoint);
      robotJointConfiguration.add(jointAngle);
   }

   public void addJoint(double linkLength, double jointAngle)
   {
      PlanarRobotJoint joint = new PlanarRobotJoint(getLastJoint(), linkLength, jointAngle);
      robotJoints.add(joint);
      robotJointConfiguration.add(jointAngle);
   }

   public void addTask(PlanarRobotTask taskToAdd)
   {
      robotTasks.add(taskToAdd);
   }

   public void setJointConfiguration(TDoubleArrayList jointConfiguration)
   {
      if (jointConfiguration.size() != robotJointConfiguration.size())
         System.out.println("the joint configuration has different size with robot joint dimension (" + jointConfiguration.size() + ", "
               + robotJointConfiguration.size() +").");
      else
      {
         robotJointConfiguration.clear();
         robotJointConfiguration.addAll(jointConfiguration);
         for (int i = 0; i < getJointDimension(); i++)
            robotJoints.get(i).setJointAngle(jointConfiguration.get(i));
      }
      updateRobot();
   }

   public void setJointConfiguration(int i, double jointAngle)
   {
      robotJoints.get(i).setJointAngle(jointAngle);
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

   public TDoubleArrayList getJointConfiguration()
   {
      robotJointConfiguration.clear();
      for (int i = 0; i < robotJoints.size(); i++)
         robotJointConfiguration.add(robotJoints.get(i).getJointAngle());
      return robotJointConfiguration;
   }

   public TDoubleArrayList getTaskConfiguration()
   {
      robotTaskConfiguration.clear();
      for (int i = 0; i < robotTasks.size(); i++)
         robotTaskConfiguration.add(robotTasks.get(i).getTask(robotJointConfiguration));

      return robotTaskConfiguration;
   }

   public double getTask(int i)
   {
      return robotTasks.get(i).getTask(robotJointConfiguration);
   }

   public String getTaskName(int i)
   {
      return robotTasks.get(i).getTaskName();
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
