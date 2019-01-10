package us.ihmc.manipulation.planning.planarRobot;

import java.util.ArrayList;

import us.ihmc.euclid.tuple2D.Point2D;

public class PlanarRobot
{
   private final ArrayList<PlanarRobotJoint> robotJoints;

   //   private final TDoubleArrayList robotTaskSpace;
   //   private final TDoubleArrayList robotJointConfiguration;
   //   
   //   private final ArrayList<PlanarRobotTask> robotTasks;

   public PlanarRobot()
   {
      robotJoints = new ArrayList<PlanarRobotJoint>();
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

   }
   
   public void setJointConfiguration()
   {

   }

   public void updateRobot()
   {
      for(int i=0;i<robotJoints.size();i++)
         robotJoints.get(i).updateJoint();
   }

   public int getJointDimension()
   {
      return robotJoints.size();
   }

   public int getTaskDimension()
   {
      return 0;
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
