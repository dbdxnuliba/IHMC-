package us.ihmc.avatar.initialSetup;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

import java.util.List;

public interface DRCRobotInitialSetup<T extends Robot>
{
   void initializeRobot(T robot, DRCRobotJointMap jointMap);

   default List<Double> getInitialJointAngles()
   {
      throw new RuntimeException("Not implemented.");
   }

   default Pose3DReadOnly getInitialPelvisPose()
   {
      throw new RuntimeException("Not implemented.");
   }

   void setInitialYaw(double yaw);
   double getInitialYaw();

   void setInitialGroundHeight(double groundHeight);
   double getInitialGroundHeight();

   void setOffset(Vector3D additionalOffset);
   void getOffset(Vector3D offsetToPack);
}
