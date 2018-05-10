package us.ihmc.commonWalkingControlModules.controlModules.flight;

import java.util.List;

import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.ControlModuleHelper;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.LinearMotionPlannerNode;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.YawMotionPlannerNode;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * Wrapper around the {@code CentroidalMotionPlanner} & {@code TwistPlanner} to enable high level objectives to be translated to 
 * motion planning inputs
 * 
 * 
 * 
 * @author Apoorv S
 */
public class WholeBodyMotionPlanner
{
   public static final int numberOfForceCoefficients = ControlModuleHelper.forceCoefficients;
   public static final int maxNumberOfSegments = 20;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final RecyclingArrayList<LinearMotionPlannerNode> linearMotionPlannerNodeList;
   private final RecyclingArrayList<YawMotionPlannerNode> yawMotionPlannerNodeList;

   private final FrameQuaternion tempOrientationForComputation = new FrameQuaternion(worldFrame);

   public WholeBodyMotionPlanner(YoVariableRegistry registry)
   {
      this.linearMotionPlannerNodeList = new RecyclingArrayList<>(maxNumberOfSegments, LinearMotionPlannerNode.class);
      this.yawMotionPlannerNodeList = new RecyclingArrayList<>(maxNumberOfSegments, YawMotionPlannerNode.class);
   }

   public void reset()
   {
      this.linearMotionPlannerNodeList.clear();
      this.yawMotionPlannerNodeList.clear();
   }

   public void processContactStateList(List<ContactState> contactStateList)
   {
      int numberOfContactStates = contactStateList.size();
      for (int i = 0; i < numberOfContactStates; i++)
      {
         ContactState contactState = contactStateList.get(i);
         contactState.getOrientation(tempOrientationForComputation);
         tempOrientationForComputation.changeFrame(worldFrame);
         double contactStateYawAngle = tempOrientationForComputation.getYaw();
         BipedContactType contactType = contactState.getContactType();
         boolean robotIsSupported = contactType.isRobotSupported();
         if (robotIsSupported)
         {

         }
         else
         {

         }
      }
   }
}