package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class CollinearForceBasedPlannerOptimizationControlModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoDouble maxPlannerSegmentTime;
   private final YoInteger maxNumberOfPlanningSegments;

   private final YoFramePoint initialCoMLocation;
   private final YoFramePoint initialCoPLocation;
   private final YoFrameVector initialCoMVelocity;
   private final YoFramePoint finalCoMLocation;
   private final YoFramePoint finalCoPLocation;
   private final YoFrameVector finalCoMVelocity;

   private final RecyclingArrayList<ContactState> motionPlannerNodeList;
   private final FrameVector3DReadOnly gravity;
   private final CollinearForceBasedPlannerIterationResult sqpSolution;

   public CollinearForceBasedPlannerOptimizationControlModule(CollinearForceBasedPlannerIterationResult sqpSolution, FrameVector3DReadOnly gravity, YoVariableRegistry registry)
   {
      String namePrefix = getClass().getSimpleName();
      maxPlannerSegmentTime = new YoDouble(namePrefix + "MaxPlannerSegmentTime", registry);
      maxNumberOfPlanningSegments = new YoInteger(namePrefix + "MaxNumberOfPlanningSegments", registry);

      initialCoMLocation = new YoFramePoint(namePrefix + "InitialCoMLocation", worldFrame, registry);
      initialCoPLocation = new YoFramePoint(namePrefix + "InitialCoPLocation", worldFrame, registry);
      initialCoMVelocity = new YoFrameVector(namePrefix + "InitialCoMVelocity", worldFrame, registry);
      finalCoMLocation = new YoFramePoint(namePrefix + "FinalCoMLocation", worldFrame, registry);
      finalCoPLocation = new YoFramePoint(namePrefix + "FinalCoPLocation", worldFrame, registry);
      finalCoMVelocity = new YoFrameVector(namePrefix + "FinalCoMVelocity", worldFrame, registry);

      this.sqpSolution = sqpSolution;
      this.gravity = gravity;
      motionPlannerNodeList = new RecyclingArrayList<>(100, ContactState.class);
   }

   public void initialize(double maxPlannerSegmentTime, int maxNumberOfPlanningSegments)
   {
      this.maxPlannerSegmentTime.set(maxPlannerSegmentTime);
      this.maxNumberOfPlanningSegments.set(maxNumberOfPlanningSegments);
   }
   
   public void reset()
   {
      motionPlannerNodeList.clear();
   }
   
   public void setInitialState(FramePoint3D initialCoMLocation, FrameVector3D initialCoMVelocity, FramePoint3D initialCoPLocation)
   {
      this.initialCoMLocation.set(initialCoMLocation);
      this.initialCoMVelocity.set(initialCoMVelocity);
      this.initialCoPLocation.set(initialCoPLocation);
   }

   public void setFinalState(FramePoint3D finalCoMLocation, FrameVector3D finalCoMVelocity, FrameVector3D finalCoPLocation)
   {
      this.finalCoMLocation.set(finalCoMLocation);
      this.finalCoMVelocity.set(finalCoMVelocity);
      this.finalCoPLocation.set(finalCoPLocation);
   }

   public void submitContactState(ContactState contactState)
   {
      motionPlannerNodeList.add().set(contactState);
   }

   public void generateLinearizedSystemObjective()
   {
      // TODO Auto-generated method stub
      
   }

   public void generateLinearizedSystemConstraints()
   {
      // TODO Auto-generated method stub
      
   }

   public void updateSolution()
   {
      // TODO Auto-generated method stub
      
   }

   public boolean compute()
   {
      // TODO Auto-generated method stub
      return false;
   }
}
