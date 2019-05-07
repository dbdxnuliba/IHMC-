package us.ihmc.footstepPlanning.graphSearch.parameters;

import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertySet;

public class BipedFootstepPlannerParameters extends BipedFootstepPlannerCostParameters implements BipedFootstepPlannerParametersReadOnly
{
   private final StoredPropertySet propertySet;

   public static final String CONFIGURATION_FILE_NAME = "./saved-configurations/footstepPlannerParameters.txt";

   public BipedFootstepPlannerParameters()
   {
      this(null);
   }

   public BipedFootstepPlannerParameters(BipedFootstepPlannerParametersReadOnly footstepPlannerParameters)
   {
      super(footstepPlannerParameters.getCostParameters());

      this.propertySet = new StoredPropertySet(BipedFootstepPlannerParameterKeys.keys,
                                               getClass(),
                                               "ihmc-open-robotics-software",
                                               "ihmc-footstep-planning/src/main/resources");

      if (footstepPlannerParameters != null)
         set(footstepPlannerParameters);
   }

   public void set(BipedFootstepPlannerParametersReadOnly footstepPlannerParameters)
   {
      propertySet.setAllValues(footstepPlannerParameters.getStoredPropertySet().getAllValues());
   }

   public void setValue(DoubleStoredPropertyKey key, double value)
   {
      propertySet.setValue(key, value);
   }

   public void setValue(IntegerStoredPropertyKey key, int value)
   {
      propertySet.setValue(key, value);
   }

   public void setValue(BooleanStoredPropertyKey key, boolean value)
   {
      propertySet.setValue(key, value);
   }

   public void setIdealFootstepWidth(double idealFootstepWidth)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.idealFootstepWidth, idealFootstepWidth);
   }

   public void setIdealFootstepLength(double idealFootstepLength)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.idealFootstepLength, idealFootstepLength);
   }

   public void setWiggleInsideDelta(double wiggleInsideDelta)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.wiggleInsideDelta, wiggleInsideDelta);
   }

   public void setWiggleIntoConvexHullOfPlanarRegions(boolean wiggleIntoConvexHullOfPlanarRegions)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.wiggleIntoConvexHullOfPlanarRegions, wiggleIntoConvexHullOfPlanarRegions);
   }

   public void setRejectIfCannotFullyWiggleInside(boolean rejectIfCannotFullyWiggleInside)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.rejectIfCannotFullyWiggleInside, rejectIfCannotFullyWiggleInside);
   }

   public void setMaximumXYWiggleDistance(double maximumXYWiggleDistance)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.maximumXYWiggleDistance, maximumXYWiggleDistance);
   }

   public void setMaximumYawWiggle(double maximumYawWiggle)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.maximumYawWiggle, maximumYawWiggle);
   }

   public void setMaximumStepReach(double maxStepReach)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.maxStepReach, maxStepReach);
   }

   public void setMaximumStepYaw(double maxStepYaw)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.maxStepYaw, maxStepYaw);
   }

   public void setMinimumStepWidth(double minStepWidth)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.minStepWidth, minStepWidth);
   }

   public void setMinimumStepLength(double minStepLength)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.minStepLength, minStepLength);
   }

   public void setMinimumStepYaw(double minStepYaw)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.minStepYaw, minStepYaw);
   }

   public void setMaximumStepZ(double maxStepZ)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.maxStepZ, maxStepZ);
   }

   public void setMaximumStepWidth(double maxStepWidth)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.maxStepWidth, maxStepWidth);
   }

   public void setMinimumFootholdPercent(double minFootholdPercent)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.minFootholdPercent, minFootholdPercent);
   }

   public void setMinimumSurfaceInclineRadians(double minSurfaceIncline)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.minSurfaceIncline, minSurfaceIncline);
   }

   public void setMaximumStepReachWhenSteppingUp(double maximumStepReachWhenSteppingUp)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.maximumStepReachWhenSteppingUp, maximumStepReachWhenSteppingUp);
   }

   public void setMaximumStepZWhenSteppingUp(double maximumStepZWhenSteppingUp)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.maximumStepZWhenSteppingUp, maximumStepZWhenSteppingUp);
   }

   public void setMaximumStepXWhenForwardAndDown(double maximumStepXWhenForwardAndDown)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.maximumStepXWhenForwardAndDown, maximumStepXWhenForwardAndDown);
   }

   public void setMaximumStepZWhenForwardAndDown(double maximumStepZWhenForwardAndDown)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.maximumStepZWhenForwardAndDown, maximumStepZWhenForwardAndDown);
   }

   public void setMinXClearanceFromStance(double minXClearanceFromStance)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.minXClearanceFromStance, minXClearanceFromStance);
   }

   public void setMinYClearanceFromStance(double minYClearanceFromStance)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.minYClearanceFromStance, minYClearanceFromStance);
   }

   public void setMaximumZPenetrationOnValleyRegions(double maximumZPenetrationOnValleyRegions)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.maximumZPenetrationOnValleyRegions, maximumZPenetrationOnValleyRegions);
   }

   public void setCliffHeightToAvoid(double cliffHeightToAvoid)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.cliffHeightToAvoid, cliffHeightToAvoid);
   }

   public void setMinimumDistanceFromCliffBottoms(double minimumDistanceFromCliffBottoms)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.minimumDistanceFromCliffBottoms, minimumDistanceFromCliffBottoms);
   }

   public void setReturnBestEffortPlan(boolean returnBestEffortPlan)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.returnBestEffortPlan, returnBestEffortPlan);
   }

   public void setMinimumStepsForBestEffortPlan(int minimumStepsForBestEffortPlan)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.minimumStepsForBestEffortPlan, minimumStepsForBestEffortPlan);
   }

   public void setBodyGroundClearance(double bodyGroundClearance)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.bodyGroundClearance, bodyGroundClearance);
   }

   public void setCheckForBodyBoxCollisions(boolean checkForBodyBoxCollisions)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.checkForBodyBoxCollisions, checkForBodyBoxCollisions);
   }

   public void setPerformHeuristicSearchPolicies(boolean performHeuristicSearchPolicies)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.performHeuristicSearchPolicies, performHeuristicSearchPolicies);
   }

   public void setBodyBoxWidth(double bodyBoxWidth)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.bodyBoxWidth, bodyBoxWidth);
   }

   public void setBodyBoxHeight(double bodyBoxHeight)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.bodyBoxHeight, bodyBoxHeight);
   }

   public void setBodyBoxDepth(double bodyBoxDepth)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.bodyBoxDepth, bodyBoxDepth);
   }

   public void setBodyBoxBaseX(double bodyBoxBaseX)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.bodyBoxBaseX, bodyBoxBaseX);
   }

   public void setBodyBoxBaseY(double bodyBoxBaseY)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.bodyBoxBaseY, bodyBoxBaseY);
   }

   public void setBodyBoxBaseZ(double bodyBoxBaseZ)
   {
      propertySet.setValue(BipedFootstepPlannerParameterKeys.bodyBoxBaseZ, bodyBoxBaseZ);
   }

   @Override
   public StoredPropertySet getStoredPropertySet()
   {
      return propertySet;
   }

   @Override
   public BipedFootstepPlannerCostParametersReadOnly getCostParameters()
   {
      return this;
   }
}
