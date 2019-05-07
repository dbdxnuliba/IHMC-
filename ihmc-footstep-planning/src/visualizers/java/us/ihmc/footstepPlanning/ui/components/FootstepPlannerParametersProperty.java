package us.ihmc.footstepPlanning.ui.components;

import javafx.beans.property.Property;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.BipedFootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.BipedFootstepPlannerParameters;
import us.ihmc.robotEnvironmentAwareness.ui.properties.ParametersProperty;

public class FootstepPlannerParametersProperty extends ParametersProperty<BipedFootstepPlannerParameters>
{
   private DoubleField idealFootstepWidth = new DoubleField(BipedFootstepPlannerParameters::getIdealFootstepWidth, BipedFootstepPlannerParameters::setIdealFootstepWidth);
   private DoubleField idealFootstepLength = new DoubleField(BipedFootstepPlannerParameters::getIdealFootstepLength, BipedFootstepPlannerParameters::setIdealFootstepLength);
   private DoubleField maxStepReach = new DoubleField(BipedFootstepPlannerParameters::getMaximumStepReach, BipedFootstepPlannerParameters::setMaximumStepReach);
   private DoubleField maxStepYaw = new DoubleField(BipedFootstepPlannerParameters::getMaximumStepYaw, BipedFootstepPlannerParameters::setMaximumStepYaw);
   private DoubleField minStepWidth = new DoubleField(BipedFootstepPlannerParameters::getMinimumStepWidth, BipedFootstepPlannerParameters::setMinimumStepWidth);
   private DoubleField minStepLength = new DoubleField(BipedFootstepPlannerParameters::getMinimumStepLength, BipedFootstepPlannerParameters::setMinimumStepLength);
   private DoubleField minStepYaw = new DoubleField(BipedFootstepPlannerParameters::getMinimumStepYaw, BipedFootstepPlannerParameters::setMinimumStepYaw);
   private DoubleField maxStepZ = new DoubleField(BipedFootstepPlannerParameters::getMaximumStepZ, BipedFootstepPlannerParameters::setMaximumStepZ);
   private DoubleField minFootholdPercent = new DoubleField(BipedFootstepPlannerParameters::getMinimumFootholdPercent, BipedFootstepPlannerParameters::setMinimumFootholdPercent);
   private DoubleField minSurfaceIncline = new DoubleField(BipedFootstepPlannerParameters::getMinimumSurfaceInclineRadians, BipedFootstepPlannerParameters::setMinimumSurfaceInclineRadians);
   private DoubleField maxStepWidth = new DoubleField(BipedFootstepPlannerParameters::getMaximumStepWidth, BipedFootstepPlannerParameters::setMaximumStepWidth);
   private DoubleField minXClearanceFromStance = new DoubleField(BipedFootstepPlannerParameters::getMinXClearanceFromStance, BipedFootstepPlannerParameters::setMinXClearanceFromStance);
   private DoubleField minYClearanceFromStance = new DoubleField(BipedFootstepPlannerParameters::getMinYClearanceFromStance, BipedFootstepPlannerParameters::setMinYClearanceFromStance);
   private DoubleField maxXForStepUp = new DoubleField(BipedFootstepPlannerParameters::getMaximumStepReachWhenSteppingUp, BipedFootstepPlannerParameters::setMaximumStepReachWhenSteppingUp);
   private DoubleField minZToConsiderStepUp = new DoubleField(BipedFootstepPlannerParameters::getMaximumStepZWhenSteppingUp, BipedFootstepPlannerParameters::setMaximumStepZWhenSteppingUp);
   private DoubleField maxXForStepDown = new DoubleField(BipedFootstepPlannerParameters::getMaximumStepXWhenForwardAndDown, BipedFootstepPlannerParameters::setMaximumStepXWhenForwardAndDown);
   private DoubleField minZToConsiderStepDown = new DoubleField(BipedFootstepPlannerParameters::getMaximumStepZWhenForwardAndDown, BipedFootstepPlannerParameters::setMaximumStepZWhenForwardAndDown);

   private BooleanField returnBestEffortPlan = new BooleanField(BipedFootstepPlannerParameters::getReturnBestEffortPlan, BipedFootstepPlannerParameters::setReturnBestEffortPlan);
   private BooleanField useQuadraticDistanceCost = new BooleanField(BipedFootstepPlannerParameters::useQuadraticDistanceCost, BipedFootstepPlannerParameters::setUseQuadraticDistanceCost);
   private BooleanField useQuadraticHeightCost = new BooleanField(BipedFootstepPlannerParameters::useQuadraticHeightCost, BipedFootstepPlannerParameters::setUseQuadraticHeightCost);
   private DoubleField yawWeight = new DoubleField(BipedFootstepPlannerParameters::getYawWeight, BipedFootstepPlannerParameters::setYawWeight);
   private DoubleField pitchWeight = new DoubleField(BipedFootstepPlannerParameters::getPitchWeight, BipedFootstepPlannerParameters::setPitchWeight);
   private DoubleField rollWeight = new DoubleField(BipedFootstepPlannerParameters::getRollWeight, BipedFootstepPlannerParameters::setRollWeight);
   private DoubleField forwardWeight = new DoubleField(BipedFootstepPlannerParameters::getForwardWeight, BipedFootstepPlannerParameters::setForwardWeight);
   private DoubleField lateralWeight = new DoubleField(BipedFootstepPlannerParameters::getLateralWeight, BipedFootstepPlannerParameters::setLateralWeight);
   private DoubleField stepUpWeight = new DoubleField(BipedFootstepPlannerParameters::getStepUpWeight, BipedFootstepPlannerParameters::setStepUpWeight);
   private DoubleField stepDownWeight = new DoubleField(BipedFootstepPlannerParameters::getStepDownWeight, BipedFootstepPlannerParameters::setStepDownWeight);
   private DoubleField costPerStep = new DoubleField(BipedFootstepPlannerParameters::getCostPerStep, BipedFootstepPlannerParameters::setCostPerStep);
   private DoubleField boundingBoxCost = new DoubleField(BipedFootstepPlannerParameters::getBoundingBoxCost, BipedFootstepPlannerParameters::setBoundingBoxCost);
   private DoubleField maximum2dDistanceFromBoundingBoxToPenalize = new DoubleField(BipedFootstepPlannerParameters::getMaximum2dDistanceFromBoundingBoxToPenalize, BipedFootstepPlannerParameters::setMaximum2dDistanceFromBoundingBoxToPenalize);
   private DoubleField aStarHeuristicsWeight = new DoubleField(BipedFootstepPlannerParameters::getAStarHeuristicsWeight, BipedFootstepPlannerParameters::setAStarHeuristicsWeight);
   private DoubleField visGraphWithAStarHeuristicsWeight = new DoubleField(BipedFootstepPlannerParameters::getVisGraphWithAStarHeuristicsWeight, BipedFootstepPlannerParameters::setVisGraphWithAStarHeuristicsWeight);
   private DoubleField depthFirstHeuristicsWeight = new DoubleField(BipedFootstepPlannerParameters::getDepthFirstHeuristicsWeight, BipedFootstepPlannerParameters::setDepthFirstHeuristicsWeight);
   private DoubleField bodyPathBasedHeuristicsWeight = new DoubleField(BipedFootstepPlannerParameters::getBodyPathBasedHeuristicsWeight, BipedFootstepPlannerParameters::setBodyPathBasedHeuristicsWeight);

   private BooleanField checkForBodyBoxCollision = new BooleanField(BipedFootstepPlannerParameters::checkForBodyBoxCollisions, BipedFootstepPlannerParameters::setCheckForBodyBoxCollisions);
   private BooleanField performHeuristicSearchPolicies = new BooleanField(BipedFootstepPlannerParameters::performHeuristicSearchPolicies, BipedFootstepPlannerParameters::setPerformHeuristicSearchPolicies);
   private DoubleField bodyBoxWidth = new DoubleField(BipedFootstepPlannerParameters::getBodyBoxWidth, BipedFootstepPlannerParameters::setBodyBoxWidth);
   private DoubleField bodyBoxDepth = new DoubleField(BipedFootstepPlannerParameters::getBodyBoxDepth, BipedFootstepPlannerParameters::setBodyBoxDepth);
   private DoubleField bodyBoxHeight = new DoubleField(BipedFootstepPlannerParameters::getBodyBoxHeight, BipedFootstepPlannerParameters::setBodyBoxHeight);
   private DoubleField bodyBoxBaseX = new DoubleField(BipedFootstepPlannerParameters::getBodyBoxBaseX, BipedFootstepPlannerParameters::setBodyBoxBaseX);
   private DoubleField bodyBoxBaseY = new DoubleField(BipedFootstepPlannerParameters::getBodyBoxBaseY, BipedFootstepPlannerParameters::setBodyBoxBaseY);
   private DoubleField bodyBoxBaseZ = new DoubleField(BipedFootstepPlannerParameters::getBodyBoxBaseZ, BipedFootstepPlannerParameters::setBodyBoxBaseZ);

   private DoubleField cliffHeight = new DoubleField(BipedFootstepPlannerParameters::getCliffHeightToAvoid, BipedFootstepPlannerParameters::setCliffHeightToAvoid);
   private DoubleField cliffClearance = new DoubleField(BipedFootstepPlannerParameters::getMinimumDistanceFromCliffBottoms, BipedFootstepPlannerParameters::setMinimumDistanceFromCliffBottoms);
   private DoubleField maxWiggleXY = new DoubleField(BipedFootstepPlannerParameters::getMaximumXYWiggleDistance, BipedFootstepPlannerParameters::setMaximumXYWiggleDistance);
   private DoubleField maxWiggleYaw = new DoubleField(BipedFootstepPlannerParameters::getMaximumYawWiggle, BipedFootstepPlannerParameters::setMaximumYawWiggle);
   private DoubleField wiggleInsideDelta = new DoubleField(BipedFootstepPlannerParameters::getWiggleInsideDelta, BipedFootstepPlannerParameters::setWiggleInsideDelta);

   public FootstepPlannerParametersProperty(Object bean, String name)
   {
      this(bean, name, new DefaultFootstepPlanningParameters());
   }

   public FootstepPlannerParametersProperty(Object bean, String name, BipedFootstepPlannerParametersReadOnly footstepPlannerParameters)
   {
      super(bean, name, new BipedFootstepPlannerParameters(footstepPlannerParameters));
   }

   public void setPlannerParameters(BipedFootstepPlannerParametersReadOnly parameters)
   {
      setValue(new BipedFootstepPlannerParameters(parameters));
   }

   @Override
   protected BipedFootstepPlannerParameters getValueCopy(BipedFootstepPlannerParameters valueToCopy)
   {
      return new BipedFootstepPlannerParameters(valueToCopy);
   }

   public void bidirectionalBindIdealFootstepWidth(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, idealFootstepWidth);
   }

   public void bidirectionalBindIdealFootstepLength(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, idealFootstepLength);
   }

   public void bidirectionalBindReturnBestEffortPlan(Property<Boolean> property)
   {
      bindFieldBidirectionalToBooleanProperty(property, returnBestEffortPlan);
   }

   public void bidirectionalBindMaxStepReach(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxStepReach);
   }

   public void bidirectionalBindMaxStepYaw(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxStepYaw);
   }

   public void bidirectionalBindMinStepWidth(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minStepWidth);
   }

   public void bidirectionalBindMinStepLength(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minStepLength);
   }

   public void bidirectionalBindMinStepYaw(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minStepYaw);
   }

   public void bidirectionalBindMaxStepZ(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxStepZ);
   }

   public void bidirectionalBindMinFootholdPercent(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minFootholdPercent);
   }

   public void bidirectionalBindMinSurfaceIncline(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minSurfaceIncline);
   }

   public void bidirectionalBindMaxStepWidth(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxStepWidth);
   }

   public void bidirectionalBindMinXClearanceFromStance(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minXClearanceFromStance);
   }

   public void bidirectionalBindMinYClearanceFromStance(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minYClearanceFromStance);
   }


   public void bidirectionalBindCheckBodyBoxCollisions(Property<Boolean> property)
   {
      bindFieldBidirectionalToBooleanProperty(property, checkForBodyBoxCollision);
   }

   public void bidirectionalBindPerformHeuristicSearchPolicies(Property<Boolean> property)
   {
      bindFieldBidirectionalToBooleanProperty(property, performHeuristicSearchPolicies);
   }

   public void bidirectionalBindMaxWiggleXY(Property<Double> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxWiggleXY);
   }

   public void bidirectionalBindMaxWiggleYaw(Property<Double> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxWiggleYaw);
   }

   public void bidirectionalBindWiggleInsideDelta(Property<Double> property)
   {
      bindFieldBidirectionalToNumberProperty(property, wiggleInsideDelta);
   }

   public void bidirectionalBindBodyBoxWidth(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, bodyBoxWidth);
   }

   public void bidirectionalBindBodyBoxDepth(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, bodyBoxDepth);
   }

   public void bidirectionalBindBodyBoxHeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, bodyBoxHeight);
   }

   public void bidirectionalBindBodyBoxBaseX(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, bodyBoxBaseX);
   }

   public void bidirectionalBindBodyBoxBaseY(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, bodyBoxBaseY);
   }

   public void bidirectionalBindBodyBoxBaseZ(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, bodyBoxBaseZ);
   }

   public void bidirectionalBindYawWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, yawWeight);
   }

   public void bidirectionalBindPitchWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, pitchWeight);
   }

   public void bidirectionalBindRollWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, rollWeight);
   }

   public void bidirectionalBindForwardWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, forwardWeight);
   }

   public void bidirectionalBindLateralWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, lateralWeight);
   }

   public void bidirectionalBindStepUpWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, stepUpWeight);
   }

   public void bidirectionalBindStepDownWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, stepDownWeight);
   }

   public void bidirectionalBindCostPerStep(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, costPerStep);
   }

   public void bidirectionalBindCliffHeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, cliffHeight);
   }
   
   public void bidirectionalBindCliffClearance(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, cliffClearance);
   }

   public void bidirectionalBindMaximum2dDistanceFromBoundingBoxToPenalize(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maximum2dDistanceFromBoundingBoxToPenalize);
   }

   public void bidirectionalBindBoundingBoxCost(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, boundingBoxCost);
   }

   public void bidirectionalBindAStarHeuristicsWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, aStarHeuristicsWeight);
   }

   public void bidirectionalBindUseQuadraticHeightCost(Property<Boolean> property)
   {
      bindFieldBidirectionalToBooleanProperty(property, useQuadraticHeightCost);
   }

   public void bidirectionalBindUseQuadraticDistanceCost(Property<Boolean> property)
   {
      bindFieldBidirectionalToBooleanProperty(property, useQuadraticDistanceCost);
   }

   public void bidirectionBindMaxXForStepUp(Property<Double> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxXForStepUp);
   }

   public void bidirectionBindMinZToConsiderStepUp(Property<Double> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minZToConsiderStepUp);
   }

   public void bidirectionBindMaxXForStepDown(Property<Double> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxXForStepDown);
   }

   public void bidirectionBindMinZToConsiderStepDown(Property<Double> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minZToConsiderStepDown);
   }
}
