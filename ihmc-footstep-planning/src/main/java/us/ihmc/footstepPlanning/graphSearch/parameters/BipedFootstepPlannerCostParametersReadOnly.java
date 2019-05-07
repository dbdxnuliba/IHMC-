package us.ihmc.footstepPlanning.graphSearch.parameters;

import us.ihmc.footstepPlanning.graphSearch.stepCost.QuadraticDistanceAndYawCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.EuclideanDistanceAndYawBasedCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.LinearHeightCost;
import us.ihmc.tools.property.StoredPropertySetReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;
import static us.ihmc.footstepPlanning.graphSearch.parameters.BipedFootstepPlannerCostParameterKeys.*;

public interface BipedFootstepPlannerCostParametersReadOnly
{
   StoredPropertySetReadOnly getStoredPropertySet();

   /**
    * Determines which cost function for distance and yaw to use, between {@link QuadraticDistanceAndYawCost} and {@link EuclideanDistanceAndYawBasedCost}
    */
   default boolean useQuadraticDistanceCost()
   {
      return getStoredPropertySet().getValue(useQuadraticDistanceCost);
   }

   /**
    * Determines which cost function for distance and yaw to use, between {@link QuadraticDistanceAndYawCost} and {@link LinearHeightCost}
    */
   default boolean useQuadraticHeightCost()
   {
      return getStoredPropertySet().getValue(useQuadraticHeightCost);
   }

   /**
    * Gets the weight for the heuristics in the A Star planner.
    */
   default DoubleProvider getAStarHeuristicsWeightProvider()
   {
      return () -> getStoredPropertySet().getValue(aStarHeuristicsWeight);
   }

   /**
    * Gets the weight for the heuristics in the Visibility graph with A star planner.
    */
   default DoubleProvider getVisGraphWithAStarHeuristicsWeightProvider()
   {
      return () -> getStoredPropertySet().getValue(visGraphWithAStarHeuristicsWeight);
   }

   /**
    * Gets the weight for the heuristics in the Depth First planner.
    */
   default DoubleProvider getDepthFirstHeuristicsWeightProvider()
   {
      return () -> getStoredPropertySet().getValue(depthFirstHeuristicsWeight);
   }

   /**
    * Gets the weight for the heuristics in the Body path based planner.
    */
   default DoubleProvider getBodyPathBasedHeuristicsWeightProvider()
   {
      return () -> getStoredPropertySet().getValue(bodyPathBasedHeuristicsWeight);
   }

   /**
    * Gets the weight for the heuristics in the A Star planner.
    */
   default double getAStarHeuristicsWeight()
   {
      return getStoredPropertySet().getValue(aStarHeuristicsWeight);
   }

   /**
    * Gets the weight for the heuristics in the Visibility graph with A star planner.
    */
   default double getVisGraphWithAStarHeuristicsWeight()
   {
      return getStoredPropertySet().getValue(visGraphWithAStarHeuristicsWeight);
   }

   /**
    * Gets the weight for the heuristics in the Depth First planner.
    */
   default double getDepthFirstHeuristicsWeight()
   {
      return getStoredPropertySet().getValue(depthFirstHeuristicsWeight);
   }

   /**
    * Gets the weight for the heuristics in the Body path based planner.
    */
   default double getBodyPathBasedHeuristicsWeight()
   {
      return getStoredPropertySet().getValue(bodyPathBasedHeuristicsWeight);
   }

   /**
    * When using a cost based planning approach this value defined how the yaw of a footstep will be
    * weighted in comparison to its position.
    */
   default double getYawWeight()
   {
      return getStoredPropertySet().getValue(yawWeight);
   }

   /**
    * <p>
    * This value defined how the forward (or backward) displacement of a footstep will be weighted in
    * comparison to its position.
    * </p>
    * <p>
    *    Note that when using a Euclidean distance, this weight is averaged with the value returned by
    *    {@link #getLateralWeight()}
    * </p>
    */
   default double getForwardWeight()
   {
      return getStoredPropertySet().getValue(forwardWeight);
   }

   /**
    * <p>
    * This value defined how the lateral displacement of a footstep will be weighted in comparison to
    * its position.
    * </p>
    * <p>
    *    Note that when using a Euclidean distance, this weight is averaged with the value returned by
    *    {@link #getForwardWeight()}
    * </p>
    */
   default double getLateralWeight()
   {
      return getStoredPropertySet().getValue(lateralWeight);
   }

   /**
    * When using a cost based planning approach this value defines the cost that is added for each step
    * taken. Setting this value to a high number will favor plans with less steps.
    */
   default double getCostPerStep()
   {
      return getStoredPropertySet().getValue(costPerStep);
   }

   /**
    * When using a cost based planning approach this value defines how the height change when stepping
    * up will be weighted.
    */
   default double getStepUpWeight()
   {
      return getStoredPropertySet().getValue(stepUpWeight);
   }

   /**
    * When using a cost based planning approach this value defines how the height change when stepping
    * down will be weighted.
    */
   default double getStepDownWeight()
   {
      return getStoredPropertySet().getValue(stepDownWeight);
   }

   /**
    * When using a cost based planning approach this value defines how the roll will be weighted.
    */
   default double getRollWeight()
   {
      return getStoredPropertySet().getValue(rollWeight);
   }

   /**
    * When using a cost based planning approach this value defines how the pitch will be weighted.
    */
   default double getPitchWeight()
   {
      return getStoredPropertySet().getValue(pitchWeight);
   }

   /**
    * If this value is non-zero, nodes will be given cost if the bounding box is within this xy distance of a planar region
    * @see BipedFootstepPlannerCostParametersReadOnly#getBoundingBoxCost
    */
   default double getMaximum2dDistanceFromBoundingBoxToPenalize()
   {
      return getStoredPropertySet().getValue(maximum2dDistanceFromBoundingBoxToPenalize);
   }

   /**
    * If a node doesn't have bounding box collisions at the default dimensions, but does when increasing the xy dimensions by d,
    * where d < getMaximum2DDistanceFromBoundingBoxToPenalize, there will be a cost given to the node of:
    * {@code c * (1 - d / d_max)}, where d_max is this value.
    */
   default double getBoundingBoxCost()
   {
      return getStoredPropertySet().getValue(boundingBoxCost);
   }
}
