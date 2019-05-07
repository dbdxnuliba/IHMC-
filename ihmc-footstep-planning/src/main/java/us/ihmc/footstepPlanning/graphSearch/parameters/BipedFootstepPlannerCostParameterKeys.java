package us.ihmc.footstepPlanning.graphSearch.parameters;

import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;
import us.ihmc.tools.property.StoredPropertySet;

public class BipedFootstepPlannerCostParameterKeys
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList("bipedFootstepPlannerCostParameters");

   public static final BooleanStoredPropertyKey useQuadraticDistanceCost                   = keys.addBooleanKey("Use quadratic distance cost"                          );
   public static final BooleanStoredPropertyKey useQuadraticHeightCost                     = keys.addBooleanKey("Use quadratic height cost"                            );
   public static final DoubleStoredPropertyKey  aStarHeuristicsWeight                      = keys.addDoubleKey ("AStar heuristics weight"                          );
   public static final DoubleStoredPropertyKey  visGraphWithAStarHeuristicsWeight          = keys.addDoubleKey ("Vis graph with AStar heuristics weight"           );
   public static final DoubleStoredPropertyKey  depthFirstHeuristicsWeight                 = keys.addDoubleKey ("Depth first heuristics weight"                    );
   public static final DoubleStoredPropertyKey  bodyPathBasedHeuristicsWeight              = keys.addDoubleKey ("Body path based heuristics weight"                );
   public static final DoubleStoredPropertyKey  yawWeight                                  = keys.addDoubleKey ("Yaw weight"                                       );
   public static final DoubleStoredPropertyKey  forwardWeight                              = keys.addDoubleKey ("Forward weight"                                   );
   public static final DoubleStoredPropertyKey  lateralWeight                              = keys.addDoubleKey ("Lateral weight"                                   );
   public static final DoubleStoredPropertyKey  costPerStep                                = keys.addDoubleKey ("Cost per step"                                    );
   public static final DoubleStoredPropertyKey  stepUpWeight                               = keys.addDoubleKey ("Step up weight"                                   );
   public static final DoubleStoredPropertyKey  stepDownWeight                             = keys.addDoubleKey ("Step down weight"                                 );
   public static final DoubleStoredPropertyKey  rollWeight                                 = keys.addDoubleKey ("Roll weight"                                      );
   public static final DoubleStoredPropertyKey  pitchWeight                                = keys.addDoubleKey ("Pitch weight"                                     );
   public static final DoubleStoredPropertyKey  maximum2dDistanceFromBoundingBoxToPenalize = keys.addDoubleKey ("Maximum 2D distance from bounding box to penalize");
   public static final DoubleStoredPropertyKey  boundingBoxCost                            = keys.addDoubleKey ("Bounding box cost"                                );

   public static void main(String[] args)
   {
      StoredPropertySet.printInitialSaveFileContents(keys.keys());
   }
}
