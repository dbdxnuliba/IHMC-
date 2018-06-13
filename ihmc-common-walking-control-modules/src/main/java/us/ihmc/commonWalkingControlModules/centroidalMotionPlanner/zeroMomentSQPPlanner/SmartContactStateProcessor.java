package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.commonWalkingControlModules.controlModules.flight.TransformHelperTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Processes the contact states for the {@code CollinearForceBasedCoMMotionPlanner} in an effort to minimize the 
 * number of decision variables that the planner needs to determine. This is done by selecting an appropriate 
 * discretization of the contact states.
 * @author Apoorv S
 */
public class SmartContactStateProcessor
{
   private final List<Double> nodeTimesForContactState = new ArrayList<>();
   private final List<Double> segmentTimesForContactState = new ArrayList<>();
   private final YoInteger numberOfSegmentsPerSupportChange;
   private final double defaultPrecision = 1e-5;
   private final ReferenceFrame planningFrame;
   private final FramePose3D tempPose = new FramePose3D();
   private final ConvexPolygon2D tempSupportPolygon = new ConvexPolygon2D();

   private final ArrayList<Point2D> tempVertices = new ArrayList<>();

   public SmartContactStateProcessor(ReferenceFrame planningFrame, String namePrefix, YoVariableRegistry registry)
   {
      this.planningFrame = planningFrame;
      numberOfSegmentsPerSupportChange = new YoInteger(namePrefix + "NumberOfSegmentsPerSupportChange", registry);
   }

   public void initialize(int numberOfSegmentsPerSupportChange, int maxNumberOfSupportPolygonVertices)
   {
      this.numberOfSegmentsPerSupportChange.set(numberOfSegmentsPerSupportChange);
      for (int i = 0; i < maxNumberOfSupportPolygonVertices; i++)
         tempVertices.add(new Point2D());
   }

   public void processContactStates(List<ContactState> contactStatesToProcess, RecyclingArrayList<CollinearForceMotionPlannerSegment> segmentListToPopulate)
   {
      int numberOfContactStates = contactStatesToProcess.size();
      if (numberOfContactStates < 1)
         throw new RuntimeException("Cannot process an empty contact state list");

      segmentListToPopulate.clear();
      ContactState firstContactState = contactStatesToProcess.get(0);
      if (numberOfContactStates == 1)
      {
         CollinearForceMotionPlannerSegment segment = segmentListToPopulate.add();
         segment.setSegmentDuration(firstContactState.getDuration());
         getSupportPolygon(segment.supportPolygon, firstContactState);
         return;
      }
      // Process the first state
      boolean isStateSupported = firstContactState.isSupported();
      ContactState nextState = contactStatesToProcess.get(1);
      boolean isNextStateSupported = nextState.isSupported();
      getSegmentDurations(firstContactState.getDuration(), isStateSupported, isStateSupported, isNextStateSupported, segmentTimesForContactState);
      getSupportPolygon(tempSupportPolygon, firstContactState);
      createSegments(segmentListToPopulate, segmentTimesForContactState, tempSupportPolygon);
      boolean isPreviousStateSupported;
      ContactState state;
      for (int i = 1; i < numberOfContactStates - 1; i++)
      {
         isPreviousStateSupported = isStateSupported;
         isStateSupported = isNextStateSupported;
         state = nextState;
         nextState = contactStatesToProcess.get(i + 1);
         isNextStateSupported = nextState.isSupported();
         getSegmentDurations(state.getDuration(), isStateSupported, isPreviousStateSupported, isNextStateSupported, segmentTimesForContactState);
         getSupportPolygon(tempSupportPolygon, state);
         createSegments(segmentListToPopulate, segmentTimesForContactState, tempSupportPolygon);
      }
      state = nextState;
      isPreviousStateSupported = isStateSupported;
      isStateSupported = isNextStateSupported;
      getSegmentDurations(state.getDuration(), isStateSupported, isPreviousStateSupported, isStateSupported, segmentTimesForContactState);
      getSupportPolygon(tempSupportPolygon, state);
      createSegments(segmentListToPopulate, segmentTimesForContactState, tempSupportPolygon);
   }

   private void createSegments(RecyclingArrayList<CollinearForceMotionPlannerSegment> segmentListToPopulate, List<Double> segmentDurations,
                               ConvexPolygon2D associatedSupportPolygon)
   {
      for (int i = 0; i < segmentDurations.size(); i++)
      {
         CollinearForceMotionPlannerSegment segmentToAdd = segmentListToPopulate.add();
         segmentToAdd.setSupportPolygon(associatedSupportPolygon);
         segmentToAdd.setSegmentDuration(segmentDurations.get(i));
      }
   }

   private void getSupportPolygon(ConvexPolygon2D supportPolygonToSet, ContactState contactState)
   {
      SideDependentList<Boolean> footInContact = contactState.footInContact;
      SideDependentList<ConvexPolygon2D> footSupportPolygons = contactState.footSupportPolygons;
      SideDependentList<FramePose3D> footPoses = contactState.footPoses;
      int numberOfVertices = 0;
      int listIndex = 0;
      for(RobotSide side : RobotSide.values)
      {
         if(footInContact.get(side))
         {
            ConvexPolygon2D supportPolygon = footSupportPolygons.get(side);
            numberOfVertices += supportPolygon.getNumberOfVertices();
            tempPose.setIncludingFrame(footPoses.get(side));
            tempPose.changeFrame(planningFrame);
            for (int i = 0; i < supportPolygon.getNumberOfVertices(); i++)
            {
               Point2D vertex = tempVertices.get(listIndex++);
               vertex.set(supportPolygon.getVertex(i));
               TransformHelperTools.transformFromPoseToReferenceFrameByProjection(tempPose, vertex);
            }
         }
      }
      generateMinimalVertexSupportPolygon(supportPolygonToSet, tempVertices, numberOfVertices, defaultPrecision);
   }

   private void getSegmentDurations(double contactStateDuration, boolean isStateSupported, boolean isPreviousStateSupported, boolean isNextStateSupported,
                                    List<Double> segmentDurationsToPopulate)
   {
      if (isStateSupported)
         getSegmentDurations(contactStateDuration, isPreviousStateSupported != isStateSupported, isNextStateSupported != isStateSupported,
                             segmentDurationsToPopulate);
      else
         getSegmentDurations(contactStateDuration, false, false, segmentDurationsToPopulate);
   }

   private void getSegmentDurations(double contactStateDuration, boolean expandFirstHalf, boolean expandLastHalf, List<Double> segmentDurationsToPopulate)
   {
      segmentDurationsToPopulate.clear();
      if (!(expandFirstHalf || expandLastHalf))
      {
         segmentDurationsToPopulate.add(contactStateDuration);
         return;
      }
      double halfStateDuration = contactStateDuration / 2.0;
      nodeTimesForContactState.clear();
      nodeTimesForContactState.add(0.0);
      int numberOfNodesToGenerate = numberOfSegmentsPerSupportChange.getIntegerValue() - 1;
      if (expandFirstHalf)
         appendChebyshevNodesBiasedTowardsIntervalBeginning(nodeTimesForContactState, 0.0, halfStateDuration, numberOfNodesToGenerate);
      nodeTimesForContactState.add(halfStateDuration);
      if (expandLastHalf)
         appendChebyshevNodesBiasedTowardsIntervalEnd(nodeTimesForContactState, halfStateDuration, contactStateDuration, numberOfNodesToGenerate);
      nodeTimesForContactState.add(contactStateDuration);
      for (int i = 0; i < nodeTimesForContactState.size() - 1; i++)
         segmentDurationsToPopulate.add(nodeTimesForContactState.get(i + 1) - nodeTimesForContactState.get(i));
   }

   /**
    * Populates the list provided with roots of the nth Chebyshev polynomial of the first kind
    * @param listToPopulate the list in which computed values are stored. The list is not cleared and entries are simply appended using the {@code List#add()} method
    * @param n the order of the Chebyshev polynomial or number of nodes to generate
    */
   public static void generateChebyshevNodes(List<Double> listToPopulate, int n)
   {
      for (int i = n; i > 0; i--)
         listToPopulate.add(Math.cos((2.0 * i - 1.0) / (2.0 * n) * Math.PI));
   }

   /**
    * Populates the list by mapping roots of the nth Chebyshev polynomial of the first kind to the interval (x0, xf)
    * @param listToPopulate the list in which computed values are stored. The list is not cleared and entries are simply appended using the {@code List#add()} method
    * @param x0 initial value for interval
    * @param xF final value for interval
    * @param n the order of the Chebyshev polynomial or number of nodes to generate
    */
   public static void generateChebyshevNodesWithArbitraryRange(List<Double> listToPopulate, double x0, double xF, int n)
   {
      for (int i = n; i > 0; i--)
         listToPopulate.add(((Math.cos((2.0 * i - 1.0) / (2.0 * n) * Math.PI)) * (xF - x0) + xF + x0) * 0.5);
   }

   /**
    * @param listToPopulate
    * @param x0 first node value
    * @param xF last node value
    * @param n the number of nodes to generate
    */
   public static void generateChebyshevNodesAndIncludeEndPoints(List<Double> listToPopulate, double x0, double xF, int n)
   {
      listToPopulate.add(x0);
      generateChebyshevNodesWithArbitraryRange(listToPopulate, x0, xF, n - 2);
      listToPopulate.add(xF);
   }

   /**
    * 
    * @param listToPopulate
    * @param x0
    * @param xF
    * @param n
    */
   public static void appendChebyshevNodesBiasedTowardsIntervalBeginning(List<Double> listToPopulate, double x0, double xF, int n)
   {
      for (int i = 2 * n; i > n; i--)
         listToPopulate.add((Math.cos((2.0 * i - 1.0) / (4.0 * n) * Math.PI) * (xF - x0) + (xF)));
   }

   /**
    * 
    * @param listToPopulate
    * @param x0
    * @param xF
    * @param n
    */
   public static void appendChebyshevNodesBiasedTowardsIntervalEnd(List<Double> listToPopulate, double x0, double xF, int n)
   {
      for (int i = n; i > 0; i--)
         listToPopulate.add((Math.cos((2.0 * i - 1.0) / (4.0 * n) * Math.PI) * (xF - x0) + (x0)));
   }

   private static void roundToPrecision(ArrayList<Point2D> vertexList, double precision)
   {
      for (int i = 0; i < vertexList.size(); i++)
      {
         Point2D vertexToRound = vertexList.get(i);
         double newX = MathTools.roundToPrecision(vertexToRound.getX(), precision);
         double newY = MathTools.roundToPrecision(vertexToRound.getY(), precision);
         vertexToRound.set(newX, newY);
      }
   }

   public void combinePolygons(ConvexPolygon2D polygonToSet, FramePose3DReadOnly pose1, ConvexPolygon2D polygon1, FramePose3DReadOnly pose2,
                               ConvexPolygon2D polygon2)
   {
      int numberOfVertices = polygon1.getNumberOfVertices() + polygon2.getNumberOfVertices();
      if (numberOfVertices > tempVertices.size())
         throw new RuntimeException("Insufficient temporary variables for computation");
      tempPose.setIncludingFrame(pose1);
      tempPose.changeFrame(planningFrame);
      for (int i = 0; i < polygon1.getNumberOfVertices(); i++)
      {
         Point2D vertex = tempVertices.get(i);
         vertex.set(polygon2.getVertex(i));
         TransformHelperTools.transformFromPoseToReferenceFrameByProjection(tempPose, vertex);
      }
      tempPose.setIncludingFrame(pose2);
      tempPose.changeFrame(planningFrame);
      for (int i = 0; i < polygon1.getNumberOfVertices(); i++)
      {
         Point2D vertex = tempVertices.get(i);
         vertex.set(polygon2.getVertex(i));
         TransformHelperTools.transformFromPoseToReferenceFrameByProjection(tempPose, vertex);
      }
      generateMinimalVertexSupportPolygon(polygonToSet, tempVertices, numberOfVertices, defaultPrecision);
   }

   public static void generateMinimalVertexSupportPolygon(ConvexPolygon2D polygonToSet, ArrayList<Point2D> vertexList, int numberOfVertices, double precision)
   {
      roundToPrecision(vertexList, precision);
      if (numberOfVertices == 0)
      {
         polygonToSet.clear();
         polygonToSet.update();
         return;
      }
      // Generate the minimal vertex polygon. New gift wrapping algorithm
      // Get the max X max Y element. 
      int candidateVertexIndex = 0;
      for (int i = 1; i < numberOfVertices; i++)
      {
         if (vertexList.get(i).getX() > vertexList.get(candidateVertexIndex).getX())
            candidateVertexIndex = i;
         else if (vertexList.get(i).getX() == vertexList.get(candidateVertexIndex).getX()
               && vertexList.get(i).getY() > vertexList.get(candidateVertexIndex).getY())
            candidateVertexIndex = i;
      }
      // Place the top right vertex at the beginning of list
      Point2D topRightVertex = vertexList.get(candidateVertexIndex);
      Point2D firstVertex = vertexList.get(0);
      vertexList.set(0, topRightVertex);
      vertexList.set(candidateVertexIndex, firstVertex);
      // Start the marching
      for (int i = 1; i < numberOfVertices; i++)
      {
         Point2D lastComputedPoint = vertexList.get(i - 1);
         Point2D candidatePoint = vertexList.get(i);
         // Find the next one 
         for (int j = i + 1; j < numberOfVertices; j++)
         {
            Point2D pointUnderConsideration = vertexList.get(j);
            double det = (pointUnderConsideration.getY() - lastComputedPoint.getY()) * (candidatePoint.getX() - lastComputedPoint.getX())
                  - (pointUnderConsideration.getX() - lastComputedPoint.getX()) * (candidatePoint.getY() - lastComputedPoint.getY());
            boolean swap = det > 0.0 || (det == 0.0 && lastComputedPoint.distance(pointUnderConsideration) > lastComputedPoint.distance(candidatePoint));
            if (swap)
            {
               vertexList.set(j, candidatePoint);
               vertexList.set(i, pointUnderConsideration);
               candidatePoint = pointUnderConsideration;
            }
         }
         double det2 = (topRightVertex.getY() - lastComputedPoint.getY()) * (candidatePoint.getX() - lastComputedPoint.getX())
               - (topRightVertex.getX() - lastComputedPoint.getX()) * (candidatePoint.getY() - lastComputedPoint.getY());
         boolean terminate = det2 > 0.0 || (det2 == 0.0 && lastComputedPoint.distance(candidatePoint) < lastComputedPoint.distance(topRightVertex));
         if (terminate)
         {
            numberOfVertices = i;
            break;
         }
      }
      polygonToSet.setAndUpdate(vertexList, numberOfVertices);
   }

}
