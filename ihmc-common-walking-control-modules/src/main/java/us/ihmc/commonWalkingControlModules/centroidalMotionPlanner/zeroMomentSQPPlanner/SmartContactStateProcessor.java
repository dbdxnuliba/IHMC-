package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import java.util.ArrayList;
import java.util.List;

import afu.org.checkerframework.checker.units.qual.s;
import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.robotics.lists.RecyclingArrayList;
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

   public SmartContactStateProcessor(String namePrefix, YoVariableRegistry registry)
   {
      numberOfSegmentsPerSupportChange = new YoInteger(namePrefix + "NumberOfSegmentsPerSupportChange", registry);
   }

   public void initialize(int numberOfSegmentsPerSupportChange)
   {
      this.numberOfSegmentsPerSupportChange.set(numberOfSegmentsPerSupportChange);
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
         segment.setContactState(firstContactState);
         return;
      }
      // Process the first state
      boolean isStateSupported = firstContactState.isSupported();
      ContactState nextState = contactStatesToProcess.get(1);
      boolean isNextStateSupported = nextState.isSupported();
      getSegmentDurations(firstContactState.getDuration(), isStateSupported, isStateSupported, isNextStateSupported, segmentTimesForContactState);
      createSegments(segmentListToPopulate, segmentTimesForContactState, firstContactState);
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
         createSegments(segmentListToPopulate, segmentTimesForContactState, state);
      }
      state = nextState;
      isPreviousStateSupported = isStateSupported;
      isStateSupported = isNextStateSupported;
      getSegmentDurations(state.getDuration(), isStateSupported, isPreviousStateSupported, isStateSupported, segmentTimesForContactState);
      createSegments(segmentListToPopulate, segmentTimesForContactState, state);
   }

   private void createSegments(RecyclingArrayList<CollinearForceMotionPlannerSegment> segmentListToPopulate, List<Double> segmentDurations,
                               ContactState associatedContactState)
   {
      for (int i = 0; i < segmentDurations.size(); i++)
      {
         CollinearForceMotionPlannerSegment segmentToAdd = segmentListToPopulate.add();
         segmentToAdd.setContactState(associatedContactState);
         segmentToAdd.setSegmentDuration(segmentDurations.get(i));
      }
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
}
