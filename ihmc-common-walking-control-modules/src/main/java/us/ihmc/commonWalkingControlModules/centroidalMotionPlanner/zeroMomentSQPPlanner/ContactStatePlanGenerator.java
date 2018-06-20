package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import java.util.List;

import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * A helper class to help with generation of contact state plans 
 * for the {@code CollinearForceBasedCoMMotionPlanner}. Methods are provided to generate 
 * contact state plans given some basic details of the footstep plans as well as 
 * convert more complex footstep plans and generate the corresponding contact state plans
 * Note : Can only handle flat ground cases 
 * @author Apoorv S
 * 
 */
public class ContactStatePlanGenerator
{
   private final FramePose2D tempPoseForLeftFoot = new FramePose2D();
   private final FramePose2D tempPoseForRightFoot = new FramePose2D();

   public static void setContactStatesFromPoses(ContactState contactStateToPopulate, double duration,
                                                SideDependentList<? extends FramePose2DReadOnly> anklePoses,
                                                SideDependentList<ConvexPolygon2D> footSupportPolygonsInAnkleFrame)
   {
      setContactStatesFromPoses(contactStateToPopulate, duration, anklePoses.get(RobotSide.LEFT), anklePoses.get(RobotSide.RIGHT),
                                footSupportPolygonsInAnkleFrame.get(RobotSide.LEFT), footSupportPolygonsInAnkleFrame.get(RobotSide.RIGHT));
   }

   public static void setContactStatesFromPoses(List<ContactState> contactStateList, List<Double> durations,
                                                List<SideDependentList<? extends FramePose2DReadOnly>> anklePoses,
                                                SideDependentList<ConvexPolygon2D> footSupportPolygonsInAnkleFrame)
   {
      if (contactStateList.size() < anklePoses.size())
         throw new IllegalArgumentException("Contact state list does not have enough elements to store all processed support polygons");
      for (int i = 0; i < anklePoses.size(); i++)
         setContactStatesFromPoses(contactStateList.get(i), durations.get(i), anklePoses.get(i), footSupportPolygonsInAnkleFrame);
   }

   public static void setContactStatesFromPoses(ContactState contactStateToPopulate, double duration, FramePose2DReadOnly leftAnklePose,
                                                FramePose2DReadOnly rightAnklePose, ConvexPolygon2D leftFootSupportPolygon,
                                                ConvexPolygon2D rightFootSupportPolygon)
   {
      boolean isLeftFootSupported = leftAnklePose != null && !leftAnklePose.containsNaN() && leftFootSupportPolygon.getNumberOfVertices() > 0;
      boolean isRightFootSupported = rightAnklePose != null && !rightAnklePose.containsNaN() && rightFootSupportPolygon.getNumberOfVertices() > 0;
      contactStateToPopulate.reset();
      contactStateToPopulate.setDuration(duration);
      if (isLeftFootSupported)
      {
         RobotSide side = RobotSide.LEFT;
         contactStateToPopulate.setFootInContact(side, isLeftFootSupported);
         contactStateToPopulate.setFootPose(side, leftAnklePose);
         contactStateToPopulate.setSupportPolygon(side, leftFootSupportPolygon);
      }
      else
         contactStateToPopulate.setFootInContact(RobotSide.LEFT, isLeftFootSupported);
      if (isRightFootSupported)
      {
         RobotSide side = RobotSide.RIGHT;
         contactStateToPopulate.setFootInContact(side, isRightFootSupported);
         contactStateToPopulate.setFootPose(side, rightAnklePose);
         contactStateToPopulate.setSupportPolygon(side, rightFootSupportPolygon);
      }
      else
         contactStateToPopulate.setFootInContact(RobotSide.RIGHT, isRightFootSupported);
   }

   public static void setContactStateForSingleSupport(ContactState contactStateToPopulate, double duration, RobotSide supportSide,
                                                      FramePose2DReadOnly supportSidePose, ConvexPolygon2D supportFootPolygon)
   {
      contactStateToPopulate.reset();
      contactStateToPopulate.setDuration(duration);
      contactStateToPopulate.setFootInContact(supportSide, true);
      contactStateToPopulate.setFootInContact(supportSide.getOppositeSide(), false);
      contactStateToPopulate.setFootPose(supportSide, supportSidePose);
      contactStateToPopulate.setSupportPolygon(supportSide, supportFootPolygon);
   }

   public static void setContactStateForNoSupport(ContactState contactStateToPoulate, double duration)
   {
      contactStateToPoulate.reset();
      contactStateToPoulate.setDuration(duration);
      contactStateToPoulate.setFootInContact(false, false);
   }

   public static void setContactStateForDoubleSupport(ContactState contactStateToPopulate, double duration, FramePose2DReadOnly leftAnklePose,
                                                      FramePose2DReadOnly rightAnklePose, ConvexPolygon2D leftFootSupportPolygon,
                                                      ConvexPolygon2D rightFootSupportPolygon)
   {
      contactStateToPopulate.reset();
      contactStateToPopulate.setDuration(duration);
      contactStateToPopulate.setFootInContact(true, true);
      contactStateToPopulate.setFootPose(RobotSide.LEFT, leftAnklePose);
      contactStateToPopulate.setFootPose(RobotSide.RIGHT, rightAnklePose);
      contactStateToPopulate.setSupportPolygon(RobotSide.LEFT, leftFootSupportPolygon);
      contactStateToPopulate.setSupportPolygon(RobotSide.RIGHT, rightFootSupportPolygon);
   }

   public static void setContactStateForDoubleSupport(ContactState contactStateToPopulate, double duration, RobotSide nextSupportSide,
                                                      FramePose2DReadOnly nextSupportSidePose, ConvexPolygon2D nextSupportSideSupportPolygon,
                                                      FramePose2DReadOnly nextSwingSidePose, ConvexPolygon2D nextSwingSideSupportPolygon)
   {
      RobotSide nextSwingSide = nextSupportSide.getOppositeSide();
      contactStateToPopulate.reset();
      contactStateToPopulate.setDuration(duration);
      contactStateToPopulate.setFootInContact(true, true);
      contactStateToPopulate.setFootPose(nextSupportSide, nextSupportSidePose);
      contactStateToPopulate.setFootPose(nextSwingSide, nextSwingSidePose);
      contactStateToPopulate.setSupportPolygon(nextSupportSide, nextSupportSideSupportPolygon);
      contactStateToPopulate.setSupportPolygon(nextSwingSide, nextSwingSideSupportPolygon);
   }

   /**
    * Generates a contact state plan for a walking gait from the specified footstep poses. No checks are performed on the pose locations
    * @param footstepSolePoses the list of sole poses. In case {@code startInDoubleSupport} flag is true then the 
    * current poses are considered ordered as initial swing foot pose followed by initial support foot pose
    * @param contactStateList the list to which the generated contact states will be appended
    * @param leftFootSupportPolygon defined in the sole frame 
    * @param rightFootSupportPolygon defined in the sole frame
    * @param firstSupportSide the first pose is considered to be for this side. 
    * @param startInDoubleSupport indicates whether the first contact state is a double / single support state
    * 
    * @return number of contact states planned
    */
   public static int processFootstepPlanForWalking(List<FramePose2D> footstepSolePoses, List<ContactState> contactStateList,
                                                    ConvexPolygon2D leftFootSupportPolygon, ConvexPolygon2D rightFootSupportPolygon, RobotSide firstSupportSide,
                                                    boolean startInDoubleSupport, boolean endInDoubleSupport, double singleSupportDuration,
                                                    double doubleSupportDuration)
   {
      int poseIndex = 0;
      int contactStateIndex = 0;
      ConvexPolygon2D polygon1, polygon2;
      if (firstSupportSide == RobotSide.RIGHT)
      {
         polygon1 = leftFootSupportPolygon;
         polygon2 = rightFootSupportPolygon;
      }
      else
      {
         polygon1 = rightFootSupportPolygon;
         polygon2 = leftFootSupportPolygon;
      }
      FramePose2D pose1 = footstepSolePoses.get(poseIndex++);
      RobotSide supportSide = firstSupportSide;
      if (!startInDoubleSupport)
      {
         ContactState contactState = contactStateList.get(contactStateIndex++);
         contactState.reset();
         setContactStateForSingleSupport(contactState, singleSupportDuration, supportSide, pose1, polygon2);
         ConvexPolygon2D polyRef = polygon1;
         polygon1 = polygon2;
         polygon2 = polyRef;
         supportSide = supportSide.getOppositeSide();
      }
      FramePose2D pose2 = null;
      int loopCount = endInDoubleSupport ? footstepSolePoses.size() - 1 : footstepSolePoses.size();
      for (; poseIndex < loopCount; poseIndex++)
      {
         pose2 = footstepSolePoses.get(poseIndex);
         ContactState doubleSupportState = contactStateList.get(contactStateIndex++);
         setContactStateForDoubleSupport(doubleSupportState, doubleSupportDuration, supportSide, pose2, polygon2, pose1, polygon1);
         ContactState singleSupportState = contactStateList.get(contactStateIndex++);
         setContactStateForSingleSupport(singleSupportState, singleSupportDuration, supportSide, pose2, polygon2);
         pose1 = pose2;
         ConvexPolygon2D polyRef = polygon1;
         polygon1 = polygon2;
         polygon2 = polyRef;
         supportSide = supportSide.getOppositeSide();
      }
      if (endInDoubleSupport)
      {
         pose2 = footstepSolePoses.get(poseIndex);
         ContactState doubleSupportState = contactStateList.get(contactStateIndex++);
         setContactStateForDoubleSupport(doubleSupportState, doubleSupportDuration, supportSide, pose2, polygon2, pose1, polygon1);
      }
      return contactStateIndex;
   }

   /**
    * Generates a contact state plan for running motions
    * @param footstepPoses the list of foot poses to be converted to a contact state plan. In case the {@code startInDoubleSupport} flag is true this list should 
    * start with the initial pose of the first swing foot. Otherwise, should start with the pose of the first support foot
    * @param contactStateList the list of contact states to be populated
    * @param leftFootSupportPolygon the default support polygon of the left foot
    * @param rightFootSupportPolygon the default support polygon of the right foot
    * @param firstSupportSide the first support side for the running gait
    * @param flightDuration the duration of a flight phase
    * @param singleSupportDuration the duration of a singleSupportPhase
    * @param doubleSupportDuration the duration of a doubleSupportPhase. Not used in case no double support is to be planned
    * @param startInDoubleSupport 
    * @param startInSingleSupport
    * @param endInDoubleSupport
    * @param endInSingleSupport
    * 
    * @return number of contact states planned
    */
   public static int processFootstepPlanForRunning(List<FramePose2D> footstepPoses, List<ContactState> contactStateList,
                                                    ConvexPolygon2D leftFootSupportPolygon, ConvexPolygon2D rightFootSupportPolygon, RobotSide firstSupportSide,
                                                    double flightDuration, double singleSupportDuration, double doubleSupportDuration,
                                                    boolean startInDoubleSupport, boolean startInSingleSupport, boolean endInDoubleSupport,
                                                    boolean endInSingleSupport)
   {
      if ((startInDoubleSupport && startInSingleSupport) || (endInDoubleSupport && endInSingleSupport))
         throw new RuntimeException("Incompatible arguments. Please specify the state correctly");
      int poseIndex = 0;
      int contactStateIndex = 0;
      ConvexPolygon2D polygon1, polygon2;
      FramePose2D pose1, pose2;
      if (firstSupportSide == RobotSide.LEFT)
      {
         polygon1 = leftFootSupportPolygon;
         polygon2 = rightFootSupportPolygon;
      }
      else
      {
         polygon1 = rightFootSupportPolygon;
         polygon2 = leftFootSupportPolygon;
      }
      RobotSide supportSide = firstSupportSide;
      pose1 = footstepPoses.get(poseIndex++);
      if (startInDoubleSupport)
      {
         pose2 = footstepPoses.get(poseIndex++);
         ContactState initialDoubleSupportState = contactStateList.get(contactStateIndex++);
         setContactStateForDoubleSupport(initialDoubleSupportState, doubleSupportDuration, supportSide.getOppositeSide(), pose1, polygon2, pose2, polygon1);
         pose1 = pose2;
      }
      else if (!startInSingleSupport)
      {
         ContactState initialFlightState = contactStateList.get(contactStateIndex++);
         setContactStateForNoSupport(initialFlightState, flightDuration);
      }
      int loopCount = endInDoubleSupport ? footstepPoses.size() - 1 : footstepPoses.size();
      for (; poseIndex < loopCount; poseIndex++)
      {
         pose2 = footstepPoses.get(poseIndex);
         ContactState singleSupportState = contactStateList.get(contactStateIndex++);
         setContactStateForSingleSupport(singleSupportState, singleSupportDuration, supportSide, pose1, polygon1);
         ContactState flightState = contactStateList.get(contactStateIndex++);
         setContactStateForNoSupport(flightState, flightDuration);
         pose1 = pose2;
         ConvexPolygon2D polyRef = polygon1;
         polygon1 = polygon2;
         polygon2 = polyRef;
         supportSide = supportSide.getOppositeSide();
      }
      if (endInSingleSupport || endInDoubleSupport)
      {
         ContactState lastSingleSupportState = contactStateList.get(contactStateIndex++);
         setContactStateForSingleSupport(lastSingleSupportState, singleSupportDuration, supportSide, pose1, polygon1);
      }
      else
      {
         ContactState singleSupportState = contactStateList.get(contactStateIndex++);
         setContactStateForSingleSupport(singleSupportState, singleSupportDuration, supportSide, pose1, polygon1);
         ContactState flightState = contactStateList.get(contactStateIndex++);
         setContactStateForNoSupport(flightState, flightDuration);
      }
      if (endInDoubleSupport)
      {
         pose2 = footstepPoses.get(poseIndex);
         ContactState lastDoubleSupportState = contactStateList.get(contactStateIndex++);
         setContactStateForDoubleSupport(lastDoubleSupportState, doubleSupportDuration, supportSide, pose1, polygon1, pose2, polygon2);
      }
      return contactStateIndex;
   }

   /**
    * Populates the pose list with a series of alternating footsteps
    * @param framePosesToSet
    * @param initialLeftAnklePose
    * @param initialRightAnklePose
    * @param stepSize
    * @param numberOfSteps
    * @param startSide
    * @param stopAtEnd
    */
   public void generateAlternatingFootstepPoses(List<FramePose2D> framePosesToSet, FramePose2DReadOnly initialLeftAnklePose,
                                                FramePose2DReadOnly initialRightAnklePose, Vector2DReadOnly stepSize, int numberOfSteps, RobotSide startSide,
                                                boolean stopAtEnd)
   {
      int poseIndex = 0;
      if (startSide == RobotSide.LEFT)
      {
         tempPoseForLeftFoot.setIncludingFrame(initialLeftAnklePose);
         tempPoseForRightFoot.setIncludingFrame(initialRightAnklePose);
      }
      else
      {
         tempPoseForLeftFoot.setIncludingFrame(initialRightAnklePose);
         tempPoseForRightFoot.setIncludingFrame(initialLeftAnklePose);
      }
      framePosesToSet.get(poseIndex++).setIncludingFrame(tempPoseForLeftFoot);
      framePosesToSet.get(poseIndex++).setIncludingFrame(tempPoseForRightFoot);
      FramePose2D pose1 = tempPoseForLeftFoot;
      FramePose2D pose2 = tempPoseForRightFoot;
      for (int i = 0; i < numberOfSteps - 1; i++)
      {
         pose1.appendTranslation(stepSize.getX(), stepSize.getY());
         framePosesToSet.get(poseIndex++).setIncludingFrame(pose1);
         pose2.appendTranslation(stepSize.getX(), stepSize.getY());
         FramePose2D tempRef = pose1;
         pose1 = pose2;
         pose2 = tempRef;
      }
      if (!stopAtEnd)
         pose1.appendTranslation(stepSize.getX(), stepSize.getY());
      framePosesToSet.get(poseIndex++).setIncludingFrame(tempPoseForLeftFoot);
   }

   public int generateContactStatePlanForJumping(List<ContactState> contactStatesToSet, FramePose2DReadOnly initialLeftAnklePose,
                                                  FramePose2DReadOnly initialRightAnklePose, Vector2DReadOnly stepSize, int numberOfJumps,
                                                  ConvexPolygon2D leftFootSupportPolygon, ConvexPolygon2D rightFootSupportPolygon, double supportDuration,
                                                  double flightDuration)
   {
      int contactStateIndex = 0;
      tempPoseForLeftFoot.setIncludingFrame(initialLeftAnklePose);
      tempPoseForRightFoot.setIncludingFrame(initialRightAnklePose);
      for (int i = 0; i < numberOfJumps; i++)
      {
         ContactState groundState = contactStatesToSet.get(contactStateIndex++);
         setContactStateForDoubleSupport(groundState, supportDuration, tempPoseForLeftFoot, tempPoseForRightFoot, leftFootSupportPolygon,
                                         rightFootSupportPolygon);
         ContactState flightState = contactStatesToSet.get(contactStateIndex++);
         setContactStateForNoSupport(flightState, flightDuration);
         tempPoseForLeftFoot.appendTranslation(stepSize.getX(), stepSize.getY());
         tempPoseForRightFoot.appendTranslation(stepSize.getX(), stepSize.getY());
      }
      ContactState groundState = contactStatesToSet.get(contactStateIndex++);
      setContactStateForDoubleSupport(groundState, supportDuration, tempPoseForLeftFoot, tempPoseForRightFoot, leftFootSupportPolygon, rightFootSupportPolygon);
      return contactStateIndex;
   }

   private final RecyclingArrayList<FramePose2D> tempFramePoses = new RecyclingArrayList<>(FramePose2D.class);

   public int generateContactStatePlanForWalking(List<ContactState> contactStatesToSet, FramePose2DReadOnly initialLeftAnklePose,
                                                  FramePose2DReadOnly initialRightAnklePose, Vector2DReadOnly stepSize, int numberOfSteps,
                                                  ConvexPolygon2D leftFootSupportPolygon, ConvexPolygon2D rightFootSupportPolygon, RobotSide firstSupportSide,
                                                  boolean startInDoubleSupport, boolean endInDoubleSupport, double singleSupportDuration,
                                                  double doubleSupportDuration)
   {
      populateTempPoses(numberOfSteps + 2);
      generateAlternatingFootstepPoses(tempFramePoses, initialLeftAnklePose, initialRightAnklePose, stepSize, numberOfSteps,
                                       startInDoubleSupport ? firstSupportSide.getOppositeSide() : firstSupportSide, endInDoubleSupport);
      return processFootstepPlanForWalking(tempFramePoses, contactStatesToSet, leftFootSupportPolygon, rightFootSupportPolygon, firstSupportSide, startInDoubleSupport,
                                    endInDoubleSupport, singleSupportDuration, doubleSupportDuration);
   }

   public int generateContactStatePlanForRunning(List<ContactState> contactStatesToSet, FramePose2DReadOnly initialLeftAnklePose,
                                                  FramePose2DReadOnly initialRightAnklePose, Vector2DReadOnly stepSize, int numberOfSteps,
                                                  ConvexPolygon2D leftFootSupportPolygon, ConvexPolygon2D rightFootSupportPolygon, RobotSide firstSupportSide,
                                                  boolean startInDoubleSupport, boolean startInSingleSupport, boolean endInDoubleSupport,
                                                  boolean endInSingleSupport, double flightDuration, double singleSupportDuration, double doubleSupportDuration)
   {
      populateTempPoses(numberOfSteps + 2);
      generateAlternatingFootstepPoses(tempFramePoses, initialLeftAnklePose, initialRightAnklePose, stepSize, numberOfSteps,
                                       startInDoubleSupport ? firstSupportSide.getOppositeSide() : firstSupportSide, endInDoubleSupport);
      return processFootstepPlanForRunning(tempFramePoses, contactStatesToSet, leftFootSupportPolygon, rightFootSupportPolygon, firstSupportSide, flightDuration,
                                    singleSupportDuration, doubleSupportDuration, startInDoubleSupport, startInSingleSupport, endInDoubleSupport,
                                    endInSingleSupport);
   }

   private void populateTempPoses(int numberOfPoses)
   {
      tempFramePoses.clear();
      for (int i = 0; i < numberOfPoses; i++)
         tempFramePoses.add().setToNaN();
   }
}
