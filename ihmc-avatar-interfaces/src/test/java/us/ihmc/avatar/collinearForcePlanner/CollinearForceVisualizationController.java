package us.ihmc.avatar.collinearForcePlanner;

import java.awt.Color;
import java.util.List;

import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController.CentroidalModelTools;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner.CollinearForceBasedCoMMotionPlanner;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner.CollinearForceBasedPlannerResult;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner.CollinearForceMotionPlannerSegment;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner.CollinearForcePlannerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.commonWalkingControlModules.controlModules.flight.TransformHelperTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.Collision.Surface.Contact;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.examples.centroidalDynamicsRobot.CentroidalRobotController;
import us.ihmc.simulationconstructionset.examples.centroidalDynamicsRobot.CentroidalStateReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class CollinearForceVisualizationController extends CentroidalRobotController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final String namePrefix = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(namePrefix + "Registry");
   private final YoFrameVector gravity;
   private final YoDouble mass;
   private YoDouble time;
   private final YoDouble lastStateChange;

   private final CollinearForceBasedCoMMotionPlanner motionPlanner;
   private final CollinearForceBasedPlannerResult sqpOutput;
   private final CentroidalRobotControllerCore controllerCore;
   private CentroidalStateReadOnly state;
   private final FramePoint3D estimatedCoM = new FramePoint3D();
   //private final YoFramePoint desiredCoM;
   //private final YoFrameVector desiredCoMAcceleration;
   private YoFramePoint desiredCoP;
   private YoFrameVector desiredGroundReactionForce;

   private final SideDependentList<YoFramePose> solePoses;
   private final SideDependentList<ConvexPolygon2D> defaultSupportPolygons;
   private final SideDependentList<YoBoolean> footOnGround = new SideDependentList<YoBoolean>();
   private final SideDependentList<YoFrameConvexPolygon2d> feetPolygonsInWorldFrame = new SideDependentList<YoFrameConvexPolygon2d>();
   private final YoFrameConvexPolygon2d netSupportPolygon;

   // YoVariables 
   private final YoFramePoint plannedCoM;
   private final YoFramePoint plannedCoP;
   private final YoFramePoint dynamicsCoM;
   private final YoFramePoint dynamicsCoP;

   // Graphics stuff
   private final boolean updateGraphics;
   private BagOfBalls plannedCoMTrack;
   private BagOfBalls plannedCoPTrack;
   private BagOfBalls dynamicsCoMTrack;
   private BagOfBalls dynamicsCoPTrack;
   private final SideDependentList<Color> feetPolygonColor = new SideDependentList<Color>(new Color(0.85f, 0.35f, 0.65f, 1.0f),
                                                                                          new Color(0.15f, 0.8f, 0.15f, 1.0f));

   // Variables for calculations
   private final FrameVector3D tempVector = new FrameVector3D();
   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FrameConvexPolygon2d tempPolygon = new FrameConvexPolygon2d();
   private final FramePose3D tempPose = new FramePose3D();

   public CollinearForceVisualizationController(SideDependentList<YoFramePose> solePoses, SideDependentList<ConvexPolygon2D> feetSupportPolygonInSoleFrame,
                                                YoGraphicsListRegistry graphicsListRegistry)
   {
      this.lastStateChange = new YoDouble(namePrefix + "LastStateChange", registry);
      this.mass = new YoDouble(namePrefix + "Mass", registry);
      this.gravity = new YoFrameVector(namePrefix + "Gravity", worldFrame, registry);
      this.solePoses = solePoses;
      this.defaultSupportPolygons = feetSupportPolygonInSoleFrame;
      netSupportPolygon = new YoFrameConvexPolygon2d("SupportPolygon", worldFrame, defaultSupportPolygons.get(RobotSide.LEFT).getNumberOfVertices()
            + defaultSupportPolygons.get(RobotSide.RIGHT).getNumberOfVertices(), registry);
      for (RobotSide side : RobotSide.values)
         footOnGround.set(side, new YoBoolean(side.getCamelCaseName() + "FootOnGround", registry));

      motionPlanner = new CollinearForceBasedCoMMotionPlanner(registry);
      controllerCore = createControllerCore(graphicsListRegistry);
      sqpOutput = motionPlanner.getSQPSolution();
      plannedCoM = new YoFramePoint("PlannedCoM", worldFrame, registry);
      plannedCoP = new YoFramePoint("PlannedCoP", worldFrame, registry);
      dynamicsCoM = new YoFramePoint("DynamicsCoM", worldFrame, registry);
      dynamicsCoP = new YoFramePoint("DynamicsCoP", worldFrame, registry);

      updateGraphics = graphicsListRegistry != null;
      if (updateGraphics)
         createVisualization(graphicsListRegistry);
   }

   private CentroidalRobotControllerCore createControllerCore(YoGraphicsListRegistry graphicsListRegistry)
   {
      CentroidalRobotControllerCore controllerCore = new CentroidalRobotControllerCore(defaultSupportPolygons, registry, graphicsListRegistry);
      controllerCore.setRobotMass(mass);
      controllerCore.setGravity(gravity);
      controllerCore.setCoefficientOfFriction(1.0);
      controllerCore.setComputeFootPolygonFromFootPose(false);
      controllerCore.setAngularMomentumWeights(new FrameVector3D(worldFrame, 0.1, 0.1, 0.1));
      controllerCore.setLinearMomentumWeights(new FrameVector3D(worldFrame, 1.0, 1.0, 1.0));
      return controllerCore;
   }

   public void setControllerTime(YoDouble controllerTime)
   {
      time = controllerTime;
   }

   private void createVisualization(YoGraphicsListRegistry graphicsListRegistry)
   {

      plannedCoMTrack = new BagOfBalls(100, 0.001, "PlannedCoM", new YoAppearanceRGBColor(Color.BLACK, 0.0), registry, graphicsListRegistry);
      plannedCoPTrack = new BagOfBalls(100, 0.001, "PlannedCoP", new YoAppearanceRGBColor(Color.RED, 0.0), registry, graphicsListRegistry);
      dynamicsCoMTrack = new BagOfBalls(100, 0.001, "DynamicsCoM", new YoAppearanceRGBColor(Color.WHITE, 0.0), registry, graphicsListRegistry);
      dynamicsCoPTrack = new BagOfBalls(100, 0.001, "DynamicsCoP", new YoAppearanceRGBColor(Color.ORANGE, 0.0), registry, graphicsListRegistry);

      YoGraphicsList pointList = new YoGraphicsList("ControlPoints");
      ArtifactList pointArtifactList = new ArtifactList("ControlPoints");
      YoGraphicPosition plannedCoMViz = new YoGraphicPosition("plannedCoMViz", plannedCoM, 0.005, new YoAppearanceRGBColor(Color.BLACK, 0.0), GraphicType.BALL);
      pointList.add(plannedCoMViz);
      pointArtifactList.add(plannedCoMViz.createArtifact());
      YoGraphicPosition plannedCoPViz = new YoGraphicPosition("plannedCoPViz", plannedCoP, 0.005, new YoAppearanceRGBColor(Color.RED, 0.0), GraphicType.BALL);
      pointList.add(plannedCoPViz);
      pointArtifactList.add(plannedCoPViz.createArtifact());
      YoGraphicPosition dynamicsCoMViz = new YoGraphicPosition("dynamicsCoMViz", dynamicsCoM, 0.005, new YoAppearanceRGBColor(Color.WHITE, 0.0),
                                                               GraphicType.BALL_WITH_ROTATED_CROSS);
      pointList.add(dynamicsCoMViz);
      pointArtifactList.add(dynamicsCoMViz.createArtifact());
      YoGraphicPosition dynamicsCoPViz = new YoGraphicPosition("dynamicsCoPViz", dynamicsCoP, 0.005, new YoAppearanceRGBColor(Color.ORANGE, 0.0),
                                                               GraphicType.BALL_WITH_ROTATED_CROSS);
      pointList.add(dynamicsCoPViz);
      pointArtifactList.add(dynamicsCoPViz.createArtifact());
      graphicsListRegistry.registerArtifactList(pointArtifactList);
      graphicsListRegistry.registerYoGraphicsList(pointList);

      ArtifactList supportPolygonArtifactList = new ArtifactList("ControllerPolygon");
      for (RobotSide side : RobotSide.values)
      {
         String polygonName = side.getCamelCaseName() + "FootSupportPolygon";
         YoFrameConvexPolygon2d footPolygon = new YoFrameConvexPolygon2d(polygonName, worldFrame, defaultSupportPolygons.get(side).getNumberOfVertices(),
                                                                         registry);
         YoArtifactPolygon footPolygonForVisualization = new YoArtifactPolygon(polygonName, footPolygon, feetPolygonColor.get(side), false);
         feetPolygonsInWorldFrame.put(side, footPolygon);
         supportPolygonArtifactList.add(footPolygonForVisualization);
      }
      YoArtifactPolygon footPolygonForVisualization = new YoArtifactPolygon("SupportPolygon", netSupportPolygon, Color.PINK, false);
      supportPolygonArtifactList.add(footPolygonForVisualization);
      graphicsListRegistry.registerArtifactList(supportPolygonArtifactList);
   }

   public void setupController(double robotMass, FrameVector3DReadOnly gravity)
   {
      this.mass.set(robotMass);
      tempVector.setIncludingFrame(gravity);
      tempVector.changeFrame(worldFrame);
      this.gravity.set(tempVector);

      motionPlanner.initialize(new CollinearForcePlannerParameters(), tempVector);
   }

   private void updateVisualization()
   {
      plannedCoMTrack.setBallLoop(plannedCoM);
      plannedCoPTrack.setBallLoop(plannedCoP);
      dynamicsCoMTrack.setBallLoop(dynamicsCoM);
      dynamicsCoPTrack.setBallLoop(dynamicsCoP);
   }

   private void updateYoVariables()
   {
      for (RobotSide side : RobotSide.values)
      {
         tempPolygon.clear();
         YoFramePose solePose = solePoses.get(side);

         footOnGround.get(side).set(solePose.getZ() < 1e-4);
         if (footOnGround.get(side).getBooleanValue())
         {
            tempPolygon.setIncludingFrame(solePose.getReferenceFrame(), defaultSupportPolygons.get(side));
            solePose.getFramePose(tempPose);
            TransformHelperTools.transformFromPoseToReferenceFrameByProjection(tempPose, tempPolygon.getGeometryObject());
            tempPolygon.changeFrame(worldFrame);
            feetPolygonsInWorldFrame.get(side).setFrameConvexPolygon2d(tempPolygon);
         }
      }
      tempPolygon.clear();
      for (RobotSide side : RobotSide.values)
      {
         if (footOnGround.get(side).getBooleanValue())
            tempPolygon.addVertices(feetPolygonsInWorldFrame.get(side).getFrameConvexPolygon2d());
      }
      tempPolygon.update();
      netSupportPolygon.setFrameConvexPolygon2d(tempPolygon);
   }

   @Override
   public void initialize()
   {
      if (state == null)
         throw new RuntimeException("Estimated state has not been provided to the controller");
      initializeForStanding();
   }

   private void initializeForStanding()
   {
      desiredGroundReactionForce.set(gravity);
      desiredGroundReactionForce.scale(-mass.getDoubleValue());
      state.getPosition(tempPoint);
      desiredCoP.set(tempPoint.getX(), tempPoint.getY(), 0.0);
   }

   private final FrameVector3D desiredAngularMomentumRateOfChange = new FrameVector3D(worldFrame, 0.0, 0.0, 0.0);
   private final FrameVector3D desiredLinearMomentumRateOfChange = new FrameVector3D();
   private final boolean useControllerCoreOutput = true;

   @Override
   public void doControl()
   {
      double timeInState = time.getDoubleValue() - lastStateChange.getDoubleValue();
      sqpOutput.compute(timeInState);
      state.getPosition(estimatedCoM);
      runControllerCore(estimatedCoM, achievedLinearMomentumRateOfChange, achivedAngularMomentumRateOfChange);
      plannedCoP.set(sqpOutput.getDesiredCoPPosition());
      plannedCoM.set(sqpOutput.getDesiredCoMPosition());

      if (useControllerCoreOutput)
      {
         CentroidalModelTools.computeGroundReactionForce(achievedLinearMomentumRateOfChange, gravity, mass.getDoubleValue(), computedGroundReactionForce);
         CentroidalModelTools.computeCenterOfPressureForFlatGround(estimatedCoM, 0.0, computedGroundReactionForce, achivedAngularMomentumRateOfChange,
                                                                   computedCenterOfPressure);
         desiredCoP.set(computedCenterOfPressure);
         desiredGroundReactionForce.set(computedGroundReactionForce);
      }
      else
      {
         desiredCoP.set(plannedCoP);
         desiredGroundReactionForce.set(sqpOutput.getDesiredGroundReactionForce());
      }

      updateYoVariables();
      if (updateGraphics)
         updateVisualization();
   }

   private final FrameVector3D achievedLinearMomentumRateOfChange = new FrameVector3D();
   private final FrameVector3D achivedAngularMomentumRateOfChange = new FrameVector3D();
   private final FrameVector3D computedGroundReactionForce = new FrameVector3D();
   private final FramePoint3D computedCenterOfPressure = new FramePoint3D();
   private final FrameVector3D previousAngularMomentumRateOfChange = new FrameVector3D();
   private final FrameVector3D previousLinearMomentumRateOfChange = new FrameVector3D();
   private void runControllerCore(FramePoint3DReadOnly centerOfMass, FrameVector3D linearMomentumRateOfChange, FrameVector3D angularMomentumRateOfChange)
   {
      int segmentIndex = sqpOutput.getCurrentSegmentIndex();
      CollinearForceMotionPlannerSegment segment = motionPlanner.getSegmentList().get(segmentIndex); 
      controllerCore.reset();
      controllerCore.setCenterOfMassLocation(centerOfMass);
      tempPolygon.setToZero(worldFrame);
      segment.getSupportPolygon(tempPolygon.getGeometryObject());
      controllerCore.setSupportPolygon(tempPolygon);
      desiredLinearMomentumRateOfChange.setIncludingFrame(sqpOutput.getDynamicsCoMAcceleration());
      desiredLinearMomentumRateOfChange.scale(mass.getDoubleValue());
      controllerCore.setDesiredLinearMomentumRateOfChange(desiredLinearMomentumRateOfChange);
      controllerCore.setDesiredAngularMomentumRateOfChange(desiredAngularMomentumRateOfChange);
      controllerCore.compute();
      controllerCore.getAchievedLinearMomentumRateOfChange(linearMomentumRateOfChange);
      controllerCore.getAchievedAngularMomentumRateOfChange(angularMomentumRateOfChange);
      if(linearMomentumRateOfChange.containsNaN())
         linearMomentumRateOfChange.setIncludingFrame(previousLinearMomentumRateOfChange);
      else
         previousLinearMomentumRateOfChange.setIncludingFrame(linearMomentumRateOfChange);
      if(angularMomentumRateOfChange.containsNaN())
         angularMomentumRateOfChange.setIncludingFrame(previousAngularMomentumRateOfChange);
      else
         previousAngularMomentumRateOfChange.setIncludingFrame(angularMomentumRateOfChange);
      //controllerCore.setFeetLocations(solePoses.get(RobotSide.LEFT).getPosition(), solePoses.get(RobotSide.RIGHT).getPosition());
      //controllerCore.updateFootContactState(footOnGround.get(RobotSide.LEFT).getBooleanValue(), footOnGround.get(RobotSide.RIGHT).getBooleanValue());
   }

   @Override
   public void setOutputVariables(YoFramePoint desiredCoPToSet, YoFrameVector desiredExternalForce)
   {
      desiredCoP = desiredCoPToSet;
      desiredGroundReactionForce = desiredExternalForce;
   }

   @Override
   public void setCurrentState(CentroidalStateReadOnly stateToSet)
   {
      state = stateToSet;
   }

   @Override
   public YoVariableRegistry getRegistry()
   {
      return registry;
   }

   private final FramePoint3D initialCoMPosition = new FramePoint3D();
   private final FrameVector3D initialCoMVelocity = new FrameVector3D();
   private final FramePoint3D initialCoPPosition = new FramePoint3D();
   private final FramePoint3D finalCoMPosition = new FramePoint3D();
   private final FrameVector3D finalCoMVelocity = new FrameVector3D();
   private final FramePoint3D finalCoPPosition = new FramePoint3D();

   public void submitFootstepPlan()
   {
      
   }
   
   public void submitContactStateList(List<ContactState> contactStatePlanForController)
   {
      motionPlanner.reset();
      for (int i = 0; i < contactStatePlanForController.size(); i++)
      {
         motionPlanner.appendContactStateToList(contactStatePlanForController.get(i));
      }
      state.getPosition(initialCoMPosition);
      state.getLinearVelocity(initialCoMVelocity);
      initialCoPPosition.setIncludingFrame(desiredCoP);
      motionPlanner.setInitialState(initialCoMPosition, initialCoMVelocity, initialCoPPosition);
      ContactState lastContactState = contactStatePlanForController.get(contactStatePlanForController.size() - 1);
      finalCoMPosition.setIncludingFrame(lastContactState.getPose(RobotSide.RIGHT).getPosition());
      finalCoMPosition.changeFrame(worldFrame);
      finalCoMPosition.addZ(0.435);
      finalCoMVelocity.setToZero(worldFrame);
      finalCoPPosition.setIncludingFrame(lastContactState.getPose(RobotSide.RIGHT).getPosition());
      motionPlanner.setFinalState(finalCoMPosition, finalCoMVelocity, finalCoPPosition);
   }

   public void runIteration()
   {
      motionPlanner.runIterations(1);
   }

   public void setTimeForStateChange(double controllerTime)
   {
      lastStateChange.set(controllerTime);
   }

   public boolean hasPlannerFailed()
   {
      return motionPlanner.hasPlannerFailed();
   }
}
