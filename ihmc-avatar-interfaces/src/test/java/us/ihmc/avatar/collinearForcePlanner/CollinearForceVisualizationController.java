package us.ihmc.avatar.collinearForcePlanner;

import java.awt.Color;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner.CollinearForceBasedCoMMotionPlanner;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner.CollinearForceBasedPlannerResult;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner.CollinearForcePlannerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.commonWalkingControlModules.controlModules.flight.TransformHelperTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.convexOptimization.quadraticProgram.JavaQuadProgSolver;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
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
   private CentroidalStateReadOnly state;
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
      sqpOutput = motionPlanner.getSQPSolution();
      plannedCoM = new YoFramePoint("PlannedCoM", worldFrame, registry);
      plannedCoP = new YoFramePoint("PlannedCoP", worldFrame, registry);
      dynamicsCoM = new YoFramePoint("DynamicsCoM", worldFrame, registry);
      dynamicsCoP = new YoFramePoint("DynamicsCoP", worldFrame, registry);

      updateGraphics = graphicsListRegistry != null;
      if (updateGraphics)
         createVisualization(graphicsListRegistry);
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
            TransformHelperTools.transformFromPoseToReferenceFrame(tempPose, tempPolygon.getGeometryObject());
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

   @Override
   public void doControl()
   {
      double timeInState = time.getDoubleValue() - lastStateChange.getDoubleValue();
      sqpOutput.compute(timeInState);

      plannedCoP.set(sqpOutput.getDesiredCoPPosition());
      plannedCoM.set(sqpOutput.getDesiredCoMPosition());
      desiredCoP.set(plannedCoP);
      desiredGroundReactionForce.set(sqpOutput.getDesiredGroundReactionForce());
      updateYoVariables();
      if (updateGraphics)
         updateVisualization();
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
   }

   public void runIteration()
   {
      motionPlanner.runIterations(1);
   }

   public void setTimeForStateChange(double controllerTime)
   {
      lastStateChange.set(controllerTime);
   }

   private final JavaQuadProgSolver qpSolver = new JavaQuadProgSolver();
   private final DenseMatrix64F solver_H = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F solver_f = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F solver_Aeq = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F solver_beq = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F solver_Ain = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F solver_bin = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F qpSoln = new DenseMatrix64F(0, 1);
}
