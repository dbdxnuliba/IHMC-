package us.ihmc.commonWalkingControlModules.footController;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController.footControl.FootController;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.Vertex2DSupplierList;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoVariable;

public class FootControllerVisualizer
{
   private final static int bufferSize = 1 << 15;
   private final YoVariableRegistry registry = new YoVariableRegistry("FootControllerVisualizer");
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
   private final SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(bufferSize);
   private final SimulationConstructionSet scs;
   private final double ankleToToeX = 0.0725;
   private final double ankleToToeY = 0.0225;
   private final double ankleToMidX = 0.03625;
   private final double ankleToMidY = 0.04;
   private final double ankleToHeelX = 0.0175;
   private final double ankleToHeelY = 0.02;
   private final List<Point2D> defaultFootPolygonPointsInAnkleFrame = Stream.of(new Point2D(ankleToToeX, ankleToToeY), new Point2D(ankleToToeX, -ankleToToeY),
                                                                                new Point2D(ankleToMidX, -ankleToMidY),
                                                                                new Point2D(-ankleToHeelX, -ankleToHeelY),
                                                                                new Point2D(-ankleToHeelX, ankleToHeelY), new Point2D(ankleToMidX, ankleToMidY))
                                                                            .collect(Collectors.toList());
   private final ConvexPolygon2D defaultFootPolygon = new ConvexPolygon2D(new Vertex2DSupplierList<>(defaultFootPolygonPointsInAnkleFrame));
   private final double coefficientOfFriction = 1.0;
   private final int numberOfFrictionConeVectors = 6;

   private ArrayList<YoDouble> maxRho = new ArrayList<>();
   private YoPlaneContactState contactState;
   private final YoDouble yoTime;
   private FootController controller;

   public FootControllerVisualizer()
   {
      Robot dummyRobot = new Robot("Dummy");
      yoTime = dummyRobot.getYoTime();
      scaleDefaultPolygon(0.1);
      setupController();
      createFootGraphic();
      createFrictionConeGraphics();
      for (int i = 0; i < defaultFootPolygon.getNumberOfVertices(); i++)
         maxRho.get(i).set(.1);
      scs = new SimulationConstructionSet(dummyRobot, parameters);
      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.1);
      scs.addStaticLinkGraphics(linkGraphics);
      scs.startOnAThread();
   }

   private void scaleDefaultPolygon(double addDistance)
   {
      ConvexPolygonScaler scaler = new ConvexPolygonScaler();
      scaler.scaleConvexPolygon(defaultFootPolygon, -addDistance, defaultFootPolygon);

   }

   private void setupController()
   {
      RigidBody footSpoof = new RigidBody("FootSpoof", ReferenceFrame.getWorldFrame());
      List<FramePoint2D> framePointList = new ArrayList<>();
      for (int i = 0; i < defaultFootPolygon.getNumberOfVertices(); i++)
         framePointList.add(new FramePoint2D(footSpoof.getBodyFixedFrame(), defaultFootPolygon.getVertex(i)));
      contactState = new YoPlaneContactState("FootSpoof", footSpoof, footSpoof.getBodyFixedFrame(), framePointList, coefficientOfFriction, registry);
      for (int i = 0; i < defaultFootPolygon.getNumberOfVertices(); i++)
         maxRho.add(new YoDouble("Vertex" + i + "MaxRho", registry));
      controller = new FootController(yoTime, contactState, footSpoof.getBodyFixedFrame(), RobotSide.LEFT, registry);
      controller.setParameters(0.0, 0.1, 0.1);
   }

   private void createFootGraphic()
   {
      YoFrameConvexPolygon2D yoPolygon = new YoFrameConvexPolygon2D("YoFootPolygon", ReferenceFrame.getWorldFrame(), defaultFootPolygon.getNumberOfVertices(),
                                                                    registry);
      yoPolygon.addVertices(defaultFootPolygon);
      yoPolygon.update();
      YoGraphicPolygon graphicPolygon = new YoGraphicPolygon("FootPolygon", yoPolygon, registry, 1.0, new YoAppearanceRGBColor(Color.CYAN, 0.0));
      graphicsListRegistry.registerYoGraphic("FootPolygon", graphicPolygon);
   }

   private void createFrictionConeGraphics()
   {
      YoAppearanceRGBColor appearance = new YoAppearanceRGBColor(Color.RED, 0.0);
      List<Vector3D> vectorList = new ArrayList<>();
      double normalizingFactor = Math.sqrt(1.0 + coefficientOfFriction * coefficientOfFriction);
      double z = 1.0 / normalizingFactor;
      double xy = coefficientOfFriction / normalizingFactor;
      double deltaTheta = 2 * Math.PI / numberOfFrictionConeVectors;
      for (int i = 0; i < numberOfFrictionConeVectors; i++)
      {
         double x = xy * Math.cos(i * deltaTheta);
         double y = xy * Math.sin(i * deltaTheta);
         vectorList.add(new Vector3D(x, y, z));
      }

      int numberOfVertices = defaultFootPolygon.getNumberOfVertices();
      for (int i = 0; i < numberOfVertices; i++)
      {
         YoDouble length = maxRho.get(i);
         YoFramePoint3D yoVertex = new YoFramePoint3D("Vertex" + i, ReferenceFrame.getWorldFrame(), registry);
         yoVertex.set(defaultFootPolygon.getVertex(i));
         for (int j = 0; j < numberOfFrictionConeVectors; j++)
         {
            Vector3D vector = vectorList.get(j);
            YoFrameVector3D yoVector = new YoFrameVector3D("Vertex" + i + "Vector" + j, ReferenceFrame.getWorldFrame(), registry);
            length.addVariableChangedListener(new VariableChangedListener()
            {

               @Override
               public void notifyOfVariableChange(YoVariable<?> v)
               {
                  yoVector.set(vector);
                  yoVector.scale(length.getDoubleValue());
               }
            });
            YoGraphicVector graphicVector = new YoGraphicVector("Vertex" + i + "GraphicVector" + j, yoVertex, yoVector, 1.0, appearance, true);
            graphicsListRegistry.registerYoGraphic("FrictionConeVector", graphicVector);
         }
      }
   }

   public void run()
   {
      controller.requestTransitionToContact(0.1, true, false);
      double dt = 0.001;
      for (double t = 0; t < 1.0; t += dt)
      {
         yoTime.add(dt);
         controller.doControl();
         updateVectorViz();
         scs.tickAndUpdate();
      }
      controller.requestTransitionToContact(0.1, true, true);
      for (double t = 0; t < 1.0; t += dt)
      {
         yoTime.add(dt);
         controller.doControl();
         updateVectorViz();
         scs.tickAndUpdate();
      }
      controller.requestTransitionToContact(false, false);
      for (double t = 0; t < 1.0; t += dt)
      {
         yoTime.add(dt);
         controller.doControl();
         updateVectorViz();
         scs.tickAndUpdate();
      }
      controller.requestTransitionToContact(0.25, false, true);
      for (double t = 0; t < 0.05; t += dt)
      {
         yoTime.add(dt);
         controller.doControl();
         updateVectorViz();
         scs.tickAndUpdate();
      }
      controller.requestTransitionToContact(0.2, true, true);
      for (double t = 0; t < 1.0; t += dt)
      {
         yoTime.add(dt);
         controller.doControl();
         updateVectorViz();
         scs.tickAndUpdate();
      }
   }

   private void updateVectorViz()
   {
      for (int i = 0; i < contactState.getTotalNumberOfContactPoints(); i++)
      {
         //PrintTools.debug("Vertex " + i + ": " + contactState.isToeContactPoint(i) + ", " + contactState.isHeelContactPoint(i) + ", "
         //      + contactState.getMaxContactPointNormalForce(contactState.getContactPoints().get(i)));
         maxRho.get(i).set(contactState.getMaxContactPointNormalForce(contactState.getContactPoints().get(i)));
      }
   }

   public static void main(String args[])
   {
      FootControllerVisualizer visualizer = new FootControllerVisualizer();
      visualizer.run();
   }
}
