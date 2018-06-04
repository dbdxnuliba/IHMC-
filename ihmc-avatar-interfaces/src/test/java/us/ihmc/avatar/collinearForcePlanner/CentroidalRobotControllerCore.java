package us.ihmc.avatar.collinearForcePlanner;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commons.PrintTools;
import us.ihmc.convexOptimization.quadraticProgram.JavaQuadProgSolver;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Creates an equivalent of the controller core so that momentum rate of change
 * objectives can be defined for the system
 * 
 * @author Apoorv S
 *
 */
public class CentroidalRobotControllerCore
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry("ControllerCore");

   // Optimization outputs
   private final YoFrameVector achievedLinearMomentumRateOfChange;
   private final YoFrameVector achievedAngularMomentumRateOfChange;

   // Optimization inputs
   private final YoFramePoint currentCenterOfMass;
   private final YoFrameVector linearMomentumWeights;
   private final YoFrameVector angularMomentumWeights;
   private final YoFrameVector groundReactionWeights;
   private final YoFrameVector desiredLinearMomentumRateOfChange;
   private final YoFrameVector desiredAngularMomentumRateOfChange;
   private final SideDependentList<YoBoolean> footOnGround = new SideDependentList<>();
   private final SideDependentList<YoFramePoint> ankleLocation = new SideDependentList<>();
   private final SideDependentList<ConvexPolygon2D> defaultFootSupportPolygons;
   private final YoFrameConvexPolygon2d supportPolygon;
   private final YoDouble coefficientOfFriction;
   private final YoInteger numberOfSupportPolygonVertices;
   private final JavaQuadProgSolver qpSolver;

   // Solver control variables
   private final YoBoolean enableGroundReactionLimits;

   // Calculation variables 
   private final DenseMatrix64F solver_H = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F solver_f = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F solver_Ain = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F solver_bin = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F solver_Aeq = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F solver_beq = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F solver_lb = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F solver_ub = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F qpSolution = new DenseMatrix64F(0, 1);

   public CentroidalRobotControllerCore(SideDependentList<ConvexPolygon2D> defaultFootSupportPolygons, YoVariableRegistry parentRegistry,
                                        YoGraphicsListRegistry graphicsListRegistry)
   {
      String namePrefix = "ControllerCore";
      achievedLinearMomentumRateOfChange = new YoFrameVector(namePrefix + "AchievedLinearMomentumRateOfChange", worldFrame, registry);
      achievedAngularMomentumRateOfChange = new YoFrameVector(namePrefix + "AchievedAngularMomentumRateOfChange", worldFrame, registry);

      currentCenterOfMass = new YoFramePoint(namePrefix + "CenterOfMass", worldFrame, registry);
      desiredLinearMomentumRateOfChange = new YoFrameVector(namePrefix + "DesiredLinearMomentumRateOfChange", worldFrame, registry);
      desiredAngularMomentumRateOfChange = new YoFrameVector(namePrefix + "DesiredAngularMomentumRateOfChange", worldFrame, registry);
      this.defaultFootSupportPolygons = defaultFootSupportPolygons;
      int maxNumberOfVertices = 0;
      for (RobotSide side : RobotSide.values)
      {
         maxNumberOfVertices += defaultFootSupportPolygons.get(side).getNumberOfVertices();
         YoBoolean onGroundFlag = new YoBoolean(namePrefix + side.getCamelCaseNameForMiddleOfExpression() + "FootOnGround", registry);
         footOnGround.put(side, onGroundFlag);
         YoFramePoint ankleLocation = new YoFramePoint(namePrefix + side.getCamelCaseNameForMiddleOfExpression() + "AnkleLocation", worldFrame, registry);
         this.ankleLocation.put(side, ankleLocation);
      }
      supportPolygon = new YoFrameConvexPolygon2d(namePrefix + "SupportPolygon", worldFrame, maxNumberOfVertices, registry);
      numberOfSupportPolygonVertices = supportPolygon.getYoNumberVertices();
      enableGroundReactionLimits = new YoBoolean(namePrefix + "EnableGroundReactionForceLimits", registry);
      groundReactionWeights = new YoFrameVector(namePrefix + "GroundReactionWeights", worldFrame, registry);
      linearMomentumWeights = new YoFrameVector(namePrefix + "LinearMomentumOptimizationWeights", worldFrame, registry);
      angularMomentumWeights = new YoFrameVector(namePrefix + "AngularMomentumOptimizationWeights", worldFrame, registry);
      coefficientOfFriction = new YoDouble(namePrefix + "CoefficientOfFriction", registry);
      qpSolver = new JavaQuadProgSolver();
      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      qpSolver.clear();
   }

   public void compute()
   {
      processInputs();
      setupSolverMatrices();
      submitMatricesAndRunOptimization();
      putOptimizationResultsIntoYoVariables();
   }

   public void setCoefficientOfFriction(double coefficientOfFrictionToSet)
   {
      this.coefficientOfFriction.set(coefficientOfFrictionToSet);
      updateFrictionCone();
   }

   private final List<Vector3D> frictionConeBasisVectors = new ArrayList<>();
   private final int numberOfFrictionConeVectors = 4;

   // This creates garbage. But since these will not be updated during runtime I think this should be fine.
   private void updateFrictionCone()
   {
      frictionConeBasisVectors.clear();
      double angleBetweenVectors = Math.PI * 2 / numberOfFrictionConeVectors;
      double normalizationFactor = Math.sqrt(coefficientOfFriction.getDoubleValue() * coefficientOfFriction.getDoubleValue() + 1.0);
      double bZ = 1.0 / normalizationFactor;
      double bXY = coefficientOfFriction.getDoubleValue() / normalizationFactor;
      for (int i = 0; i < numberOfFrictionConeVectors; i++)
      {
         double vectorOrientation = i * angleBetweenVectors;
         double bX = Math.cos(vectorOrientation) * bXY;
         double bY = Math.sin(vectorOrientation) * bXY;
         frictionConeBasisVectors.add(new Vector3D(bX, bY, bZ));
      }
   }

   public double getCoefficientOfFriction()
   {
      return coefficientOfFriction.getDoubleValue();
   }

   public void setLinearMomentumWeights(FrameVector3DReadOnly linearMomentumWeights)
   {
      this.linearMomentumWeights.set(linearMomentumWeights);
      updateLinearMomentumWeightMatrix(this.linearMomentumWeights);
   }

   private void updateLinearMomentumWeightMatrix(FrameVector3DReadOnly linearMomentumWeights)
   {
      linearWeights.reshape(3, 3);
      linearWeights.zero();
      for (int i = 0; i < 3; i++)
         linearWeights.set(i, i, linearMomentumWeights.getElement(i));
   }

   public void setAngularMomentumWeights(FrameVector3DReadOnly angularMomentumWeights)
   {
      this.angularMomentumWeights.set(angularMomentumWeights);
      updateAngularMomentumWeightMatrix(this.angularMomentumWeights);
   }

   private void updateAngularMomentumWeightMatrix(FrameVector3DReadOnly angularMomentumWeights)
   {
      angularWeights.reshape(3, 3);
      angularWeights.zero();
      for (int i = 0; i < 3; i++)
         angularWeights.set(i, i, angularMomentumWeights.getElement(i));
   }

   public void setDesiredLinearMomentumRateOfChange(FrameVector3DReadOnly desiredLinearMomentumRateOfChange)
   {
      this.desiredLinearMomentumRateOfChange.set(desiredLinearMomentumRateOfChange);
   }

   public void setDesiredAngularMomentumRateOfChange(FrameVector3DReadOnly desiredAngularMomentumRateOfChange)
   {
      this.desiredAngularMomentumRateOfChange.set(desiredAngularMomentumRateOfChange);
   }

   public void getAchievedAngularMomentumRateOfChange(FrameVector3D angularMomentumRateOfChangeToSet)
   {
      angularMomentumRateOfChangeToSet.setIncludingFrame(this.achievedAngularMomentumRateOfChange);
   }

   public void getAchievedLinearMomentumRateOfChange(FrameVector3D linearMomentumRateOfChangeToSet)
   {
      linearMomentumRateOfChangeToSet.setIncludingFrame(this.achievedLinearMomentumRateOfChange);
   }

   public void setFeetLocations(FramePoint3DReadOnly leftSoleCentroid, FramePoint3DReadOnly rightSoleCentroid)
   {
      ankleLocation.get(RobotSide.LEFT).set(leftSoleCentroid);
      ankleLocation.get(RobotSide.RIGHT).set(rightSoleCentroid);
   }

   public void updateFootContactState(boolean isLeftFootInContact, boolean isRightFootInContact)
   {
      footOnGround.get(RobotSide.LEFT).set(isLeftFootInContact);
      footOnGround.get(RobotSide.RIGHT).set(isRightFootInContact);
   }
   
   public void setCenterOfMassLocation(FramePoint3DReadOnly centerOfMass)
   {
      currentCenterOfMass.set(centerOfMass);
   }

   private final ConvexPolygon2D supportPolygonForComputation = new ConvexPolygon2D();

   private void processInputs()
   {
      updateSupportPolygon();
   }

   private void updateSupportPolygon()
   {
      supportPolygonForComputation.clear();
      for (RobotSide side : RobotSide.values)
      {
         if (footOnGround.get(side).getBooleanValue())
         {
            ConvexPolygon2D footSupportPolygon = defaultFootSupportPolygons.get(side);
            FramePoint3DReadOnly vertexOffset = ankleLocation.get(side);
            for (int i = 0; i < footSupportPolygon.getNumberOfVertices(); i++)
            {
               Point2DReadOnly vertexToAdd = footSupportPolygon.getVertex(i);
               supportPolygonForComputation.addVertex(vertexToAdd.getX() + vertexOffset.getY(), vertexToAdd.getY() + vertexOffset.getY());
            }
         }
      }
      supportPolygonForComputation.update();
      supportPolygon.setConvexPolygon2d(supportPolygonForComputation);
   }

   private void setupSolverMatrices()
   {
      generateConstraints();
      generateObjective();
   }

   private final DenseMatrix64F linearMomentumJacobian = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F linearMomentumObjective = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F linearWeights = new DenseMatrix64F(0, 1);

   // The order of the decision variables (coordinates of the force vector 
   // in terms of the basis vectors ordered corresponding to the support polygon vertices)
   private void generateObjective()
   {
      int numberOfDecisionVariables = numberOfFrictionConeVectors * numberOfSupportPolygonVertices.getIntegerValue();
      solver_H.reshape(numberOfDecisionVariables, numberOfDecisionVariables);
      solver_f.reshape(numberOfDecisionVariables, 1);
      solver_H.zero();
      solver_f.zero();
      generateLinearMomentumRateOfChangeObjective();
      generateAngularMomentumRateOfChangeObjective();
      generateForceMinimizationObjective();
   }

   private final DenseMatrix64F tempJacobian = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempObjective = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempWeights = new DenseMatrix64F(0, 1);

   private void generateForceMinimizationObjective()
   {
      int numberOfForceVectors = numberOfFrictionConeVectors * numberOfSupportPolygonVertices.getIntegerValue();
      tempJacobian.reshape(numberOfForceVectors, numberOfForceVectors);
      tempObjective.reshape(numberOfForceVectors, 1);
      CommonOps.setIdentity(tempJacobian);
      tempObjective.zero();
      tempWeights.reshape(numberOfForceVectors, numberOfForceVectors);
      CommonOps.setIdentity(tempWeights);
      CommonOps.scale(0.001, tempWeights);
      addObjective(tempJacobian, tempObjective, tempWeights);
   }

   private void generateLinearMomentumRateOfChangeObjective()
   {
      linearMomentumJacobian.reshape(3, numberOfFrictionConeVectors * numberOfSupportPolygonVertices.getIntegerValue());
      for (int i = 0; i < numberOfSupportPolygonVertices.getIntegerValue(); i++)
      {
         for (int j = 0; j < numberOfFrictionConeVectors; j++)
         {
            for (int k = 0; k < 3; k++)
            {
               linearMomentumJacobian.set(k, i * numberOfFrictionConeVectors + j, frictionConeBasisVectors.get(j).getElement(k));
            }
         }
      }

      linearMomentumObjective.reshape(3, 1);
      desiredLinearMomentumRateOfChange.get(0, linearMomentumObjective);
      addObjective(linearMomentumJacobian, linearMomentumObjective, linearWeights);
   }

   private final Point3D tempX = new Point3D();
   private final Point3D tempV = new Point3D();
   private final Vector3D tempR = new Vector3D();
   private final Vector3D tempTau = new Vector3D();

   private final DenseMatrix64F angularMomentumJacobian = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F angularMomentumObjective = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F angularWeights = new DenseMatrix64F(0, 1);

   private final void generateAngularMomentumRateOfChangeObjective()
   {
      tempX.set(currentCenterOfMass);
      angularMomentumJacobian.reshape(3, numberOfFrictionConeVectors * numberOfSupportPolygonVertices.getIntegerValue());
      for (int i = 0; i < numberOfSupportPolygonVertices.getIntegerValue(); i++)
      {
         tempV.set(supportPolygon.getFrameVertex(i));
         tempR.sub(tempV, tempX);
         for (int j = 0; j < numberOfFrictionConeVectors; j++)
         {
            tempTau.cross(tempR, frictionConeBasisVectors.get(j));
            for (int k = 0; k < 3; k++)
               angularMomentumJacobian.set(k, i * numberOfFrictionConeVectors + j, tempTau.getElement(k));
         }
      }
      angularMomentumObjective.reshape(3, 1);
      desiredAngularMomentumRateOfChange.get(0, angularMomentumObjective);
      addObjective(angularMomentumJacobian, angularMomentumObjective, angularWeights);
   }

   private final DenseMatrix64F tempH = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempf = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempJtW = new DenseMatrix64F(0, 1);

   private void addObjective(DenseMatrix64F jacobian, DenseMatrix64F objective, DenseMatrix64F weight)
   {
      // (qT JT - cT)W (Jq - c) = qT JT W J q - qT JT W c - cT W Jq + cT W c 
      int numberOfDecisionVariables = numberOfFrictionConeVectors * numberOfSupportPolygonVertices.getIntegerValue();
      tempH.reshape(numberOfDecisionVariables, numberOfDecisionVariables);
      tempf.reshape(numberOfDecisionVariables, 1);
      tempJtW.reshape(jacobian.numCols, jacobian.numRows);
      CommonOps.multTransA(jacobian, weight, tempJtW);
      CommonOps.mult(tempJtW, objective, tempf);
      CommonOps.mult(tempJtW, jacobian, tempH);
      CommonOps.scale(-1.0, tempf);
      CommonOps.addEquals(solver_H, tempH);
      CommonOps.addEquals(solver_f, tempf);
   }

   private void generateConstraints()
   {
      // generate non negativity bounds 
      int numberOfDecisionVariables = numberOfFrictionConeVectors * numberOfSupportPolygonVertices.getIntegerValue();
      solver_Ain.reshape(numberOfDecisionVariables, numberOfDecisionVariables);
      CommonOps.setIdentity(solver_Ain);
      CommonOps.scale(-1.0, solver_Ain);
      solver_bin.reshape(numberOfDecisionVariables, 1);
      solver_bin.zero();
   }

   private void submitMatricesAndRunOptimization()
   {
      qpSolver.setQuadraticCostFunction(solver_H, solver_f, 0.0);
      // qpSolver.setLinearEqualityConstraints(solver_Aeq, solver_beq);
      qpSolver.setLinearInequalityConstraints(solver_Ain, solver_bin);
      //qpSolver.setUpperBounds(solver_ub);
      //qpSolver.setLowerBounds(solver_lb);
      qpSolver.solve(qpSolution);
   }

   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(0, 1);

   private void putOptimizationResultsIntoYoVariables()
   {
      tempMatrix.reshape(linearMomentumJacobian.numRows, qpSolution.numCols);
      CommonOps.mult(linearMomentumJacobian, qpSolution, tempMatrix);
      achievedLinearMomentumRateOfChange.set(tempMatrix);
      tempMatrix.reshape(angularMomentumJacobian.numRows, qpSolution.numCols);
      CommonOps.mult(angularMomentumJacobian, qpSolution, tempMatrix);
      achievedAngularMomentumRateOfChange.set(tempMatrix);
   }

   public List<? extends Vector3DReadOnly> getFrictionCone()
   {
      return frictionConeBasisVectors;
   }
}