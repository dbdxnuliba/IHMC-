package us.ihmc.robotics.geometry;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.OrientationFrame;

//import us.ihmc.robotics.MathTools;
//import MatrixTools;

public abstract class FrameTupleTest<T extends Tuple3DBasics & GeometryObject<T>>
{
   private static final boolean VERBOSE = false;

   public static final double epsilon = 1e-10;

   public abstract FrameTuple3D<?, ?> createEmptyFrameTuple();

   public FrameTuple3D<?, ?> createFrameTuple(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      return createFrameTuple(referenceFrame, x, y, z);
   }

   public FrameTuple3D<?, ?> createFrameTuple(ReferenceFrame referenceFrame)
   {
      return createFrameTuple(referenceFrame, 0.0, 0.0, 0.0);
   }

   public FrameTuple3D<?, ?> createFrameTuple(ReferenceFrame referenceFrame, Tuple3DBasics tuple)
   {
      return createFrameTuple(referenceFrame, tuple.getX(), tuple.getY(), tuple.getZ());
   }

   public FrameTuple3D<?, ?> createFrameTuple(FrameTuple3D<?, ?> frameTuple)
   {
      return createFrameTuple(frameTuple.getReferenceFrame(), frameTuple.tuple);
   }

   protected ReferenceFrame theFrame = ReferenceFrame.constructARootFrame("theFrame");

   protected ReferenceFrame aFrame = ReferenceFrame.constructARootFrame("aFrame");

   protected RigidBodyTransform theFrameToChildFrame;

   protected ReferenceFrame childFrame;

   @Before
   public final void setUp() throws Exception
   {
      theFrameToChildFrame = new RigidBodyTransform();
      childFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("childFrame", theFrame, theFrameToChildFrame);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testSetXYZ()
   {
      FrameTuple3D<?, ?> frameTuple = createEmptyFrameTuple();

      Random random = new Random(15613L);

      RigidBodyTransform randomTransform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
      ReferenceFrame randomFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("childFrame", ReferenceFrame.getWorldFrame(), randomTransform);

      double x = RandomNumbers.nextDouble(random, -100.0, 100.0);
      double y = RandomNumbers.nextDouble(random, -100.0, 100.0);
      double z = RandomNumbers.nextDouble(random, -100.0, 100.0);

      frameTuple.set(x, y, z);
      testGetters(frameTuple, x, y, z);

      frameTuple.setIncludingFrame(randomFrame, x, y, z);
      assertEquals(randomFrame, frameTuple.getReferenceFrame());
      assertEquals(randomFrame, frameTuple.getReferenceFrame());
      testGetters(frameTuple, x, y, z);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = ReferenceFrameMismatchException.class) //Brett was here
   public final void testSetTuple()
   {
      FrameTuple3D<?, ?> frameTuple = createEmptyFrameTuple();
      Random random = new Random(15613L);
      RigidBodyTransform randomTransform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
      ReferenceFrame randomFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("childFrame", ReferenceFrame.getWorldFrame(), randomTransform);
      Tuple3DBasics randomTuple = RandomGeometry.nextPoint3D(random, 100.0, 100.0, 100.0);

      frameTuple.set(randomTuple);
      testGetters(frameTuple, randomTuple);

      frameTuple.setIncludingFrame(randomFrame, randomTuple);
      assertEquals(randomFrame, frameTuple.getReferenceFrame());
      assertEquals(randomFrame, frameTuple.getReferenceFrame());
      testGetters(frameTuple, randomTuple);

      //test non-matching reference frames
      FrameTuple3D<?, ?> ft1 = createFrameTuple(aFrame);
      FrameTuple3D<?, ?> ft2 = createFrameTuple(theFrame);
      ft1.set(ft2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testSetFrameTuple()
   {
      FrameTuple3D<?, ?> frameTuple = createEmptyFrameTuple();
      FrameTuple3D<?, ?> randomFrameTuple = createEmptyFrameTuple();

      Random random = new Random(15613L);

      RigidBodyTransform randomTransform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
      ReferenceFrame randomFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("childFrame", ReferenceFrame.getWorldFrame(), randomTransform);

      Tuple3DBasics randomTuple = RandomGeometry.nextPoint3D(random, 100.0, 100.0, 100.0);

      randomFrameTuple.setIncludingFrame(randomFrame, randomTuple);

      try
      {
         frameTuple.set(randomFrameTuple);
         fail("Should have thrown an exception");
      }
      catch (RuntimeException e)
      {
         // Good
      }

      frameTuple.setToZero(randomFrame);
      assertEquals(randomFrame, frameTuple.getReferenceFrame());

      frameTuple.set(randomFrameTuple);
      assertTuplesEqual(frameTuple, randomFrameTuple);

      frameTuple = createEmptyFrameTuple();

      frameTuple.setIncludingFrame(randomFrameTuple);
      assertTuplesEqual(frameTuple, randomFrameTuple);
   }

   private ReferenceFrame createRandomFrame(ReferenceFrame parentFrame, Random random)
   {
      FrameOrientation orientation = new FrameOrientation(parentFrame, createRandomDouble(random, -Math.PI / 3.0, Math.PI / 3.0), createRandomDouble(random, -Math.PI / 3.0, Math.PI / 3.0),
            createRandomDouble(random, -Math.PI / 3.0, Math.PI / 3.0));

      OrientationFrame frame = new OrientationFrame(orientation);

      return frame;
   }

   private double createRandomDouble(Random random, double min, double max)
   {
      return min + random.nextDouble() * (max - min);
   }

   private final void checkEquals(FrameTuple3D<?, ?> v1, FrameTuple3D<?, ?> v2)
   {
      if (!v1.epsilonEquals(v2, 1e-7))
      {
         throw new RuntimeException("Frame Vectors don't match. v1 = " + v1 + ", v2 = " + v2);
      }

   }

   private final void assertTuplesEqual(FrameTuple3D<?, ?> frameTupleToTest, FrameTuple3D<?, ?> frameTuple2)
   {
      testGetters(frameTupleToTest, frameTuple2.getPointCopy());
      assertEquals(frameTuple2.getReferenceFrame(), frameTupleToTest.getReferenceFrame());
      assertTrue(frameTupleToTest.epsilonEquals(frameTuple2, epsilon));
   }

   private final void testGetters(FrameTuple3D<?, ?> frameTuple, Tuple3DBasics tuple)
   {
      testGetters(frameTuple, tuple.getX(), tuple.getY(), tuple.getZ());
      assertTrue(frameTuple.epsilonEquals(tuple, epsilon));
   }

   private final void testGetters(FrameTuple3D<?, ?> frameTuple, double x, double y, double z)
   {
      assertEquals(x, frameTuple.getX(), epsilon);
      assertEquals(y, frameTuple.getY(), epsilon);
      assertEquals(z, frameTuple.getZ(), epsilon);

      Tuple3DBasics tuple3dToTest = new Point3D();
      frameTuple.get(tuple3dToTest);
      assertEquals(x, tuple3dToTest.getX(), epsilon);
      assertEquals(y, tuple3dToTest.getY(), epsilon);
      assertEquals(z, tuple3dToTest.getZ(), epsilon);

      assertEquals(x, frameTuple.getPointCopy().getX(), epsilon);
      assertEquals(y, frameTuple.getPointCopy().getY(), epsilon);
      assertEquals(z, frameTuple.getPointCopy().getZ(), epsilon);

      assertEquals(x, frameTuple.getVectorCopy().getX(), epsilon);
      assertEquals(y, frameTuple.getVectorCopy().getY(), epsilon);
      assertEquals(z, frameTuple.getVectorCopy().getZ(), epsilon);

      assertEquals(x, frameTuple.tuple.getX(), epsilon);
      assertEquals(y, frameTuple.tuple.getY(), epsilon);
      assertEquals(z, frameTuple.tuple.getZ(), epsilon);
   }

   // Tests copied from FramePointTest and FrameVectorTest

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testChangeFrameCopy()
   {
      Random random = new Random(1776L);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      FrameTuple3D<?, ?> vWorld = createFrameTuple(ReferenceFrame.getWorldFrame(), 1.0, 2.0, 3.0);

      ArrayList<ReferenceFrame> frames = new ArrayList<ReferenceFrame>();
      frames.add(worldFrame);

      int numberOfFrames = 10;
      ReferenceFrame parentFrame = worldFrame;

      for (int i = 0; i < numberOfFrames; i++)
      {
         ReferenceFrame frame = createRandomFrame(parentFrame, random);
         frames.add(frame);
         parentFrame = frame;
      }

      ArrayList<FrameTuple3D<?, ?>> resultVectors = new ArrayList<FrameTuple3D<?, ?>>();
      resultVectors.add(vWorld);

      // Choose random paths and move the vectors around those paths:
      int numVectors = 1000;

      for (int i = 0; i < numVectors; i++)
      {
         int pathLength = random.nextInt(20);

         FrameTuple3D<?, ?> vector = vWorld;

         for (int j = 0; j < pathLength; j++)
         {
            int frameIndex = random.nextInt(frames.size());
            vector.changeFrame(frames.get(frameIndex));
         }

         resultVectors.add(vector);
      }

      // Now compare all sets of 2 vectors. If they are in the same frame, they should have the same values
      for (FrameTuple3D<?, ?> resultVector1 : resultVectors)
      {
         // Print out the vectors:
         if (VERBOSE)
            //            System.out.println("resultVector1 = " + resultVector1);

            for (FrameTuple3D<?, ?> resultVector2 : resultVectors)
            {
               if (resultVector1.getReferenceFrame() == resultVector2.getReferenceFrame())
               {
                  checkEquals(resultVector1, resultVector2);
               }
            }
      }

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testSets() //Brett was here
   {
      FrameTuple3D<?, ?> alpha = createFrameTuple(ReferenceFrame.getWorldFrame(), 1.0, 2.0, 3.0);
      FrameTuple3D<?, ?> beta = createFrameTuple(ReferenceFrame.getWorldFrame(), 8.0, -2.0, 0.0);
      FrameTuple3D<?, ?> ones = createFrameTuple(ReferenceFrame.getWorldFrame(), 1.0, 1.0, 1.0);

      alpha.set(1.0, 1.0, 1.0);
      assertTrue("This should be true", alpha.epsilonEquals(ones, 1e-10));

      alpha.setX(-7.0);
      assertEquals("This should be equal", -7.0, alpha.getX(), 1e-10);

      alpha.setY(10.3);
      assertEquals("This should be equal", 10.3, alpha.getY(), 1e-10);

      alpha.setZ(1.9);
      assertEquals("This should be equal", 1.9, alpha.getZ(), 1e-10);

      alpha.set(10, 20, 30);
      assertEquals("This should be equal", 10, alpha.getX(), 1e-10);
      assertEquals("This should be equal", 20, alpha.getY(), 1e-10);
      assertEquals("This should be equal", 30, alpha.getZ(), 1e-10);

      alpha.setX(0);
      assertEquals("This should be equal", 0, alpha.getX(), 1e-10);

      alpha.set(beta);
      assertTrue("This should be true", alpha.epsilonEquals(beta, 1e-10));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = ReferenceFrameMismatchException.class) //Brett was here
   public final void testSetXY()
   {
      FrameTuple3D<?, ?> alpha = createFrameTuple(theFrame, 1.0, 2.0, 3.0);
      FramePoint2d framepoint2d = new FramePoint2d(theFrame);
      framepoint2d.set(-1.0, -2.0);

      alpha.setXY(framepoint2d);
      assertEquals("This should be equal", -1.0, alpha.getX(), epsilon);
      assertEquals("This should be equal", -2.0, alpha.getY(), epsilon);
      assertEquals("This should be equal", 0.0, alpha.getZ(), epsilon);

      //test non-matching reference frames
      FrameTuple3D<?, ?> ft1 = createFrameTuple(aFrame);
      ft1.setXY(framepoint2d);
   }

   // NaN was found in beta, commented out for further testing

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testCheckForNaN()
   {
      FrameTuple3D<?, ?> alpha = createFrameTuple(ReferenceFrame.getWorldFrame(), 6.0, 50.0, 2.0);
      FrameTuple3D<?, ?> beta = createFrameTuple(ReferenceFrame.getWorldFrame(), Math.sqrt(1.0), 2.0, 3.0);

      alpha.checkForNaN();
      beta.checkForNaN();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testGets()
   {
      FrameTuple3D<?, ?> alpha = createFrameTuple(ReferenceFrame.getWorldFrame(), 1.0, 2.0, 3.0);
      FrameTuple3D<?, ?> beta = createFrameTuple(ReferenceFrame.getWorldFrame(), 7.0, 0.0, -6.0);
      alpha.getX();
      beta.getY();
      beta.getZ();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testAddTuple3d()
   {
      FrameTuple3D<?, ?> alpha = createFrameTuple(ReferenceFrame.getWorldFrame(), 1.0, 2.0, 3.0);
      FrameTuple3D<?, ?> expected = createFrameTuple(ReferenceFrame.getWorldFrame(), 2.0, 3.0, 4.0);
      Point3D tuple1 = new Point3D(1.0, 1.0, 1.0);
      alpha.add(tuple1);
      assertTrue(alpha.epsilonEquals(expected, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testAddTuple3dTuple3d()
   {
      FrameTuple3D<?, ?> alpha = createFrameTuple(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);
      FrameTuple3D<?, ?> expected = createFrameTuple(ReferenceFrame.getWorldFrame(), 2.0, 2.0, 2.0);
      Point3D tuple1 = new Point3D(1.0, 1.0, 1.0);
      Point3D tuple2 = new Point3D(1.0, 1.0, 1.0);

      alpha.add(tuple1, tuple2);
      assertTrue(alpha.epsilonEquals(expected, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = ReferenceFrameMismatchException.class)
   public final void testAddFrameTuple() //Brett
   {
      FrameTuple3D<?, ?> frameTuple1 = createFrameTuple(theFrame);
      FrameTuple3D<?, ?> vector = createFrameTuple(theFrame, 10.0, 11.0, 12.0);
      frameTuple1.add(vector);

      assertEquals(10.0, frameTuple1.getX(), epsilon);
      assertEquals(11.0, frameTuple1.getY(), epsilon);
      assertEquals(12.0, frameTuple1.getZ(), epsilon);

      //Test non-matching reference frames
      FrameTuple3D<?, ?> frameTuple2 = createFrameTuple(aFrame, 10.0, 11.0, 12.0);
      frameTuple1.add(frameTuple2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testAddFrameTupleFrameTuple() //Brett
   {
      FrameTuple3D<?, ?> expected = createFrameTuple(theFrame);
      FrameTuple3D<?, ?> frameTuple1 = createFrameTuple(theFrame, 0.1, 0.1, 0.1);
      FrameTuple3D<?, ?> frameTuple2 = createFrameTuple(theFrame, 10, 10, 10);

      expected.add(frameTuple1, frameTuple2);
      assertEquals(10.1, expected.getX(), epsilon);
      assertEquals(10.1, expected.getY(), epsilon);
      assertEquals(10.1, expected.getZ(), epsilon);

      FrameTuple3D<?, ?> expected2 = createFrameTuple(theFrame);
      FrameTuple3D<?, ?> frameTuple2frame = createFrameTuple(aFrame, 10, 10, 10);
      try
      {
         expected2.add(frameTuple1, frameTuple2frame);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch (ReferenceFrameMismatchException rfme)
      {
         //Good
      }
      try
      {
         expected2.add(frameTuple2frame, frameTuple1);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch (ReferenceFrameMismatchException rfme)
      {
         //Good
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public final void testReferenceFramesAreCheckedOnSet()
   {
      FrameTuple3D<?, ?> framePointOne = createFrameTuple(theFrame);
      FrameTuple3D<?, ?> framePointTwo = createFrameTuple(aFrame);

      framePointOne.set(framePointTwo);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testEpsilonEqualsTuple()
   {
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame, 1.0, 2.0, 3.0);
      Point3D tuple1 = new Point3D(1.0, 2.0, 3.0);

      boolean tupleResult = tuple1.epsilonEquals(tuple1, epsilon);
      boolean framePointResult = framePoint.epsilonEquals(tuple1, epsilon);

      assertTrue(tupleResult == framePointResult);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = ReferenceFrameMismatchException.class)
   public final void testEpsilonEqualsFrameTuple()
   {
      FrameTuple3D<?, ?> framePoint1 = createFrameTuple(theFrame, 1.0, 2.0, 3.0);
      FrameTuple3D<?, ?> framePoint2 = createFrameTuple(aFrame, 1.0, 2.0, 3.0);

      framePoint1.epsilonEquals(framePoint2, epsilon); //test with non-matching refernce frames

      double threshold = 0.5;
      boolean expectedReturn = true;

      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame, 1.1, 2.1, 3.1);
      boolean actualReturn = framePoint1.epsilonEquals(framePoint, threshold);
      assertEquals("return value", expectedReturn, actualReturn);

      framePoint = createFrameTuple(theFrame, 1.2, 2.2, 3.2);
      actualReturn = framePoint1.epsilonEquals(framePoint, threshold);
      assertEquals("return value", expectedReturn, actualReturn);

      expectedReturn = false;
      framePoint = createFrameTuple(theFrame, 1.7, 2.1, 3.1);
      actualReturn = framePoint1.epsilonEquals(framePoint, threshold);
      assertEquals("return value", expectedReturn, actualReturn);

      framePoint = createFrameTuple(theFrame, 1.1, 2.7, 3.1);
      actualReturn = framePoint1.epsilonEquals(framePoint, threshold);
      assertEquals("return value", expectedReturn, actualReturn);

      framePoint = createFrameTuple(theFrame, 0, 0, 0);
      actualReturn = framePoint1.epsilonEquals(framePoint, threshold);
      assertEquals("return value", expectedReturn, actualReturn);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testGetPointCopy()
   {
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame, 1.0, 2.0, 3.0);

      Point3D actualReturn = framePoint.getPointCopy();
      assertEquals(1.0, actualReturn.getX(), epsilon);
      assertEquals(2.0, actualReturn.getY(), epsilon);
      assertEquals(3.0, actualReturn.getZ(), epsilon);

      framePoint.add(createFrameTuple(theFrame, 1.0, 1.0, 1.0));
      assertEquals(1.0, actualReturn.getX(), epsilon);
      assertEquals(2.0, actualReturn.getY(), epsilon);
      assertEquals(3.0, actualReturn.getZ(), epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testGetVectorCopy()
   {
      FrameTuple3D<?, ?> frameTuple = createFrameTuple(theFrame, 1.0, 2.0, 3.0);
      Vector3D tuple3d = new Vector3D(1.0, 2.0, 3.0);
      Vector3D vector3dCopy = frameTuple.getVectorCopy();

      assertTrue(tuple3d.epsilonEquals(vector3dCopy, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testGetTuple3d()
   {
      FrameTuple3D<?, ?> frameTuple = createFrameTuple(theFrame, 1.0, 2.0, 3.0);
      Vector3D tuple3d = new Vector3D();
      frameTuple.get(tuple3d);

      assertTrue(frameTuple.epsilonEquals(tuple3d, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testSetToZero()
   {
      FrameTuple3D<?, ?> frameTuple = createFrameTuple(theFrame, 1.0, 2.0, 3.0);
      FrameTuple3D<?, ?> frameTupleZero = createFrameTuple(theFrame, 0.0, 0.0, 0.0);
      frameTuple.setToZero();
      assertTrue(frameTuple.epsilonEquals(frameTupleZero, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testSetToZeroReferenceFrame()
   {
      FrameTuple3D<?, ?> frameTuple = createFrameTuple(theFrame, 1.0, 2.0, 3.0);
      FrameTuple3D<?, ?> frameTupleZero = createFrameTuple(aFrame, 0.0, 0.0, 0.0);
      frameTuple.setToZero(aFrame);
      assertTrue(frameTuple.epsilonEquals(frameTupleZero, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testSetToNaN()
   {
      FrameTuple3D<?, ?> frameTuple = createFrameTuple(theFrame, 1.0, 2.0, 3.0);
      frameTuple.setToNaN();
      assertTrue(Double.isNaN(frameTuple.getX()));
      assertTrue(Double.isNaN(frameTuple.getY()));
      assertTrue(Double.isNaN(frameTuple.getZ()));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testSetToNaNReferenceFrame()
   {
      FrameTuple3D<?, ?> frameTuple = createFrameTuple(theFrame, 1.0, 2.0, 3.0);
      frameTuple.setToNaN(aFrame);
      assertTrue(frameTuple.getReferenceFrame() == aFrame);
      assertTrue(Double.isNaN(frameTuple.getX()));
      assertTrue(Double.isNaN(frameTuple.getY()));
      assertTrue(Double.isNaN(frameTuple.getZ()));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testGetReferenceFrame()
   {
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame);

      ReferenceFrame expectedReturn = theFrame;
      ReferenceFrame actualReturn = framePoint.getReferenceFrame();
      assertEquals("return value", expectedReturn, actualReturn);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testGetX()
   {
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame, 1.1, 1.2, 1.3);

      double expectedReturn = 1.1;
      double actualReturn = framePoint.getX();
      assertEquals("return value", expectedReturn, actualReturn, Double.MIN_VALUE);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testGetY()
   {
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame, 1.1, 1.2, 1.3);

      double expectedReturn = 1.2;
      double actualReturn = framePoint.getY();
      assertEquals("return value", expectedReturn, actualReturn, Double.MIN_VALUE);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testGetZ()
   {
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame, 1.1, 1.2, 1.3);

      double expectedReturn = 1.3;
      double actualReturn = framePoint.getZ();
      assertEquals("return value", expectedReturn, actualReturn, Double.MIN_VALUE);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = NullPointerException.class)
   public final void testSet()
   {
      FrameTuple3D<?, ?> framePoint1 = createFrameTuple(theFrame);

      FrameTuple3D<?, ?> framePoint = null;
      framePoint1.set(framePoint);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = NullPointerException.class)
   public final void testSet1()
   {
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame);

      FrameTuple3D<?, ?> frameVector = null;
      framePoint.set(frameVector);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testSet2()
   {
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame);

      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      framePoint.set(x, y, z);

      assertEquals(x, framePoint.getX(), epsilon);
      assertEquals(y, framePoint.getY(), epsilon);
      assertEquals(z, framePoint.getZ(), epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testSetX()
   {
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame);

      double x = 0.0;
      framePoint.setX(x);

      assertEquals(x, framePoint.getX(), epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testSetY()
   {
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame);

      double y = 0.0;
      framePoint.setY(y);

      assertEquals(y, framePoint.getY(), epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testSetZ()
   {
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame);

      double z = 0.0;
      framePoint.setZ(z);

      assertEquals(z, framePoint.getZ(), epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testSubTuple3d()
   {
      FrameTuple3D<?, ?> frameTuple1 = createFrameTuple(theFrame, 1.0, 1.0, 1.0);
      Point3D tuple1 = new Point3D(1.0, 1.0, 1.0);

      frameTuple1.sub(tuple1);
      tuple1.sub(tuple1);

      assertTrue(frameTuple1.epsilonEquals(tuple1, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testSubTuple3dTuple3d()
   {
      FrameTuple3D<?, ?> frameTuple1 = createFrameTuple(theFrame, 1.0, 1.0, 1.0);
      FrameTuple3D<?, ?> frameTuple2 = createFrameTuple(theFrame, 1.0, 1.0, 1.0);
      FrameTuple3D<?, ?> expectedFrameTuple = createFrameTuple(theFrame, 0.0, 0.0, 0.0);
      Point3D tuple1 = new Point3D(1.0, 1.0, 1.0);
      Point3D tuple2 = new Point3D(1.0, 1.0, 1.0);
      Point3D expectedTuple = new Point3D(0.0, 0.0, 0.0);

      expectedFrameTuple.sub(frameTuple1, frameTuple2);
      expectedTuple.sub(tuple1, tuple2);

      assertTrue(expectedFrameTuple.epsilonEquals(expectedTuple, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testSubFrameTuple()
   {
      FrameTuple3D<?, ?> framePoint1 = createFrameTuple(theFrame);
      FrameTuple3D<?, ?> framePoint = null;
      FrameTuple3D<?, ?> framePoint2 = createFrameTuple(aFrame);

      try
      {
         framePoint1.sub(framePoint);
         fail("Should have thrown NullPointerException");
      }
      catch (NullPointerException npe)
      {
         //Good
      }
      try
      {
         framePoint1.sub(framePoint2);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch (ReferenceFrameMismatchException rfme)
      {
         //Good
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testSubFrameTupleFrameTuple()
   {
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame);
      FrameTuple3D<?, ?> framePoint2 = createFrameTuple(aFrame);
      FrameTuple3D<?, ?> point1 = null;
      FrameTuple3D<?, ?> point2 = null;
      try
      {
         framePoint.sub(point1, point2);
         fail("Should have thrown NullPointerException");
      }
      catch (NullPointerException npe)
      {
         //Good
      }
      try
      {
         framePoint.sub(point2, point1);
         fail("Should have thrown NullPointerException");
      }
      catch (NullPointerException npe)
      {
         //Good
      }
      try
      {
         framePoint.sub(framePoint, framePoint2);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch (ReferenceFrameMismatchException rfme)
      {
         //Good
      }
      try
      {
         framePoint.sub(framePoint2, framePoint);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch (ReferenceFrameMismatchException rfme)
      {
         //Good
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = ReferenceFrameMismatchException.class)
   public final void testCheckReferenceFrameMatch() throws ReferenceFrameMismatchException
   {
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame);

      ReferenceFrame frame = null;
      framePoint.checkReferenceFrameMatch(frame);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = NullPointerException.class)
   public final void testCheckReferenceFrameMatch1() throws ReferenceFrameMismatchException
   {
      FrameTuple3D<?, ?> framePoint1 = createFrameTuple(theFrame);

      FrameTuple3D<?, ?> framePoint = null;
      framePoint1.checkReferenceFrameMatch(framePoint);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = NullPointerException.class)
   public final void testCheckReferenceFrameMatch2() throws ReferenceFrameMismatchException
   {
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame);

      FrameTuple3D<?, ?> frameVector = null;
      framePoint.checkReferenceFrameMatch(frameVector);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = NullPointerException.class)
   public final void testWeightedAverageNullPointerException()
   {
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame);

      FrameTuple3D<?, ?> point1 = null;
      FrameTuple3D<?, ?> point2 = null;
      double weightedAverage = 0.0;
      framePoint.interpolate(point1, point2, weightedAverage);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testFramePoint()
   {
      createFrameTuple(theFrame);

      ReferenceFrame referenceFrame = null;
      createFrameTuple(referenceFrame);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public final void testFramePoint1()
   {
      createFrameTuple(theFrame);

      ReferenceFrame referenceFrame = null;
      Point3D point = null;
      createFrameTuple(referenceFrame, point);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testFramePoint2()
   {
      createFrameTuple(theFrame);

      ReferenceFrame referenceFrame = null;
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      createFrameTuple(referenceFrame, x, y, z);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public final void testFramePoint3()
   {
      createFrameTuple(theFrame);

      FrameTuple3D<?, ?> framePoint = null;
      createFrameTuple(framePoint);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public final void testFramePoint4()
   {
      createFrameTuple(theFrame);

      FrameTuple3D<?, ?> frameVector = null;
      createFrameTuple(frameVector);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testGetDirection()
   {
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame, 1.0, 2.0, 5.0);

      assertEquals(1.0, framePoint.get(Direction.X), epsilon);
      assertEquals(2.0, framePoint.get(Direction.Y), epsilon);
      assertEquals(5.0, framePoint.get(Direction.Z), epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = ReferenceFrameMismatchException.class) //Brett was here
   public final void testScale()
   {
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame, 5.0, 5.0, 5.0);
      FrameTuple3D<?, ?> tuple1 = createFrameTuple(aFrame, 5.0, 5.0, 5.0);

      double scaleFactor = 4.0;
      framePoint.scale(scaleFactor);

      double expectedReturn = 20.0;
      double actualReturn = framePoint.getZ();
      assertEquals("return value", expectedReturn, actualReturn, Double.MIN_VALUE);

      framePoint.set(1.0, 2.0, 3.0);
      framePoint.scale(3.0, 3.0 / 2.0, 1.0);
      assertEquals("return value", 3.0, framePoint.getX(), Double.MIN_VALUE);
      assertEquals("return value", 3.0, framePoint.getY(), Double.MIN_VALUE);
      assertEquals("return value", 3.0, framePoint.getZ(), Double.MIN_VALUE);

      //test non-matching reference frames
      framePoint.setAndScale(scaleFactor, tuple1);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testScaleVector()
   {
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame);
      FrameTuple3D<?, ?> frameVector = createFrameTuple(theFrame, 1.0, 3.0, -2.0);
      double scale = 2.3;

      framePoint.setAndScale(scale, frameVector);

      assertEquals(2.3, framePoint.getX(), epsilon);
      assertEquals(6.9, framePoint.getY(), epsilon);
      assertEquals(-4.6, framePoint.getZ(), epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public final void testScaleVectorException()
   {
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame);
      FrameTuple3D<?, ?> frameVector = createFrameTuple(aFrame, 1.0, 3.0, -2.0);
      double scale = 2.3;

      framePoint.setAndScale(scale, frameVector);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testScalePoint()
   {
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame);
      FrameTuple3D<?, ?> framePoint2 = createFrameTuple(theFrame, 1.0, 3.0, -2.0);
      double scale = 2.3;

      framePoint.setAndScale(scale, framePoint2);

      assertEquals(2.3, framePoint.getX(), epsilon);
      assertEquals(6.9, framePoint.getY(), epsilon);
      assertEquals(-4.6, framePoint.getZ(), epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public final void testScalePointException()
   {
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame);
      FrameTuple3D<?, ?> framePoint2 = createFrameTuple(aFrame, 1.0, 3.0, -2.0);
      double scale = 2.3;

      framePoint.setAndScale(scale, framePoint2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testScaleAddVectorVector() //Brett was here
   {
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame);
      FrameTuple3D<?, ?> frameVector1 = createFrameTuple(theFrame, 7.0, -1.5, -2.0);
      FrameTuple3D<?, ?> frameVector2 = createFrameTuple(theFrame, 1.0, 3.0, 3.6);
      double scale = 2.3;

      framePoint.scaleAdd(scale, frameVector1, frameVector2);

      assertEquals(scale * 7.0 + 1.0, framePoint.getX(), epsilon);
      assertEquals(scale * -1.5 + 3.0, framePoint.getY(), epsilon);
      assertEquals(scale * -2.0 + 3.6, framePoint.getZ(), epsilon);

      FrameTuple3D<?, ?> frameVector3 = createFrameTuple(aFrame, 1.0, 3.0, 3.6);
      try
      {
         framePoint.scaleAdd(scale, frameVector1, frameVector3);
         fail("Should throw ReferenceFrameMismatchException");
      }
      catch (ReferenceFrameMismatchException rfme)
      {
         //Good
      }
      try
      {
         framePoint.scaleAdd(scale, frameVector3, frameVector1);
         fail("Should throw ReferenceFrameMismatchException");
      }
      catch (ReferenceFrameMismatchException rfme)
      {
         //Good
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public final void testScaleAddVectorVectorException1()
   {
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame);
      FrameTuple3D<?, ?> frameVector1 = createFrameTuple(aFrame, 7.0, -1.5, -2.0);
      FrameTuple3D<?, ?> frameVector2 = createFrameTuple(theFrame, 1.0, 3.0, 3.6);
      double scale = 2.3;

      framePoint.scaleAdd(scale, frameVector1, frameVector2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public final void testScaleAddVectorVectorException2()
   {
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame);
      FrameTuple3D<?, ?> frameVector1 = createFrameTuple(theFrame, 7.0, -1.5, -2.0);
      FrameTuple3D<?, ?> frameVector2 = createFrameTuple(aFrame, 1.0, 3.0, 3.6);
      double scale = 2.3;

      framePoint.scaleAdd(scale, frameVector1, frameVector2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testScaleAddVectorPoint()
   {
      FrameTuple3D<?, ?> framePointResult1 = createFrameTuple(theFrame);
      FrameTuple3D<?, ?> framePointResult2 = createFrameTuple(theFrame);

      Vector3D vector1 = new Vector3D(7.0, -1.5, -2.0);
      Vector3D vector2 = new Vector3D(1.0, 3.0, 3.6);
      double scale = 2.3;

      FrameTuple3D<?, ?> frameVector1 = createFrameTuple(theFrame, vector1);
      FrameTuple3D<?, ?> framePoint1 = createFrameTuple(theFrame, vector2);
      FrameTuple3D<?, ?> frameVector2 = createFrameTuple(theFrame, vector2);
      FrameTuple3D<?, ?> framePoint2 = createFrameTuple(theFrame, vector1);

      framePointResult1.scaleAdd(scale, frameVector1, framePoint1);
      assertEquals(2.3 * 7.0 + 1.0, framePointResult1.getX(), epsilon);
      assertEquals(2.3 * -1.5 + 3.0, framePointResult1.getY(), epsilon);
      assertEquals(2.3 * -2.0 + 3.6, framePointResult1.getZ(), epsilon);

      framePointResult2.scaleAdd(scale, framePoint2, frameVector2);
      assertTrue(framePointResult1.epsilonEquals(framePointResult2, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public final void testScaleAddVectorPointException1()
   {
      FrameTuple3D<?, ?> framePointResult1 = createFrameTuple(theFrame);

      Vector3D vector1 = new Vector3D(7.0, -1.5, -2.0);
      Vector3D vector2 = new Vector3D(1.0, 3.0, 3.6);
      double scale = 2.3;

      FrameTuple3D<?, ?> frameVector1 = createFrameTuple(aFrame, vector1);
      FrameTuple3D<?, ?> framePoint1 = createFrameTuple(theFrame, vector2);

      framePointResult1.scaleAdd(scale, frameVector1, framePoint1);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public final void testScaleAddVectorPointException2()
   {
      FrameTuple3D<?, ?> framePointResult1 = createFrameTuple(theFrame);

      Vector3D vector1 = new Vector3D(7.0, -1.5, -2.0);
      Vector3D vector2 = new Vector3D(1.0, 3.0, 3.6);
      double scale = 2.3;

      FrameTuple3D<?, ?> frameVector1 = createFrameTuple(theFrame, vector1);
      FrameTuple3D<?, ?> framePoint1 = createFrameTuple(aFrame, vector2);

      framePointResult1.scaleAdd(scale, frameVector1, framePoint1);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public final void testScaleAddPointVectorException1()
   {
      FrameTuple3D<?, ?> framePointResult1 = createFrameTuple(theFrame);

      Vector3D vector1 = new Vector3D(7.0, -1.5, -2.0);
      Vector3D vector2 = new Vector3D(1.0, 3.0, 3.6);
      double scale = 2.3;

      FrameTuple3D<?, ?> frameVector1 = createFrameTuple(aFrame, vector1);
      FrameTuple3D<?, ?> framePoint1 = createFrameTuple(theFrame, vector2);

      framePointResult1.scaleAdd(scale, framePoint1, frameVector1);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public final void testScaleAddPointVectorException2()
   {
      FrameTuple3D<?, ?> framePointResult1 = createFrameTuple(theFrame);

      Vector3D vector1 = new Vector3D(7.0, -1.5, -2.0);
      Vector3D vector2 = new Vector3D(1.0, 3.0, 3.6);
      double scale = 2.3;

      FrameTuple3D<?, ?> frameVector1 = createFrameTuple(theFrame, vector1);
      FrameTuple3D<?, ?> framePoint1 = createFrameTuple(aFrame, vector2);

      framePointResult1.scaleAdd(scale, framePoint1, frameVector1);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testScaleAddPointPoint()
   {
      FrameTuple3D<?, ?> framePointResult = createFrameTuple(theFrame);
      FrameTuple3D<?, ?> framePoint1 = createFrameTuple(theFrame, 7.0, -1.5, -2.0);
      FrameTuple3D<?, ?> framePoint2 = createFrameTuple(theFrame, 1.0, 3.0, 3.6);
      double scale = 2.3;

      framePointResult.scaleAdd(scale, framePoint1, framePoint2);

      assertEquals(2.3 * 7.0 + 1.0, framePointResult.getX(), epsilon);
      assertEquals(2.3 * -1.5 + 3.0, framePointResult.getY(), epsilon);
      assertEquals(2.3 * -2.0 + 3.6, framePointResult.getZ(), epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public final void testScaleAddPointPointException1()
   {
      FrameTuple3D<?, ?> framePointResult = createFrameTuple(theFrame);
      FrameTuple3D<?, ?> framePoint1 = createFrameTuple(aFrame, 7.0, -1.5, -2.0);
      FrameTuple3D<?, ?> framePoint2 = createFrameTuple(theFrame, 1.0, 3.0, 3.6);
      double scale = 2.3;

      framePointResult.scaleAdd(scale, framePoint1, framePoint2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public final void testScaleAddPointPointException2()
   {
      FrameTuple3D<?, ?> framePointResult = createFrameTuple(theFrame);
      FrameTuple3D<?, ?> framePoint1 = createFrameTuple(theFrame, 7.0, -1.5, -2.0);
      FrameTuple3D<?, ?> framePoint2 = createFrameTuple(aFrame, 1.0, 3.0, 3.6);
      double scale = 2.3;

      framePointResult.scaleAdd(scale, framePoint1, framePoint2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = ReferenceFrameMismatchException.class) //Brett was here
   public final void testScaleAddScaleTuple()
   {
      FrameTuple3D<?, ?> framePointResult = createFrameTuple(theFrame, 7.0, -1.5, -2.0);
      FrameTuple3D<?, ?> frameVector = createFrameTuple(theFrame, 1.0, 3.0, 3.6);
      double scale = 2.3;

      framePointResult.scaleAdd(scale, frameVector);

      assertEquals(2.3 * 7.0 + 1.0, framePointResult.getX(), epsilon);
      assertEquals(2.3 * -1.5 + 3.0, framePointResult.getY(), epsilon);
      assertEquals(2.3 * -2.0 + 3.6, framePointResult.getZ(), epsilon);

      //test non-matching reference frames
      FrameTuple3D<?, ?> ft1 = createFrameTuple(aFrame);
      ft1.scaleAdd(scale, frameVector);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public final void testScaleAddVectorException()
   {
      FrameTuple3D<?, ?> framePointResult = createFrameTuple(theFrame, 7.0, -1.5, -2.0);
      FrameTuple3D<?, ?> frameVector = createFrameTuple(aFrame, 1.0, 3.0, 3.6);
      double scale = 2.3;

      framePointResult.scaleAdd(scale, frameVector);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testScaleAddPoint()
   {
      FrameTuple3D<?, ?> framePointResult = createFrameTuple(theFrame, 7.0, -1.5, -2.0);
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame, 1.0, 3.0, 3.6);
      double scale = 2.3;

      framePointResult.scaleAdd(scale, framePoint);

      assertEquals(2.3 * 7.0 + 1.0, framePointResult.getX(), epsilon);
      assertEquals(2.3 * -1.5 + 3.0, framePointResult.getY(), epsilon);
      assertEquals(2.3 * -2.0 + 3.6, framePointResult.getZ(), epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public final void testScaleAddPointException()
   {
      FrameTuple3D<?, ?> framePointResult = createFrameTuple(theFrame, 7.0, -1.5, -2.0);
      FrameTuple3D<?, ?> framePoint = createFrameTuple(aFrame, 1.0, 3.0, 3.6);
      double scale = 2.3;

      framePointResult.scaleAdd(scale, framePoint);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testSubFramePoint()
   {
      FrameTuple3D<?, ?> framePoint1 = createFrameTuple(theFrame, 7.0, -1.5, -2.0);
      FrameTuple3D<?, ?> framePoint2 = createFrameTuple(theFrame, 1.0, 3.0, 3.6);
      FrameTuple3D<?, ?> expectedResult = createFrameTuple(theFrame, 6.0, -4.5, -5.6);

      framePoint1.sub(framePoint2);
      assertTrue(expectedResult.epsilonEquals(framePoint1, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testSubFrameVector()
   {
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame, 7.0, -1.5, -2.0);
      FrameTuple3D<?, ?> frameVector = createFrameTuple(theFrame, 1.0, 3.0, 3.6);
      FrameTuple3D<?, ?> expectedResult = createFrameTuple(theFrame, 6.0, -4.5, -5.6);

      framePoint.sub(frameVector);
      assertTrue(expectedResult.epsilonEquals(framePoint, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testSubPointPoint()
   {
      FrameTuple3D<?, ?> framePoint1 = createFrameTuple(theFrame, 7.0, -1.5, -2.0);
      FrameTuple3D<?, ?> framePoint2 = createFrameTuple(theFrame, 1.0, 3.0, 3.6);
      FrameTuple3D<?, ?> expectedResult = createFrameTuple(theFrame, 6.0, -4.5, -5.6);

      FrameTuple3D<?, ?> actualResult = createFrameTuple(theFrame);
      actualResult.sub(framePoint1, framePoint2);
      assertTrue(expectedResult.epsilonEquals(actualResult, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testSubPointVector()
   {
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame, 7.0, -1.5, -2.0);
      FrameTuple3D<?, ?> frameVector = createFrameTuple(theFrame, 1.0, 3.0, 3.6);
      FrameTuple3D<?, ?> expectedResult = createFrameTuple(theFrame, 6.0, -4.5, -5.6);

      FrameTuple3D<?, ?> actualResult = createFrameTuple(theFrame);
      actualResult.sub(framePoint, frameVector);
      assertTrue(expectedResult.epsilonEquals(actualResult, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testSubVectorPoint()
   {
      FrameTuple3D<?, ?> frameVector = createFrameTuple(theFrame, 7.0, -1.5, -2.0);
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame, 1.0, 3.0, 3.6);
      FrameTuple3D<?, ?> expectedResult = createFrameTuple(theFrame, 6.0, -4.5, -5.6);

      FrameTuple3D<?, ?> actualResult = createFrameTuple(theFrame);
      actualResult.sub(frameVector, framePoint);
      assertTrue(expectedResult.epsilonEquals(actualResult, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testSetNaN()
   {
      FrameTuple3D<?, ?> frameVector = createFrameTuple(theFrame, 7.0, -1.5, -2.0);
      FrameTuple3D<?, ?> frameVector2 = createFrameTuple(theFrame, 20.0, -7.0, -14.0);

      frameVector.setToNaN();
      assertTrue(Double.isNaN(frameVector.getX()));
      assertTrue(Double.isNaN(frameVector.getY()));
      assertTrue(Double.isNaN(frameVector.getZ()));

      frameVector2.setToNaN(theFrame);
      assertTrue(Double.isNaN(frameVector2.getX()));
      assertTrue(Double.isNaN(frameVector2.getY()));
      assertTrue(Double.isNaN(frameVector2.getZ()));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testCheckForNaNAndContainsNaN()
   {
      FrameTuple3D<?, ?> framePoint = createFrameTuple(theFrame, 1.0, 2.0, 3.0);
      framePoint.checkForNaN();
      assertFalse(framePoint.containsNaN());

      framePoint.setX(Double.NaN);
      assertTrue(framePoint.containsNaN());

      try
      {
         framePoint.checkForNaN();
         fail("Failed to throw exception");
      }
      catch (RuntimeException e)
      {
      }

      framePoint.setX(1.0);
      framePoint.checkForNaN();
      assertFalse(framePoint.containsNaN());
      framePoint.setY(Double.NaN);
      assertTrue(framePoint.containsNaN());

      try
      {
         framePoint.checkForNaN();
         fail("Failed to throw exception");
      }
      catch (RuntimeException e)
      {
      }

      framePoint.setY(2.0);
      framePoint.checkForNaN();
      assertFalse(framePoint.containsNaN());
      framePoint.setZ(Double.NaN);
      assertTrue(framePoint.containsNaN());

      try
      {
         framePoint.checkForNaN();
         fail("Failed to throw exception");
      }
      catch (RuntimeException e)
      {
      }

      framePoint.setZ(3.0);
      framePoint.checkForNaN();
      assertFalse(framePoint.containsNaN());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testInterpolate()
   {
      FrameTuple3D<?, ?> frameTuple1 = createFrameTuple(ReferenceFrame.getWorldFrame(), -1.0, 0.0, 17.0);
      FrameTuple3D<?, ?> frameTuple2 = createFrameTuple(ReferenceFrame.getWorldFrame(), 3.3, 30.0, 9.0);
      FrameTuple3D<?, ?> frameTuple3 = createFrameTuple(ReferenceFrame.getWorldFrame(), 1.0, 2.8, 3.0);

      frameTuple3.interpolate(frameTuple1, frameTuple2, 3.0);
      frameTuple3.interpolate(frameTuple2, frameTuple1, 1);

      FrameTuple3D<?, ?> frameTuple4 = createFrameTuple(theFrame, 1.0, 2.0, 3.0);
      FrameTuple3D<?, ?> frameTuple5 = createFrameTuple(theFrame, 0.0, -1.0, 8.2);
      double alpha = 0.57;

      FrameTuple3D<?, ?> resultTuple = createFrameTuple(frameTuple4.getReferenceFrame(), frameTuple4.getX(), frameTuple4.getY(), frameTuple4.getZ());
      resultTuple.interpolate(frameTuple4, frameTuple5, alpha);

      assertEquals((1.0 - alpha) * frameTuple4.getX() + alpha * frameTuple5.getX(), resultTuple.getX(), epsilon);
      assertEquals((1.0 - alpha) * frameTuple4.getY() + alpha * frameTuple5.getY(), resultTuple.getY(), epsilon);
      assertEquals((1.0 - alpha) * frameTuple4.getZ() + alpha * frameTuple5.getZ(), resultTuple.getZ(), epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testPackMatrix() //Brett was here
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      FrameTuple3D<?, ?> frametuple = createFrameTuple(worldFrame, 10.0, 10.0, 10.0);
      Random ran = new Random(4564L);
      int numberOfIterations = 100;
      int startRow;
      for (int counter = 0; counter < numberOfIterations; counter++)
      {
         int numRows = ran.nextInt(10) + 3; //3-13 exclusive
         int numCols = ran.nextInt(10) + 3;
         if (ran.nextInt(Math.max(numRows - 3, 1)) <= 0)
         {
            startRow = 0;
         }
         else
         {
            startRow = ran.nextInt(numRows - 3);
         }
         //         int startRow = ran.nextInt(numRows - 3); //3 fewer than number of last row because of 3-tuple x, y, z

         DenseMatrix64F matrix = new DenseMatrix64F(numRows, numCols);
         //      System.out.println(matrix.toString()); //before
         frametuple.get(startRow, matrix);
         //      System.out.println(matrix.toString()); //after

         for (int i = startRow; i < startRow + 3; i++)
         {
            //            System.out.println("Entry " + i + " " + matrix.get(i, 0));
            assertEquals("Should be equal", 10.0, matrix.get(i, 0), Double.MIN_VALUE);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testClipToMinMax() //Brett was here
   {
      FrameTuple3D<?, ?> frameTuple = createFrameTuple(ReferenceFrame.getWorldFrame(), -5.0, 3.0, 10.0);
      frameTuple.clipToMinMax(4, 10); //call clipToMinMax(4, 10)
      assertEquals("Should be equal", 4, frameTuple.getX(), epsilon);
      assertEquals("Should be equal", 4, frameTuple.getY(), epsilon);
      assertEquals("Should be equal", 10, frameTuple.getZ(), epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testNegate() //Brett was here
   {
      FrameTuple3D<?, ?> frameTuple = createFrameTuple(ReferenceFrame.getWorldFrame(), -5.0, 0.0, 10.0);
      FrameTuple3D<?, ?> frameTupleToNegate = createFrameTuple(ReferenceFrame.getWorldFrame(), 5.0, 0.0, -10.0);
      frameTupleToNegate.negate();
      assertEquals("Should be equal", frameTuple.getX(), frameTupleToNegate.getX(), epsilon);
      assertEquals("Should be equal", frameTuple.getY(), frameTupleToNegate.getY(), epsilon);
      assertEquals("Should be equal", frameTuple.getZ(), frameTupleToNegate.getZ(), epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public final void testAbsolute()
   {
      FrameTuple3D<?, ?> expectedFrameTuple = createFrameTuple(ReferenceFrame.getWorldFrame(), 5.0, 0.0, 10.0);
      FrameTuple3D<?, ?> actualFrameTuple = createFrameTuple(ReferenceFrame.getWorldFrame(), 5.0, 0.0, -10.0);
      actualFrameTuple.absolute();
      assertEquals("Should be equal", expectedFrameTuple.getX(), actualFrameTuple.getX(), epsilon);
      assertEquals("Should be equal", expectedFrameTuple.getY(), actualFrameTuple.getY(), epsilon);
      assertEquals("Should be equal", expectedFrameTuple.getZ(), actualFrameTuple.getZ(), epsilon);

      expectedFrameTuple = createFrameTuple(ReferenceFrame.getWorldFrame(), 5.0, 1.0, 10.0);
      actualFrameTuple = createFrameTuple(ReferenceFrame.getWorldFrame(), 5.0, -1.0, -10.0);
      actualFrameTuple.absolute();
      assertEquals("Should be equal", expectedFrameTuple.getX(), actualFrameTuple.getX(), epsilon);
      assertEquals("Should be equal", expectedFrameTuple.getY(), actualFrameTuple.getY(), epsilon);
      assertEquals("Should be equal", expectedFrameTuple.getZ(), actualFrameTuple.getZ(), epsilon);
   }
}
