package us.ihmc.robotEnvironmentAwareness.geometry;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.random.RandomGeometry;

public class PointMeanTest extends ConcaveHullTestBasics
{
	private static final int SEED = 0;
	//private static final int ITERATIONS = 100;
	private static final double EPS = 1.0e-12;

	
	@SuppressWarnings({"deprecation", "unused"})
	@Test
	public final void testVectorMean()
	{
		Point3D sum = new Point3D();
		PointMean pMean = new PointMean();
		Random random = new Random();

		random.setSeed(SEED);
		for (int i=0; i < ITERATIONS; i++)
			sum.add(RandomGeometry.nextPoint3D(random, 0, 1));			
		sum.scale(1.0 / ITERATIONS);
		
		random.setSeed(0);
		for (int i=0; i < ITERATIONS; i++)
			pMean.update(RandomGeometry.nextPoint3D(random, 0, 1));		
		
		EuclidCoreTestTools.assertTuple3DEquals(pMean, sum, EPS);
		assertEquals(pMean.getX(), sum.getX(), EPS);
		assertEquals(pMean.getY(), sum.getY(), EPS);
		assertEquals(pMean.getZ(), sum.getZ(), EPS);
		assertEquals(pMean.getNumberOfSamples(), ITERATIONS);

		pMean.clear();
		assertEquals(pMean.getX(), 0, EPS);
		assertEquals(pMean.getY(), 0, EPS);
		assertEquals(pMean.getZ(), 0, EPS);
	}

	@Test
	public final void testWithTuple3DBasics()
	{
		Point3D sum = new Point3D();
		PointMean pMean = new PointMean();
		Random random = new Random();

		random.setSeed(SEED);
		for (int i = 0; i < ITERATIONS; i++)
			sum.add(RandomGeometry.nextPoint3D(random, 0, 1));
		sum.scale(1.0 / ITERATIONS);

		random.setSeed(0);
		for (int i = 0; i < ITERATIONS; i++)
			pMean.update((Tuple3DBasics) RandomGeometry.nextPoint3D(random, 0, 1), 1);
		
		assertEquals(pMean.getX(), sum.getX(), EPS);
		assertEquals(pMean.getY(), sum.getY(), EPS);
		assertEquals(pMean.getZ(), sum.getZ(), EPS);
		assertEquals(pMean.getNumberOfSamples(), ITERATIONS);

		pMean.clear();
		assertEquals(pMean.getX(), 0, EPS);
		assertEquals(pMean.getY(), 0, EPS);
		assertEquals(pMean.getZ(), 0, EPS);
	}

	public static void main(String[] args)
	{
		MutationTestFacilitator.facilitateMutationTestForClass(PointMeanTest.class, PointMeanTest.class);
	}
		
}
