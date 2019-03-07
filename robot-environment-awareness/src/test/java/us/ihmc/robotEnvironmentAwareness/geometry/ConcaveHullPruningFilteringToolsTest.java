package us.ihmc.robotEnvironmentAwareness.geometry;

import static org.junit.Assert.assertEquals;

import org.junit.Assert;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.MethodOrderer.OrderAnnotation;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

import us.ihmc.commons.MutationTestFacilitator;


public class ConcaveHullPruningFilteringToolsTest extends ConcaveHullTestBasics
{
	private static final boolean DEBUG = true;
	private static final double EPS = 1.0e-6;

	public ConcaveHullPruningFilteringToolsTest()
	{
		VISUALIZE = false;
	}

	@Test
	public void filterOutPeaksAndShallowAnglesHullCollectionTest()
	{
		initializeBaseClass();
		
		ConcaveHullCollection sombreroCollection = null;
		if ((sombreroCollection = this.sombreroCollection) != null)
		{
			double shallowAngleThreshold = 0, peakAngleThreshold = .1;
			int result = ConcaveHullPruningFilteringTools.filterOutPeaksAndShallowAngles(shallowAngleThreshold, peakAngleThreshold, sombreroCollection);			
			assert (result == 13); // should be 13 if createSombrero(-5, 5, 51) and peakAngleThreshold = .1
			if(DEBUG) System.out.println("ConcaveHullPruningFilteringTools.filterOutPeaksAndShallowAnglesHullCollectionTest = " + result);			
		}
		else {
			Assert.assertFalse("ConcaveHullPruningFilteringTools BaseClass not intialized!", sombreroInitialized);
			}
		}

	@Test
	public void filterOutPeaksAndShallowAnglesHullTest()
	{
		initializeBaseClass();
		if (sombreroInitialized)
		{
			double shallowAngleThreshold = 0, peakAngleThreshold = .1;
			int result = ConcaveHullPruningFilteringTools.filterOutPeaksAndShallowAngles(shallowAngleThreshold, peakAngleThreshold, sombreroHull);
			assert (result == 13); // should be 13 if createSombrero(-5, 5, 51) and peakAngleThreshold = .1
			if(DEBUG) System.out.println("ConcaveHullPruningFilteringTools.filterOutPeaksAndShallowAnglesHullTest(), result  = " + result);
		}
		else {
			Assert.assertFalse("ConcaveHullPruningFilteringTools BaseClass not intialized!", sombreroInitialized);
		}
}

	@Test
	public void filterOutPeaksAndShallowAnglesHullVerticesTest()
	{
		initializeBaseClass();
		if (sombreroInitialized)
		{
			double shallowAngleThreshold = 0, peakAngleThreshold = .1;
			int result = ConcaveHullPruningFilteringTools.filterOutPeaksAndShallowAngles(shallowAngleThreshold, peakAngleThreshold, sombrero);
			assert (result == 13); // should be 13 if createSombrero(-5, 5, 51) and peakAngleThreshold = .1
			if(DEBUG) System.out.println("ConcaveHullPruningFilteringTools.filterOutPeaksAndShallowAnglesHullVerticesTest(), result = " + result);
		}
		else {
			Assert.assertFalse("ConcaveHullPruningFilteringTools BaseClass not intialized!", sombreroInitialized);
		}
}

	@Test
	public void filterOutShallowHullVerticesTest()
	{
		initializeBaseClass();
		if (sombreroInitialized)
		{
			double percentageThreshold = 0.0067;
			int result = ConcaveHullPruningFilteringTools.filterOutShallowVertices(percentageThreshold, sombrero);
			assert (result == 40);  // should be 40 if createSombrero(-5, 5, 51) and percentageThreshold = 0.0067
			if(DEBUG) System.out.println("ConcaveHullPruningFilteringTools.filterOutShallowHullVerticesTest(), result = " + result);
		}
		else {
			Assert.assertFalse("ConcaveHullPruningFilteringTools BaseClass not intialized!", sombreroInitialized);
		}
}

	@Test
	public void filterOutGroupsOfShallowHullVerticesTest()
	{
		initializeBaseClass();
		if (sombreroInitialized)
		{
		double percentageThreshold = .0335;
		int result = ConcaveHullPruningFilteringTools.filterOutGroupsOfShallowVertices(percentageThreshold, sombrero);
		assert (result == 0);  // should be 0 if createSombrero(-5, 5, 51) and percentageThreshold = 0.0335
		if(DEBUG) System.out.println("ConcaveHullPruningFilteringTools.filterOutGroupsOfShallowHullVerticesTest(), result = " + result);
		}
		else {
			Assert.assertFalse("ConcaveHullPruningFilteringTools BaseClass not intialized!", sombreroInitialized);
		}
	}

	@Test
	public void filterOutSmallTrianglesHullVerticesTest()
	{
		initializeBaseClass();
		if (sombreroInitialized)
		{
			double areaThreshold = 1e-4;  //*Double.MIN_NORMAL;
			int result = ConcaveHullPruningFilteringTools.filterOutSmallTriangles(areaThreshold, sombrero);
			assert (result == 2);  // should be 2 if createSombrero(-5, 5, 51) and areaThreshold = 1e-30
			if(DEBUG) System.out.println("ConcaveHullPruningFilteringTools.filterOutSmallTrianglesHullVerticesTest(), result = " + result);
		}
		else {
			Assert.assertFalse("ConcaveHullPruningFilteringTools BaseClass not intialized!", sombreroInitialized);
		}
	}

	@Test
	public void flattenShallowPocketsHullVerticesTest()
	{
		initializeBaseClass();
		if (sombreroInitialized)
		{
			double depthThreshold = .954;  //*(max-min1);  //100*Double.MIN_NORMAL;
			int result = ConcaveHullPruningFilteringTools.flattenShallowPockets(depthThreshold, sombrero);
			assert (result == 24);  // should be 24 if createSombrero(-5, 5, 51) and depthThreshold = 0.925;
			if(DEBUG) System.out.println("ConcaveHullPruningFilteringTools.flattenShallowPocketsHullVerticesTest()(), result = " + result);
		}
		else {
			Assert.assertFalse("ConcaveHullPruningFilteringTools BaseClass not intialized!", sombreroInitialized);
		}
}

	@Test
	public void filterOutShortEdgesHullCollectionTest()
	{
		initializeBaseClass();
		if (sombreroInitialized)
		{
			double lengthThreshold = 2;
			int result = ConcaveHullPruningFilteringTools.filterOutShortEdges(lengthThreshold, sombreroCollection);
			assert (result == 42); // should be 42 if createSombrero(-5, 5, 51) and lengthThreshold = 2
			if(DEBUG) System.out.println("ConcaveHullPruningFilteringTools.filterOutShortEdgesHullCollectionTest(), result = " + result);
		}
		else {
			Assert.assertFalse("ConcaveHullPruningFilteringTools BaseClass not intialized!", sombreroInitialized);
		}
}

	@Test
	public void filterOutShortEdgesHullTest()
	{
		initializeBaseClass();
		if (sombreroInitialized)
		{
			double lengthThreshold = 2;
			int result = ConcaveHullPruningFilteringTools.filterOutShortEdges(lengthThreshold, sombreroHull);
			assert (result == 42); // should be 44 if createSombrero(-5, 5, 51) and lengthThreshold = 2
			if(DEBUG) System.out.println("ConcaveHullPruningFilteringTools.filterOutShortEdgesHullTest(), result = " + result);
		}
		else {
			Assert.assertFalse("ConcaveHullPruningFilteringTools BaseClass not intialized!", sombreroInitialized);
		}
}

	@Test
	public void filterOutShortEdgesHullVerticesTest()
	{
		initializeBaseClass();
		if (sombreroInitialized)
		{
			double lengthThreshold = 2;

			int result = ConcaveHullPruningFilteringTools.filterOutShortEdges(lengthThreshold, sombrero);
			assert (result == 42); // should be 42 if createSombrero(-5, 5, 51) and lengthThreshold = 2
			if(DEBUG) System.out.println("ConcaveHullPruningFilteringTools.filterOutShortEdgesHullVerticesTest(), result = " + result);
		}
		else {
			Assert.assertFalse("ConcaveHullPruningFilteringTools BaseClass not intialized!", sombreroInitialized);
		}
}

	@Test
	public void filterByRayHullVerticesTest()
	{
		initializeBaseClass();
		if (sombreroInitialized)
		{
			double threshold = 2;

			int result = ConcaveHullPruningFilteringTools.filterByRay(threshold, sombrero);
			assert (result == 35);  // should be 35 if createSombrero(-5, 5, 51) and threshold = 2
			if(DEBUG) System.out.println("ConcaveHullPruningFilteringTools.filterByRayHullVerticesTest()(), result = " + result);
		}
		else {
			Assert.assertFalse("ConcaveHullPruningFilteringTools BaseClass not intialized!", sombreroInitialized);
		}
}

	@Test
	public void innerAlphaShapeFilteringHullVerticesTest()
	{
		initializeBaseClass();
		if (sombreroInitialized)
		{
			double alpha = 0;
			int deadIndexRegion = 0;
			int result = ConcaveHullPruningFilteringTools.innerAlphaShapeFiltering(alpha, deadIndexRegion, sombrero);
			assert (result == 0);
			if(DEBUG) System.out.println("ConcaveHullPruningFilteringTools.innerAlphaShapeFilteringHullVerticesTest(), result = " + result);
		}
		else {
			Assert.assertFalse("ConcaveHullPruningFilteringTools BaseClass not intialized!", sombreroInitialized);
		}
}


	public static void main(String[] args)
	{
		MutationTestFacilitator.facilitateMutationTestForClass(ConcaveHullPruningFilteringToolsTest.class, ConcaveHullPruningFilteringToolsTest.class);		
	}

}
