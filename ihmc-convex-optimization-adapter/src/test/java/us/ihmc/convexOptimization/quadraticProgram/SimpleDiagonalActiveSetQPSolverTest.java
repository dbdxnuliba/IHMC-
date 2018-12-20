package us.ihmc.convexOptimization.quadraticProgram;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
public class SimpleDiagonalActiveSetQPSolverTest extends AbstractSimpleActiveSetQPSolverTest
{
   @Override
   public SimpleActiveSetQPSolverInterface createSolverToTest()
   {
      return new SimpleDiagonalActiveSetQPSolver();
   }

   @Test// timeout = 30000
   public void testSimpleCasesWithBoundsConstraints()
   {
      testSimpleCasesWithBoundsConstraints(1, 3, 2, 3, false);
   }
}
