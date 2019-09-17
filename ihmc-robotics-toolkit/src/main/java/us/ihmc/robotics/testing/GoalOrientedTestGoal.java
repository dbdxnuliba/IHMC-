package us.ihmc.robotics.testing;

public abstract class GoalOrientedTestGoal
{
   private final String variableOperandDescription;
   private final String operatorDescription;
   private final String constantOperandDescription;

   private boolean hasMetGoal = false;

   public GoalOrientedTestGoal(String variableOperandDescription,
                               String operatorDescription,
                               String constantOperandDescription)
   {

      this.variableOperandDescription = variableOperandDescription;
      this.operatorDescription = operatorDescription;
      this.constantOperandDescription = constantOperandDescription;
   }

   protected void update()
   {
      if (!hasMetGoal && currentlyMeetsGoal())
      {
         hasMetGoal = true;
      }
   }

   public boolean hasMetGoal()
   {
      return hasMetGoal;
   }

   public void reset()
   {
      hasMetGoal = false;
   }

   public abstract boolean currentlyMeetsGoal();

   @Override
   public String toString()
   {
      return "variable operand: " + variableOperandDescription + " operator: " + operatorDescription + " constant operand: " + constantOperandDescription;
   }

   public String getVariableOperandDescription()
   {
      return variableOperandDescription;
   }

   public String getOperatorDescription()
   {
      return operatorDescription;
   }

   public String getConstantOperandDescription()
   {
      return constantOperandDescription;
   }
}
