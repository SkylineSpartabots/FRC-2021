package frc.robot.subsystems;

/**
 * Add actions for subsystems to add to telemetry
 */
public abstract class Button {
    /**
     * If button can do the action when clicking
     * @return if button can do action
     */
    public abstract boolean canAct();

    /**
     * Action for button to do if it can do action
     */

    public abstract void successAction();

    /**
     * Action for button to do if it does not succeed
     */

    public abstract void failAction();

    /**
     * What runs when button is clicked
     */

    public void onClick(){
        if(canAct()){
            //DISPLAY SUCCESSFUL BUTTON MESSAGE
            successAction();
        }
        else{
            //DSPLAY FAILURE BUTTON MESSAGE
            failAction();
        }
    }
}