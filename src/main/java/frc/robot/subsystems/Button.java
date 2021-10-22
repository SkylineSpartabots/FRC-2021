package frc.robot.subsystems;

public abstract class Button {

    public abstract boolean canAct();

    public abstract void successAction();

    public abstract void failAction();

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