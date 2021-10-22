public abstract class Button {
    public abstract boolean act();

    public void onClick(){
        if(act()){
            //DISPLAY SUCCESSFUL BUTTON MESSAGE
        }
        else{
            //DSPLAY FAILURE BUTTON MESSAGE
        }
    }
}