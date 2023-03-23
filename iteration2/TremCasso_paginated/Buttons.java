import processing.core.*;

public class Buttons extends PApplet{
    int buttonMin;
    int buttonMax;
    int clickPoint;
    char orientation;  //force orientation
    

    Buttons(int buttonMin, int buttonMax, int clickPoint, char orientation){
        this.orientation= orientation;
        this.buttonMin= buttonMin;
        this.buttonMax= buttonMax;
        this.clickPoint= clickPoint;
    }

    public PVector applyForces(float forceMin, float forceMax, float pos, PVector fEE){
        if (pos>= buttonMin && pos <= forceMax){
          if (pos>= clickPoint && pos<= clickPoint+20){
          print("Here");
          if (orientation=='h')
                fEE.set((float)(fEE.x+Integer.signum((int)forceMin)), fEE.y);
            if (orientation=='v')
                fEE.set(fEE.x,  (float) (fEE.y+Integer.signum((int)forceMin)));
          }
        else{
            float increments= (forceMax- forceMin)/ (float)(this.buttonMax -this.buttonMin);
            float force= (pos-buttonMin)* increments;
            if (orientation=='h')
                fEE.set(fEE.x+force, fEE.y);
            if (orientation=='v')
                fEE.set(fEE.x, fEE.y+force);
        }
        }
        
        return fEE;
    }

    public boolean click()
    {
        return true;
    }

    /*
    public boolean click(int timeElapsed, int delayMax, int delayMin, int posX, int posY){
        if ()
        {
            return true
        }
        else 
            return false

    }
    */
}
