import processing.core.*;

public class Buttons extends PApplet{
    int buttonMin;
    int buttonMax;
    int clickPoint;
    char orientation;  //force orientation
    String forces= "Vertical only";
    //String forces= "Whatever";
    

    Buttons(int buttonMin, int buttonMax, int clickPoint, char orientation){
        this.orientation= orientation;
        this.buttonMin= buttonMin;
        this.buttonMax= buttonMax;
        this.clickPoint= clickPoint;
    }

    public PVector applyForces(float forceMin, float forceMax, float pos, PVector fEE){
      if (this.forces=="Vertical only"){
      if (pos>= buttonMin && pos <= buttonMax){
            if (pos> clickPoint && pos<= clickPoint+20){
              if (orientation=='h')
                    fEE.set((float)(fEE.x+Integer.signum((int)forceMin)*1.0), fEE.y);
                if (orientation=='v')
                    fEE.set(fEE.x, (float)(fEE.y+Integer.signum((int)forceMin)*1.0));
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
      }
      
      else{
      if (pos>= buttonMin && pos <= buttonMax){
              float increments= (forceMax- forceMin)/ (float)(this.buttonMax -this.buttonMin);
              float force= (pos-buttonMin)* increments;
              if (orientation=='h')
                  fEE.set(fEE.x+force, fEE.y);
              if (orientation=='v')
                  fEE.set(fEE.x, fEE.y+force);
            if (pos> clickPoint && pos<= clickPoint+30){
              if (orientation=='h')
                  fEE.set(fEE.x, (float)(fEE.y+2.5));
              if (orientation=='v')
                  fEE.set((float)(fEE.x+2.5), fEE.y);
            }
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
