import processing.core.*;

public class Buttons extends PApplet{
    int buttonMin;
    int buttonMax;
    float ratio;
    char orientation;  //force orientation
    int multiplier;

    Buttons(int buttonMin, int buttonMax, float ratio, char orientation){
        this.orientation= orientation;
        this.buttonMin= buttonMin;
        this.buttonMax= buttonMax;
        this.ratio= ratio;
        if (this.orientation=='n')
          this.multiplier=-1;
         else
           this.multiplier=1;
        
    }

    public PVector applyForces(float forceMin, float forceMax, float pos, PVector fEE){
      if (pos>= buttonMin && pos <= buttonMax){
            float increments = (forceMax - forceMin)/ (float)(this.buttonMax -this.buttonMin);
            //System.out.print("increments:"+increments);
            // System.out.println("buttonMin: " + buttonMin);
            // System.out.println("buttonMax: " + buttonMax);
            //System.out.println("pos: " + pos);
            //System.out.println("==" + (buttonMin + this.ratio*(buttonMax-buttonMin)) + "==");
            if(this.multiplier*(int)pos < (int)this.multiplier*(buttonMin + this.ratio*(buttonMax-buttonMin))){
                //System.out.println("NOT clicking");
                float force = (pos-buttonMin)* increments;
                if (orientation=='h'|| orientation=='n')
                    fEE.set(fEE.x+force, fEE.y);
                if (orientation =='v')
                    fEE.set(fEE.x, fEE.y+force);
            }else{
                 //System.out.print("clicking");
                float force = (float)0.4*(pos-buttonMin) * increments;
                if (orientation=='h')
                    fEE.set(fEE.x+force, fEE.y);
                if (orientation =='v')
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
