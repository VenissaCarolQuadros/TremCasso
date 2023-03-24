import processing.core.*;

public class Buttons extends PApplet{
    int buttonMin;
    int buttonMax;
    float ratio;
    char orientation;  //force orientation
    

    Buttons(int buttonMin, int buttonMax, float ratio, char orientation){
        this.orientation= orientation;
        this.buttonMin= buttonMin;
        this.buttonMax= buttonMax;
        this.ratio= ratio;
    }

    public PVector applyForces(float forceMin, float forceMax, float pos, PVector fEE){
        if (pos>= buttonMin && pos <= buttonMax){
            float increments = (forceMax - forceMin)/ (float)(this.buttonMax -this.buttonMin);
            // System.out.println("buttonMin: " + buttonMin);
            // System.out.println("buttonMax: " + buttonMax);
            // System.out.println("pos: " + pos);
            // System.out.println("==" + (buttonMin + 0.7*(buttonMax-buttonMin)) + "==");
            if((int)pos < (int)(buttonMin + 0.7*(buttonMax-buttonMin))){
                // System.out.println("NOT clicking");
                float force = (pos-buttonMin)* increments;
                if (orientation=='h')
                    fEE.set(fEE.x+force, fEE.y);
                if (orientation =='v')
                    fEE.set(fEE.x, fEE.y+force);
            }else{
                // System.out.println("clicking");
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
