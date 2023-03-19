import processing.core.*;

public class Buttons extends PApplet{
    int buttonMin;
    int buttonMax;
    char orientation;  //force orientation
    

    Buttons(int buttonMin, int buttonMax, char orientation){
        this.orientation= orientation;
        this.buttonMin= buttonMin;
        this.buttonMax= buttonMax;
    }

    public PVector applyForces(float forceMin, float forceMax, float pos, PVector fEE){
        if (pos>= buttonMin && pos <= buttonMax){
            float increments= (forceMax- forceMin)/ (float)(this.buttonMax -this.buttonMin);
            float force= (pos-buttonMin)* increments;
            if (orientation=='h')
                fEE.set(fEE.x+force, fEE.y);
            if (orientation=='v')
                fEE.set(fEE.x, fEE.y+force);
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