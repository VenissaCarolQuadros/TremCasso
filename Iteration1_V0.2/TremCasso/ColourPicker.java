import processing.core.*;

public class ColourPicker extends PApplet{
    int[] position;
    int[] pointer;
    PImage palette;
    float[] cposition;
    FWorld world;
    float worldWidth = (float)25.0;  
    float worldHeight = (float)10.0; 
    int colour;
    
    public ColourPicker(int[] position, int[] pointer, PImage palette, FWorld world){
        this.position=position;
        this.pointer=pointer;
        this.palette=palette;
        this.cposition= new float[]{position[0], position[1]};
        this.world= world;
    }
    public int[] up(){
        if (this.pointer[1]-this.position[1]>0){
            this.pointer[1]-=25;
        }
        else
          this.pointer[1]=300;
        return this.pointer;   
    }
    public int[] down(){
        if (this.pointer[1]-this.position[1]<300){
            this.pointer[1]+=25;
        }
        else
          this.pointer[1]=0;
        
        return this.pointer;   
    }
    public int[] right(){
        if (this.pointer[0]-this.position[0]<575){
            this.pointer[0]+=25;
        }
        else
          this.pointer[0]=0;
        return this.pointer;   
    }
    public int[] left(){
        if (this.pointer[0]-this.position[0]>0){
            this.pointer[0]-=25;
        }
        else
          this.pointer[0]= 575;
        
        return this.pointer;   
    }
    public void draw(PGraphics g) 
    {
      g.image(this.palette, this.position[0], this.position[1]);
      g.noFill();
      g.square(this.pointer[0],this.pointer[1],25);
      getColour(g);
      g.fill(g.red(this.colour), g.green(this.colour), g.blue(this.colour));
      g.rect(this.position[0]+5, this.position[1]+330, 589, 50, 28);
      
    }
    public int getColour(PGraphics g)
    {
      this.colour=g.get(this.pointer[0]+12,this.pointer[1]+12);
      return this.colour;
    }


}
