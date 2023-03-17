import processing.core.*;

public class Coloring extends PApplet{
    public Coloring(){}
    public void draw(PGraphics canvas, float r, float g, float b, int x, int y, int rad, boolean drawingMode) 
    { 
      canvas.beginDraw();
      canvas.noStroke();
      canvas.fill(color(r,g,b));
      if (drawingMode){
        canvas.circle(Math.round(x), Math.round(y), rad);
      }
      canvas.endDraw();
      /*
      g.beginDraw();
      g.loadPixels();
      //print(x,y, "\n");
      int r=100;
      if (y>=0 && x>=0){
        //g.pixels[x*1000+y] = color(colour);
        int xmin = x - r;
        int xmax = x + r;
        int ymin = y - r;
        int ymax = y + r;
        print(xmin, xmax, ymin, ymax, "\n");
      for (int i = xmin; i < xmax ; i++)
      {
          for (int j = ymin; j < ymax; y=j++)
          {
              double dx = i - x;
              double dy = j - y;
              double distanceSquared = dx * dx + dy * dy;
      
              if (distanceSquared <= 100*100)
              {
                  //print(x*1000+y, "\n");
                  g.pixels[x*1000+y]=color(0);
              }
          }
      }
      }
      //print(y*1000+x, "\n");
      /*for (int i = 0; i < (y*1000+x); i++) {
      g.pixels[i] = color(colour);
      }
      
      g.updatePixels();
      g.endDraw();
      */
      
    }
    
    
}
