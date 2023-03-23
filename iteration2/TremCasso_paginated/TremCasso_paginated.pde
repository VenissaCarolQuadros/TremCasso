/**
**********************************************************************************************************************
* @file       TremCasso.pde
* @author     Team TremCasso
* @version    V1.0
* @date       17-March-2023
* @brief      2nd iteration of an accessible drawing tool/colour picker for people with tremors
**********************************************************************************************************************
* @attention
*
*
**********************************************************************************************************************
*/



/* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/  

Buttons paintB, nextB;
Coloring col;
FBox paint, bottom, boundary, dbound;
ArrayList<FBody> bodies;

int page = 0; // page 0 = canvas; page 1 = colour picker
int colour; //change this value to set colour of h_avatar in canvas
PGraphics canvas;
boolean            actionMode = false;
boolean            pageChange = false, navChange=false;


/* Variables */
FBox              b1,b2;
FBox              v1,v2,v3,v4,v5,v6,v7,v8,v9,v10;
FBox              g1,g2,g3;
FBox              c1,c2,c3,c4,c5,c6,c7,c8,c9,c10;

FBox              next;


/* graphical elements */
/*
PShape color1, color2, color3, color4, color5, color6;
PShape color11, color12,color13,color14,color15;
PShape color21, color22,color23,color24,color25;
PShape color31, color32,color33,color34,color35;
PShape color41, color42,color43,color44,color45;
PShape color51, color52,color53,color54,color55;
*/

PGraphics pickedColour;


//float bPosX, gPosX;
//float vPosX, cPosX;

int         baseColor;
char        orientation = 'v';                    // 'v' or 'h' - Orientation of the color picker
final int   rows = 2;                             // 1 or 2 - Number of hierarchical levels displayed at a time on the screen
float       offset = 3.5;                         // Offset from center between two hierarchy levels if rows == 2
final int   NUM_SWATCHES = 8;                     // Number of swatches
final int   CP_PAGES = 2;                         // 1 or 2 - Number of pages for the color picker
final float RATIO = 2;                            // Ratio of swatch width to swatch spacing
final int   SWATCH_HEIGHT = 130;                  // Height of the rows of swatches
final int   CP_LEFT_INDENT = 20;                  // Space between left of the screen and left edge of color picker
final int   CP_RIGHT_INDENT = 280;                // Space between right of the screen and right edge of color picker
color[]     colors1 = new color[NUM_SWATCHES];    // Array containting the first row of colors 
color[]     colors2 = new color[NUM_SWATCHES];    // Array containting the second row of colors 



/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 


/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                      = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerMeter                      = 4000.0;
float             radsPerDegree                       = 0.01745;
float             pixelsPerCentimeter                 = 40.0;


/* end effector radius in meters */
float             rEE                                 = 0.004;
float             rEEContact                          = 0.006;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* device graphical position */
PVector           deviceOrigin                        = new PVector(0, 0);

/* World boundaries reference */
final int         worldPixelWidth                     = 1000;
final int         worldPixelHeight                    = 900;

FWorld            world;
float             worldWidth                          = 25.5;  
float             worldHeight                         = 25.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

/* Initialization of virtual tool */
HVirtualCoupling  s;

/* end elements definition *********************************************************************************************/



/* setup section *******************************************************************************************************/
void setup(){
/* put setup code here, run once: */

/* screen size definition */
size(1200, 900);
//swatchSize = ((1200)/ (distanceBetweenSwatches*NoOfSwatches))/32;
/* device setup */

/**  
* The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
* In the base example, a connection is setup to the first detected serial device, this parameter can be changed
* to explicitly state the serial port will look like the following for different OS:
*
*      windows:      haplyBoard = new Board(this, "COM10", 0);
*      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
*      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
*/
haplyBoard          = new Board(this, Serial.list()[0], 0);
widgetOne           = new Device(widgetOneID, haplyBoard);
pantograph          = new Pantograph();


widgetOne.set_mechanism(pantograph);

widgetOne.add_actuator(1, CCW, 2);
widgetOne.add_actuator(2, CW, 1);

widgetOne.add_encoder(1, CCW, 241, 10752, 2);
widgetOne.add_encoder(2, CW, -61, 10752, 1);

widgetOne.device_set_parameters();

/* 2D physics scaling and world creation */
hAPI_Fisica.init(this); 
hAPI_Fisica.setScale(pixelsPerCentimeter); 
world               = new FWorld();

deviceOrigin.add(worldPixelWidth/2, 0);

if (rows == 1){
offset = 0;
}

drawGUI();
drawColourPicker();
/* Setup the Virtual Coupling Contact Rendering Technique */
s                   = new HVirtualCoupling((0.75)); 
s.h_avatar.setName("reserved");
s.h_avatar.setSensor(false);
//s.h_avatar.setDensity(2); 
s.h_avatar.setFill(255,0,0); 
s.init(world, edgeTopLeftX + worldWidth / 2, edgeTopLeftY + 2); 

/* World conditions setup */
//world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
world.setEdgesRestitution(.1);
world.setEdgesFriction(0.1);

paintB = new Buttons(1040, 1300, 'h');
nextB = new Buttons(800, 1100, 'v');

col = new Coloring();
canvas = createGraphics(1200,900);
world.draw();

for (int i = 0; i<NUM_SWATCHES; i++)
colors2[i] = color(255, 255, 255);

colour=color(0,0,0);
/* setup framerate speed */
frameRate(baseFrameRate);

/* setup simulation thread to run at 1kHz */ 
SimulationThread st = new SimulationThread();
scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}



/* draw section ********************************************************************************************************/
void draw() {
/*put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
    
    if(renderingForce == false){
        background(255);
        pageSelector(); 
        update_animation(angles.x*radsPerDegree, angles.y*radsPerDegree, posEE.x, posEE.y);
    }
}
/* end draw section ****************************************************************************************************/

void keyPressed() {
    if (key ==  ' ') {
        if (actionMode)
            actionMode = false;
        else
            actionMode = true;
    }
}
    
    
/* simulation section ***********************************************************************************************/ 
class SimulationThread implements Runnable{
    
    public void run() {
        /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */ 
        renderingForce =true;
        
        if (haplyBoard.data_available()) {
            /* GET END-EFFECTOR STATE (TASK SPACE) */
            widgetOne.device_read_data();
            
            angles.set(widgetOne.get_device_angles()); 
            posEE.set(widgetOne.get_device_position(angles.array()));
            
            posEE.set(posEE.copy().mult(200)); 
        }
            
            
        s.setToolPosition(edgeTopLeftX + worldWidth / 2 - (posEE).x, edgeTopLeftY + (posEE).y - 4); 
        s.updateCouplingForce();
        
        fEE.set( - s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
        fEE.div(100000);//dynes to newtons
        
        forceSetter();
        world.step(1.0f / 1000.0f);
        
        torques.set(widgetOne.set_device_torques(fEE.array()));
        widgetOne.device_write_torques(); 

        /*

        if (s.h_avatar.isTouchingBody(paint)) {
            if (page == 0 &&!pageChange){
                page = 1;
                pageChange =true;
                //print(pageChange);
            }
            if (page == 1 &&!pageChange){
                page = 0;
                pageChange =true;
                //print(pageChange);
            }

            
        }*/
            
        if (s.h_avatar.isTouchingBody(boundary)) {
            pageChange =false;
            //print(pageChange);
        }

        /*  
        if (s.h_avatar.isTouchingBody(next)) {
            if (page == 1 &&!navChange){
                page = 2; 
                navChange =true;
                //print(pageChange);
            }
            if (page == 2 &&!navChange){
                page = 1;
                navChange =true;
                //print(pageChange);
            }
            
        }*/
        
        if (s.h_avatar.getX()>27.95){
              paint.setPosition(s.h_avatar.getX()+1.2, 11.25);
              if (s.h_avatar.getX()>29.5 && !pageChange){
                
                if (page == 0){
                  page = 1; 
                }
                else{
                  page = 0;
                }
                pageChange =true;
              }
            }
        
        if (s.h_avatar.getY()>20.45 && s.h_avatar.getX() < 27 && page!=0){
               next.setPosition(13.5, s.h_avatar.getY()+1.2);
               if (s.h_avatar.getY()>22.1 && !navChange){
                if (page == 1){
                  page = 2; 
                }
                else{
                  page = 1;
                }
                navChange =true;
              }
            }
        
        if (s.h_avatar.isTouchingBody(dbound) && page!=0) {
            navChange =false;
            //print(pageChange);
        }
        
        

            
        renderingForce = false;
    }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/
    
    
    
void page0(){
    bodies = world.getBodies();
    //print(bodies);
    for (FBody b: bodies) { 
        if (b.getName()!= "reserved") {
            b.setSensor(true);
            b.setNoFill();
            b.setNoStroke();
        }
    }

    next.dettachImage();
    Vec2 pos = hAPI_Fisica.worldToScreen(s.h_avatar.getX(), s.h_avatar.getY());
    s.h_avatar.setFill(red(colour), green(colour), blue(colour));
    col.draw(canvas, red(colour), green(colour), blue(colour), Math.round(pos.x), Math.round(pos.y), 20, actionMode);
    image(canvas,0,0);
    if (!pageChange) {
        PImage img =loadImage("assets/paint.png");
        paint.attachImage(img);
    }
    world.draw();
    
}
void page1(){
    bodies = world.getBodies();
    //print(bodies);
    for (FBody b: bodies) { 
        if (b.getName()!= "reserved") {
            if (b.getName() == "swatchSpacer") {
                b.setSensor(false);
                b.setFillColor(color(255,255,255));
                b.setStrokeWeight(12);
                b.setStrokeColor(color(0,0,0));
            }
            else if (b.getName()=="semireserved"){
              b.setSensor(true);
            }
            else{

                b.setSensor(false);
                b.setFill(0);
            }
        }
    }
    if (page==1 && !navChange){
      PImage img = loadImage("assets/down.png");
      next.attachImage(img);
    }
    if (page==2 && !navChange){
      PImage img = loadImage("assets/up.png");
      next.attachImage(img);
    }
    //next.setSensor(true);
    if (!pageChange) {
        PImage img = loadImage("assets/canvas.png");
        paint.attachImage(img);
    }
    // Color in the swatches
    drawSwatches();
    
    // Check if the end effector is over the colorswatches, and if so, update selected color
    if (s.h_avatar.getX() * pixelsPerCentimeter > CP_LEFT_INDENT 
    && s.h_avatar.getX() * pixelsPerCentimeter < width - CP_RIGHT_INDENT
    && s.h_avatar.getY() * pixelsPerCentimeter > height / 2 - offset * pixelsPerCentimeter - SWATCH_HEIGHT / 2 - 30
    && s.h_avatar.getY() * pixelsPerCentimeter < height / 2 - offset * pixelsPerCentimeter - SWATCH_HEIGHT / 2 - 30 + SWATCH_HEIGHT) {
    
        colour = color(get((int)(s.h_avatar.getX() * pixelsPerCentimeter),(int)(s.h_avatar.getY() * pixelsPerCentimeter)));
        if (rows == 2)
            colors2 = getShades(colour);
            if(CP_PAGES == 2)
                updateRow2();
    } else if (s.h_avatar.getX() * pixelsPerCentimeter > CP_LEFT_INDENT 
    && s.h_avatar.getX() * pixelsPerCentimeter < width - CP_RIGHT_INDENT
    && s.h_avatar.getY() * pixelsPerCentimeter > height / 2 + offset * pixelsPerCentimeter - SWATCH_HEIGHT / 2 - 30
    && s.h_avatar.getY() * pixelsPerCentimeter < height / 2 + offset * pixelsPerCentimeter - SWATCH_HEIGHT / 2 - 30 + SWATCH_HEIGHT) {
        colour = color(get((int)(s.h_avatar.getX() * pixelsPerCentimeter),(int)(s.h_avatar.getY() * pixelsPerCentimeter)));
    }
    s.h_avatar.setFill(red(colour), green(colour), blue(colour));
    world.draw();
}
    
    
void pageSelector() {
    switch(page){
        case 0:
        page0();
        break;
        
        case 1 : 
        page1();
        break;

        
        case 2:
        page1();
        break;

    }
}
    
public void drawGUI() {
    /*

    bottom = new FBox(2, 25.5);
    bottom.setFill(0, 114, 160);
    bottom.setPosition(29,12.5);
    bottom.setStatic(true);
    bottom.setSensor(true);
    bottom.setNoStroke();
    bottom.setName("reserved");
    world.add(bottom);
    
    */
    paint = new FBox(1.75, 25.5);
    PImage img =loadImage("assets/paint.png");
    paint.attachImage(img);
    paint.setFill(0);
    paint.setPosition(29.2,11.25);
    paint.setStatic(true);
    paint.setSensor(true);
    paint.setNoStroke();
    paint.setName("reserved");
    world.add(paint);
    
    boundary = new FBox(0.5, 25.5);
    boundary.setNoFill();
    boundary.setPosition(26.9,12.5);
    boundary.setStatic(true);
    boundary.setSensor(true);
    boundary.setNoStroke();
    boundary.setName("reserved");
    world.add(boundary);
    

    dbound= new FBox(26.75, 0.5);
    dbound.setNoFill();
    dbound.setPosition(13.5, 19.8);
    dbound.setStatic(true);
    dbound.setSensor(true);
    dbound.setNoStroke();
    dbound.setName("reserved");
    world.add(dbound);
    
    /*
    settings=newFBox(3.5, 1.90);
    PImage img1=loadImage("assets/settings.png");
    img1.resize(0,70);
    settings.attachImage(img1);
    settings.setFill(255,255,255);
    settings.setPosition(18,16.5);
    settings.setStatic(true);
    settings.setSensor(true);
    settings.setNoStroke();
    world.add(settings);
    */
}

/**
* This method draws the color swatches in the color picker.
*/
public void drawSwatches(){
  colors1 = getColors();  // Getting NUM_SWATCHES colors equally spaced out on the color wheel
  // Calculating the width of a swatch and of a space between the swatches based on the width of the color picker and the ratio of swatch width to space width
  float widthOfSwatch = (width-CP_LEFT_INDENT-CP_RIGHT_INDENT-1)/(NUM_SWATCHES + 1/RATIO*(NUM_SWATCHES-1));
  float widthOfSpace = 1/RATIO*widthOfSwatch;
  noStroke();
  // color the first row
  for (int i = 0; i<NUM_SWATCHES; i++){
    fill(colors1[i]);
    rect(CP_LEFT_INDENT + i*(widthOfSwatch + widthOfSpace), height/2-offset*pixelsPerCentimeter-SWATCH_HEIGHT/2 + 10, widthOfSwatch, SWATCH_HEIGHT);
  }
  
  if(rows == 2){
    // color the second row
    for (int i = 0; i<NUM_SWATCHES; i++){
      fill(colors2[i]);
      rect(CP_LEFT_INDENT + i*(widthOfSwatch + widthOfSpace), height/2+offset*pixelsPerCentimeter - SWATCH_HEIGHT/2 + 10, widthOfSwatch, SWATCH_HEIGHT);
    }
  }
}

/**
* When the method is called, it returns an array of colors of length NoOfSwatches.
*/
color[] getColors(){
  color[] colors = new color[NUM_SWATCHES];              // Makes a new array of length NUM_SWATCHES 
  colorMode(HSB, 360, 100, 100);                         // Change color mode to HSB to make it easier to choose colors
  for (int i = 0; i<NUM_SWATCHES; i++){                  // Compute the new color and store it in each array space
    colors[i] = color((360/NUM_SWATCHES)*i, 100, 100);
  }
  colorMode(RGB, 255, 255, 255);                         // Return to RGB color mode
  return colors;
}

/**
* When the method is called with parameters r, g, and b, it returns 
* an array of hues based on the given color of length NoOfSwatches.
*/
color[] getShades(int r, int g, int b){
  color[] colors = new color[NUM_SWATCHES];              // Makes a new array of length NUM_SWATCHES 
  color baseColor = color(r, g, b);                      // Store the base color in a color object type
  colorMode(HSB, 360, 100, 100);                         // Change color mode to HSB to make it easier to choose colors
  for (int i = 0; i<NUM_SWATCHES; i++){                  // Create desired number of shades based on the color
    colors[i] = color(hue(baseColor), 100, i*100/(NUM_SWATCHES-1));
  }
  colorMode(RGB, 255, 255, 255);                         // Return to RGB color mode
  return colors;
}

/**
* When the method is called with a color parameter, it returns an 
* array of hues based on the given color of length NoOfSwatches.
*/
color[] getShades(color baseColor){
  color[] colors = new color[NUM_SWATCHES];              // Makes a new array of length NUM_SWATCHES 
  colorMode(HSB, 360, 100, 100);                         // Change color mode to HSB to make it easier to choose colors
  for (int i = 0; i<NUM_SWATCHES; i++){                  // Create desired number of shades based on the color
    colors[i] = color(hue(baseColor), 100, i*100/(NUM_SWATCHES-1));
  }
  colorMode(RGB, 255, 255, 255);                         // Return to RGB color mode
  return colors;
}

/**
* Draws a linear gradient at desired (x,y) location with width w and height h from color c1 to color c2. 
*/
void drawGradient(int x, int y, float w, float h, color c1, color c2){
  noFill();                                        // Set to not fill in
  colorMode(HSB, 360, 100, 100);                   // Change color mode to HSB to make it easier to choose colors
  float step = (hue(c2) - hue(c1)) / w ;           // Compute the size of the step from one color swatch to the next based on the number of desired swatches
  for (int i = 0; i <= w; i++) {                   
    color c = color(hue(c1)+step*i, 100, 100);     // Create the new color
    stroke(c);                                     // Set the new color as the active color
    line(x+i, y, x+i, y+h);                        // Draw a line of the chosen color at the adequate location
  }
}


/**
* This method draws the color swatches in the color picker.
*/
public void drawSwatches() {
    colors1 = getColors();  // Getting NUM_SWATCHES colors equally spaced out on the color wheel
    // Calculating the width of a swatch and of a space between the swatches based on the width of the color picker and the ratio of swatch width to space width
    float widthOfSwatch = (width - CP_LEFT_INDENT - CP_RIGHT_INDENT - 1) / (NUM_SWATCHES + 1 / RATIO * (NUM_SWATCHES - 1));
    float widthOfSpace = 1 / RATIO * widthOfSwatch;
    noStroke();
    if(CP_PAGES == 1){
        // color thefirst row
        for (int i = 0; i < NUM_SWATCHES; i++) {
            fill(colors1[i]);
            rect(CP_LEFT_INDENT + i * (widthOfSwatch + widthOfSpace), height / 2 - offset * pixelsPerCentimeter - SWATCH_HEIGHT / 2 - 30, widthOfSwatch, SWATCH_HEIGHT);
        }
        
        if (rows == 2) {
            // color the second row
            for (int i = 0; i < NUM_SWATCHES; i++) {
                fill(colors2[i]);
                rect(CP_LEFT_INDENT + i * (widthOfSwatch + widthOfSpace), height / 2 + offset * pixelsPerCentimeter - SWATCH_HEIGHT / 2 - 30, widthOfSwatch, SWATCH_HEIGHT);
            }
        }
    }else if(CP_PAGES == 2){
        colors1 = append(colors1, colors1[0]);  // Append to the end of the colors1 array the first color
        for (int i = 0; i < NUM_SWATCHES; i++) {
            drawGradient((int)(CP_LEFT_INDENT + i * (widthOfSwatch + widthOfSpace)), 
                        (int)(height / 2 - offset * pixelsPerCentimeter - SWATCH_HEIGHT / 2 - 30), 
                        widthOfSwatch, (float)SWATCH_HEIGHT, colors1[i], colors1[i+1]);
        }
        
        if (rows == 2) {
            if(colors2.length == NUM_SWATCHES){
                colorMode(HSB, 360, 100, 100);
                colors2 = append(colors2, color(0, 100, 100));
                colorMode(RGB, 255, 255, 255);
            }
            // color the second row
            for (int i = 0; i < NUM_SWATCHES; i++) {
                drawGradient((int)(CP_LEFT_INDENT + i * (widthOfSwatch + widthOfSpace)), 
                            (int)(height / 2 + offset * pixelsPerCentimeter - SWATCH_HEIGHT / 2 - 30), 
                            widthOfSwatch, (float)SWATCH_HEIGHT, colors2[i], colors2[i+1]);
            }

        }
    }
}

/**
* When the method is called, it returns an array of colors of length NoOfSwatches.
*/
color[] getColors() {
    color[] colors = new color[NUM_SWATCHES];              // Makes a new array of length NUM_SWATCHES 
    colorMode(HSB, 360, 100, 100);                         // Change color mode to HSBto make it easier to choose colors
    for (int i = 0; i < NUM_SWATCHES; i++) {                // Compute the new color and store it in each array space
        colors[i] = color((360 / NUM_SWATCHES) * i, 100, 100);
    }
    colorMode(RGB, 255, 255, 255);                         // Return to RGB color mode
    return colors;
}

/**
* When the method is called, it returns an array of colors between c1 and c2 of length NoOfSwatches.
*/
color[] getColors(float c1, float c2) {
    color[] colors = new color[NUM_SWATCHES];              // Makes a new array of length NUM_SWATCHES 
    float dist = c2 - c1;
    if (dist > 180)
      dist = abs(dist-360);
    else if (dist < -180)
      dist = abs(dist+360);
    colorMode(HSB, 360, 100, 100);                         // Change color mode to HSBto make it easier to choose colors
    for (int i = 0; i < NUM_SWATCHES; i++) {                // Compute the new color and store it in each array space
        float colorHue = c1 + (dist / NUM_SWATCHES) * i;
        if(colorHue < 0)
            colorHue += 360;
        if(colorHue >= 360)
            colorHue -= 360;
        colors[i] = color(colorHue, 100, 100);
    }
    colorMode(RGB, 255, 255, 255);                         // Return to RGB color mode
    return colors;
}

/**
* When the method is called with parameters r, g, and b, it returns 
* an array of hues based on the given color of length NoOfSwatches.
*/
color[] getShades(int r, int g, int b) {
    color[] colors = new color[NUM_SWATCHES];              // Makes a new array of length NUM_SWATCHES 
    color baseColor = color(r, g, b);                      // Store the base color in a color object type
    colorMode(HSB, 360, 100, 100);                         // Change color mode to HSB to makeit easier to choose colors
    for (int i =0; i < NUM_SWATCHES; i++) {                // Create desired number of shades based on the color
        colors[i] = color(hue(baseColor), 100, i * 100 / (NUM_SWATCHES - 1));
    }
    colorMode(RGB, 255, 255, 255);                         // Return to RGB color mode
    return colors;
}

/**
* When the method is called with a color parameter, it returns an 
* array of hues based on the given color of length NoOfSwatches.
*/
color[] getShades(color baseColor) {
    color[] colors = new color[NUM_SWATCHES];              // Makes a new array of length NUM_SWATCHES
    colorMode(HSB, 360, 100, 100);                         // Change color mode to HSB to make it easier to choose colors
    for (int i = 0; i < NUM_SWATCHES; i++) {                // Create desired number of shades based on the color
        colors[i] = color(hue(baseColor), 100, i * 100 / (NUM_SWATCHES - 1));
    }
    colorMode(RGB, 255, 255, 255);                         // Return to RGB color mode
    return colors;
}

/**
* Draws a linear gradient at desired(x,y) location with width w and height h from color c1 to color c2.
*/
void drawGradient(int x, int y, float w, float h, color c1, color c2) {
    noFill();   // Set to not fill in
    colorMode(HSB, 360, 100, 100);                          // Change color mode to HSB to make it easier to choose colors
    float dist = hue(c2) - hue(c1);
    if (dist > 180)
      dist = abs(dist-360);
    else if (dist < -180)
      dist = abs(dist+360);
    float step = abs(dist) / w;              // Compute the size of the step from one color swatch to the next based on the number of desired swatches
    for (int i = 0; i <= w; i++) {
        float selectedHue = hue(c1) + step * i;
        if (selectedHue >= 360.0)
          selectedHue -= 360.0;
        else if (selectedHue < 0)
          selectedHue += 360.0;
        color c = color(selectedHue, 100, 100);      // Create the new color
        stroke(c);                                          // Set the new color as the active color
        line(x + i, y, x + i, y + h);                       // Draw a line of the chosen color at the adequate location
    }
    colorMode(RGB, 255, 255, 255);                         // Return to RGB color mode
}

/**
* computes the colors of the gradient for the second row.
*/
void updateRow2(){
    colorMode(HSB, 360, 100, 100);
    // Figure out which color to expand
    float closestHue = hue(colors1[0]);
    for (int i = 0; i < NUM_SWATCHES+1; i++){
        if(abs(hue(colors1[i])-hue(colour)) < abs(closestHue-hue(colour))){
            closestHue = hue(colors1[i]);
        }
    }
    if(closestHue > hue(colour)){
        colors2 = getColors(closestHue - (1.5)*360/(NUM_SWATCHES), closestHue + (0.5)*360/(NUM_SWATCHES));

    }else{
        colors2 = getColors(closestHue - (0.5)*360/(NUM_SWATCHES), closestHue + (1.5)*360/(NUM_SWATCHES));
    }
    // extrapolate to add an extra color at the end of the array
    colorMode(HSB, 360, 100, 100);
    color newColor = color(2.0*hue(colors2[NUM_SWATCHES-1]) - hue(colors2[NUM_SWATCHES-2]), 100, 100);
    colors2 = append(colors2, newColor);
    colorMode(RGB, 255, 255, 255);                         // Return to RGB color mode
}


public void drawColourPicker() {
    
    // Computingthe width of a color swatch and the width of the space between swatches
    float widthOfSwatch = (width - CP_LEFT_INDENT - CP_RIGHT_INDENT - 1) / (NUM_SWATCHES + 1 / RATIO * (NUM_SWATCHES - 1));
    float widthOfSpace = 1 / RATIO * widthOfSwatch;
    float ppcm =pixelsPerCentimeter;  // shortcut for pixelsPerCentimeter to make for shorter functions
    
    // Draw leftedge
    b1 = new FBox(0.3, 25.5);
    b1.setPosition(CP_LEFT_INDENT / pixelsPerCentimeter, edgeTopLeftY + worldHeight / 2.0 - 0.8); 
    b1.setFill(0);
    b1.setNoStroke();
    b1.setStaticBody(true);
    world.add(b1);
    
    // IF orientation is vertical:
    if (orientation == 'v') {
        //print(offset);
        // Create N-1 separations between each swatch.FBoxes with white color fill and black stroke of weight 12.
        for (int i =0; i < NUM_SWATCHES - 1; i++) {
            g3 = new FBox(widthOfSpace / ppcm,(SWATCH_HEIGHT + 10) / ppcm);
            //g3.setPosition((CP_LEFT_INDENT + (i + 1) * (widthOfSwatch + widthOfSpace) - widthOfSpace / 2) / ppcm, edgeTopLeftY + worldHeight / 2.0 - offset); 
            g3.setPosition((CP_LEFT_INDENT + (i + 1) * (widthOfSwatch + widthOfSpace) - widthOfSpace / 2) / ppcm, worldHeight / 2.0 - offset - 2);
            g3.setFillColor(color(255,255,255));
            g3.setStrokeWeight(12);
            g3.setStrokeColor(color(0,0,0));
            g3.setStaticBody(true);
            g3.setName("swatchSpacer");
            world.add(g3);
        }
        
        // Draw Right edge
        b1 = new FBox(0.3, 13.0);
        b1.setPosition((width - CP_RIGHT_INDENT) / ppcm, edgeTopLeftY + worldHeight / 2.0 - 1.8); 
        b1.setFill(0);
        b1.setNoStroke();
        b1.setStaticBody(true);
        world.add(b1);
        
        // Second Row of swatches
        if (rows == 2) {
            for (int i =0; i < NUM_SWATCHES - 1; i++) {
                g3 = new FBox(widthOfSpace / ppcm,(SWATCH_HEIGHT + 10) / ppcm);
                g3.setPosition((CP_LEFT_INDENT + (i + 1) * (widthOfSwatch + widthOfSpace) - widthOfSpace / 2) / ppcm, worldHeight / 2.0 + offset - 2); 
                g3.setFillColor(color(255,255,255));
                g3.setStrokeWeight(12);
                g3.setStrokeColor(color(0,0,0));
                g3.setStaticBody(true);
                g3.setName("swatchSpacer");
                world.add(g3);
            }     
        }
        
        
        next = new FBox(26.75, 1.75);
        next.setPosition(13.5, 21.7);
        next.setStatic(true);
        next.setSensor(true);
        next.setNoStroke();
        next.setName("semireserved");
        world.add(next);
        
    } 
}

void update_animation(float th1, float th2, float xE, float yE) {
    xE = pixelsPerMeter * xE;
    yE = pixelsPerMeter * yE;
    
    th1 = 3.14 -th1;
    th2 = 3.14 -th2;
    
    translate(xE, yE);
    
}

public void forceSetter() {
    Vec2 pos = hAPI_Fisica.worldToScreen(s.h_avatar.getX(), s.h_avatar.getY());
    PVector f = paintB.applyForces(5, 10, pos.x, fEE);
    if (page !=0) {
        f = nextB.applyForces( -10, -30, pos.y, fEE);
    }
    fEE.set(f);
}

PVector device_to_graphics(PVector deviceFrame) {
    return deviceFrame.set( -deviceFrame.x, deviceFrame.y);
}


PVector graphics_to_device(PVector graphicsFrame) {
    return graphicsFrame.set( -graphicsFrame.x, graphicsFrame.y);
}


/* end helper functions section ***************************************************************************************/
