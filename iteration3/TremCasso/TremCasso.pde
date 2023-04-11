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

Buttons paintB, nextB, setB;
Coloring col;
FBox paint, bottom, boundary, dbound, presetBG, buttonBG, buttonBGH;
ArrayList<FBody> bodies;

int page = 0; // page 0 = canvas; page 1 = colour picker p1, page 2 = color picker p2
int colour; //change this value to set colour of h_avatar in canvas
PGraphics canvas;
boolean            actionMode = false;
boolean            pageChange = false, navChange=false, presetChange=false;

float[] nextPos = {11.8, 21.7};
float[] paintPos = {29.2,11.25};

//float[] paintPos={29.2,11.25};
float[] setPos = {0.5,0};

int preset = 0;

/* Variables */

ArrayList<FBox> seperators = new ArrayList<FBox>();
ArrayList<FBox> seperators2 = new ArrayList<FBox>();

FBox              b1,b2;
FBox              a1;


FBox              next;
FCircle           settings;

PGraphics pickedColour;


//float bPosX, gPosX;
//float vPosX, cPosX;

int         baseColor;
char        orientation = 'v';                    // 'v' or 'h' - Orientation of the color picker

int   rows = 1;                                   // 1 or 2 - Number of hierarchical levels displayed at a time on the screen
float offset = 3.5;                               // Offset from center between two hierarchy levels if rows == 2
int   NUM_SWATCHES = 3;                           // Number of swatches
int   CP_PAGES = 1;                               // 1 or 2 - Number of pages for the color picker
float RATIO = 2;                                  // Ratio of swatch width to swatch spacing
final int   SWATCH_HEIGHT = 130;                  // Height of the rows of swatches
final int   CP_LEFT_INDENT = 50;                  // Space between left of the screen and left edge of color picker
int   CP_RIGHT_INDENT = 280;                      // Space between right of the screen and right edge of color picker

color[]     colors1 = new color[NUM_SWATCHES];    // Array containting the first row of colors 
color[]     colors2 = new color[NUM_SWATCHES];    // Array containting the second row of colors 
color[]     colors3 = new color[NUM_SWATCHES];    // Array containting the first row of colors 
color[]     colors4 = new color[NUM_SWATCHES];    // Array containting the second row of colors 

int         swatchID = -1;                        // Last selected swatch (takes value 0 to NUM_SWATCHES-1)
boolean     newSwatchSelected = false;            // True if a new swatch has been selected, false otherwise.
int swatchWidth = 30;
int animationSpeed = 90; //60
float iter = 0;
int descent = 0;

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


haplyBoard          = new Board(this, Serial.list()[2], 0);

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

paintB = new Buttons(1040, 1400, 0.4, 'h');
nextB = new Buttons(800, 1100, 0.25, 'v');

setB = new Buttons(0, 120, 0.6, 'n');

col = new Coloring();
canvas = createGraphics(1200,900);
world.draw();


colorMode(RGB, 255, 255, 255);

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

        renderingForce = true;
        
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

            
        if (s.h_avatar.isTouchingBody(boundary)) {
            pageChange =false;
            //print(pageChange);
        }
        
        if (s.h_avatar.getX()>27.95){
              paint.setPosition(s.h_avatar.getX()+1.2, paintPos[1]);
              if (s.h_avatar.getX()>29.5 && !pageChange){
                
                if (page == 0){
                  page = 1; 
                }
                else{
                  page = 0;
                }
                pageChange = true;
              }
            }
        
        if (s.h_avatar.getY()>20.45 && s.h_avatar.getX() <= (nextPos[0]+11.3) && page!=0 && CP_PAGES>=2){

               next.setPosition(nextPos[0], s.h_avatar.getY()+1.2);
               if (s.h_avatar.getY()>22.1 && !navChange){
                if (page == 1){
                  page = 2; 
                }
                else{
                  page = 1;
                }
                navChange = true;
              }
            }

        if (next.getY()> nextPos[1] && (s.h_avatar.getY()<=20.45 || s.h_avatar.getX() > (nextPos[0]+11.3))){
          next.setPosition(nextPos[0], nextPos[1]);
        }
        if (paint.getX() > paintPos[0] && s.h_avatar.getX()<=27.95 ){
          paint.setPosition(paintPos[0], paintPos[1]);
        }
        
        if (s.h_avatar.isTouchingBody(dbound) && page!=0) {
            navChange =false;
            //print(pageChange);
        }
        
        //if (sqrt(pow(s.h_avatar.getX()-0.7, 2)+pow(s.h_avatar.getY()+0.5, 2))<=3.5 && (page==1 || page==2)){ ((s.h_avatar.getX()-(3+setPos[0]))<=0) && ((s.h_avatar.getY()-(3+setPos[1]))<=0)
        if (sqrt(pow(s.h_avatar.getX()-0.7, 2)+pow(s.h_avatar.getY()+0.5, 2))<=3.5 && (page==1 || page==2)){
          settings.setPosition(s.h_avatar.getX()-(cos(asin(s.h_avatar.getY()/3.15))*3.15+0.10), setPos[1]);
          if ((s.h_avatar.getX()<=(setPos[0]+1.5)) && !presetChange){
            preset++;
            page=1;
            
           // delete old preset from the screen 
           for (int i = 0; i < NUM_SWATCHES - 1; i++) {
              world.remove(seperators.get(i));
              }
            if (rows == 2){
              for (int i =0; i < NUM_SWATCHES - 1; i++) {
              world.remove(seperators2.get(i));
              }
            }
            world.remove(b1);
            world.remove(b2);
            // world.remove(next);
            world.remove(settings);
            seperators.clear();
            
 
            // second preset parameters
            if (preset == 1){
              offset = 3.5;
              RATIO =  2.5;
              rows = 2;
              NUM_SWATCHES = 5;
              CP_PAGES = 2;
              CP_RIGHT_INDENT = 250;
              colors1 = expand(colors1, NUM_SWATCHES+1);
              colors2 = expand(colors2, NUM_SWATCHES+1);
              colors3 = expand(colors3, NUM_SWATCHES+1);
              colors4 = expand(colors4, NUM_SWATCHES+1);

               for (int i = 0; i<NUM_SWATCHES; i++)
                   colors2[i] = color(255, 255, 255);
            } 
            
            // thrid preset parameters
            else if (preset == 2){  
              seperators.clear();
              seperators2.clear();
              
              offset = 3.5;
              RATIO =  3.0;
              rows = 2;
              NUM_SWATCHES = 7;
              CP_PAGES = 2;
              CP_RIGHT_INDENT = 230;
              colors1 = expand(colors1, NUM_SWATCHES+1);
              colors2 = expand(colors2, NUM_SWATCHES+1);
              colors3 = expand(colors3, NUM_SWATCHES+1);
              colors4 = expand(colors4, NUM_SWATCHES+1);

               for (int i = 0; i<NUM_SWATCHES; i++)
                   colors2[i] = color(255, 255, 255);
              
            } 
            
            // default preset parameters
            else {
                seperators.clear();
                seperators2.clear();
                
                
                offset = 0;
                RATIO =  2.0;
                rows = 1;
                NUM_SWATCHES = 3;
                CP_PAGES = 1;
                CP_RIGHT_INDENT = 280;

                    next.dettachImage();
                    buttonBGH.dettachImage();
              
                for (int i = 0; i<NUM_SWATCHES+1; i++){
                    colors1 = shorten(colors1);
                    colors2 = shorten(colors2);
                    colors3 = shorten(colors3);
                    colors4 = shorten(colors4);
                }
                
                colorMode(RGB, 255, 255, 255);
                for (int i = 0; i<NUM_SWATCHES; i++)
                    colors2[i] = color(255, 255, 255);
                }
             
                background(255);
    
                drawColourPicker();
                if (preset>=3)
                    preset=0;
                
                presetChange = true;
                swatchID = -1;
            }
        }
        if (sqrt(pow(s.h_avatar.getX()-0.7, 2)+pow(s.h_avatar.getY()+0.5, 2))>=3.55 && (page==1 || page==2)){
          presetChange=false;
        }
        if (sqrt(pow(s.h_avatar.getX()-0.7, 2)+pow(s.h_avatar.getY()+0.5, 2))>=3.5 && (page==1 || page==2)){
          settings.setPosition(setPos[0], setPos[1]);
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
    buttonBGH.dettachImage();
    settings.dettachImage();
    presetBG.dettachImage();
    Vec2 pos = hAPI_Fisica.worldToScreen(s.h_avatar.getX(), s.h_avatar.getY());
    s.h_avatar.setFill(red(colour), green(colour), blue(colour));
    col.draw(canvas, red(colour), green(colour), blue(colour), Math.round(pos.x), Math.round(pos.y), 20, actionMode);
    image(canvas,0,0);
    if (!pageChange) {

        PImage img2 = loadImage("assets/buttonBackground2.png");
        buttonBG.attachImage(img2);
        PImage img = loadImage("assets/palette2.png");
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

    PImage img = loadImage("assets/settings.png");
    settings.attachImage(img);
    img=loadImage("assets/presetBackground.png");
    presetBG.attachImage(img);
    
    if(CP_PAGES >=2){
        if (page==1 && !navChange){
            img = loadImage("assets/hButtonBackground2.png");
            buttonBGH.attachImage(img);      
            img = loadImage("assets/next.png");
            next.attachImage(img);
        }
        if (page==2 && !navChange){
            img = loadImage("assets/hButtonBackground2.png");
            buttonBGH.attachImage(img);
            img = loadImage("assets/prev.png");
            next.attachImage(img);
        }
    }
    //next.setSensor(true);
    if (!pageChange) {
        // PImage img = loadImage("assets/canvas.png");
        img = loadImage("assets/exit2.png");
        paint.attachImage(img);
    }
    // Color in the swatches
    drawSwatches();
    if(newSwatchSelected && CP_PAGES == 2){
        animateGradientExpansion();
    }

    // Check if the end effector is over the colorswatches, and if so, update selected color
    if (s.h_avatar.getX() * pixelsPerCentimeter > CP_LEFT_INDENT 
    && s.h_avatar.getX() * pixelsPerCentimeter < width - CP_RIGHT_INDENT
    && s.h_avatar.getY() * pixelsPerCentimeter > height / 2 - offset * pixelsPerCentimeter - SWATCH_HEIGHT / 2 - 30
    && s.h_avatar.getY() * pixelsPerCentimeter < height / 2 - offset * pixelsPerCentimeter - SWATCH_HEIGHT / 2 - 30 + SWATCH_HEIGHT) {
        
        determineSwatchIndex();
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
        // while in page 1, pre-calculate the hues that will be presented in page 2
        colorMode(HSB, 360, 100, 100);
        colors3 = getColors(hue(colour) - 360/(2*NUM_SWATCHES), hue(colour) + 360/(2*NUM_SWATCHES));
        colorMode(RGB, 255, 255, 255);
        colors4 = getShades(colour);
    }
    s.h_avatar.setFill(red(colour), green(colour), blue(colour));
    world.draw();
    setPresets();
}

void page2(){

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

    PImage img = loadImage("assets/settings.png");
    settings.attachImage(img);
    img = loadImage("assets/presetBackground.png");
    presetBG.attachImage(img);
    if(CP_PAGES >= 2){
        if (page==1 && !pageChange){
            img = loadImage("assets/hButtonBackground2.png");
            buttonBGH.attachImage(img);
            img = loadImage("assets/next.png");
            next.attachImage(img);
        }
        if (page==2 && !pageChange){
            img = loadImage("assets/hButtonBackground2.png");
            buttonBGH.attachImage(img);
            img = loadImage("assets/prev.png");
            next.attachImage(img); 
        }
    }
    //next.setSensor(true);
    if (!pageChange) {
        img = loadImage("assets/exit2.png");
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

    } else if (s.h_avatar.getX() * pixelsPerCentimeter > CP_LEFT_INDENT 
    && s.h_avatar.getX() * pixelsPerCentimeter < width - CP_RIGHT_INDENT
    && s.h_avatar.getY() * pixelsPerCentimeter > height / 2 + offset * pixelsPerCentimeter - SWATCH_HEIGHT / 2 - 30
    && s.h_avatar.getY() * pixelsPerCentimeter < height / 2 + offset * pixelsPerCentimeter - SWATCH_HEIGHT / 2 - 30 + SWATCH_HEIGHT) {
        colour = color(get((int)(s.h_avatar.getX() * pixelsPerCentimeter),(int)(s.h_avatar.getY() * pixelsPerCentimeter)));
        if (rows == 2)
            colors4 = getShades(colour);
    }
    s.h_avatar.setFill(red(colour), green(colour), blue(colour));
    world.draw();
    setPresets();

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
        page2();
        break;

    }
}
    
public void drawGUI() {


    buttonBG = new FBox(1.75, 25.5);
    PImage img2 = loadImage("assets/buttonBackground2.png");
    buttonBG.attachImage(img2);
    buttonBG.setFill(0);
    buttonBG.setPosition(29.2,11.25);
    buttonBG.setStatic(true);
    buttonBG.setSensor(true);
    buttonBG.setNoStroke();
    buttonBG.setName("reserved");
    world.add(buttonBG);

    paint = new FBox(1.75, 25.5);
    PImage img = loadImage("assets/palette2.png");
    paint.attachImage(img);
    paint.setFill(0);
    paint.setPosition(paintPos[0], paintPos[1]);
    paint.setStatic(true);
    paint.setSensor(true);
    paint.setNoStroke();
    paint.setName("reserved");
    world.add(paint);

    buttonBGH = new FBox(26.75, 1.75);
    buttonBGH.setPosition(nextPos[0], nextPos[1]);
    buttonBGH.setStatic(true);
    buttonBGH.setSensor(true);
    buttonBGH.setNoStroke();
    buttonBGH.setName("semireserved");
    // PImage img = loadImage("assets/hButtonBackground2.png");
    // buttonBGH.attachImage(img);
    world.add(buttonBGH);

    next = new FBox(26.75, 1.75);
    next.setPosition(nextPos[0], nextPos[1]);
    next.setStatic(true);
    next.setSensor(true);
    next.setNoStroke();
    next.setNoFill();
    next.setName("semireserved");
    world.add(next);

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

    presetBG = new FBox(CP_LEFT_INDENT/pixelsPerCentimeter, worldHeight);
    presetBG.setPosition(CP_LEFT_INDENT/(2*pixelsPerCentimeter), worldHeight/2-0.75);
    presetBG.setStatic(true);
    presetBG.setSensor(true);
    presetBG.setNoStroke();
    presetBG.setNoFill();
    //presetBG.setFill(224,224,224);
    presetBG.setName("semireserved");
    world.add(presetBG);
    
}

/**
* This method draws the color swatches in the color picker.
*/
public void drawSwatches() {
    
    // Calculating the width of a swatch and of a space between the swatches based on the width of the color picker and the ratio of swatch width to space width
    float widthOfSwatch = (width - CP_LEFT_INDENT - CP_RIGHT_INDENT - 1) / (NUM_SWATCHES + 1 / RATIO * (NUM_SWATCHES - 1));
    float widthOfSpace = 1 / RATIO * widthOfSwatch;
    noStroke();
    if(CP_PAGES == 1){
        colors1 = getColors();  // Getting NUM_SWATCHES colors equally spaced out on the color wheel
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
        if(page == 1){
            colors1 = getColors();  // Getting NUM_SWATCHES colors equally spaced out on the color wheel
            colors1 = append(colors1, colors1[0]);  // Append to the end of the colors1 array the first color
            for (int i = 0; i < NUM_SWATCHES; i++) {
                drawGradient((int)(CP_LEFT_INDENT + i * (widthOfSwatch + widthOfSpace)), 
                            (int)(height / 2 - offset * pixelsPerCentimeter - SWATCH_HEIGHT / 2 - 30), 
                            widthOfSwatch, (float)SWATCH_HEIGHT, colors1[i], colors1[i+1]);
            }
            

            if (rows == 2 && newSwatchSelected == false) {
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
        }else if(page == 2){
            if (rows == 2){
                // Bottom Row
                for (int i = 0; i < NUM_SWATCHES; i++) {
                    fill(colors3[i]);
                    rect(CP_LEFT_INDENT + i * (widthOfSwatch + widthOfSpace), height / 2 + offset * pixelsPerCentimeter - SWATCH_HEIGHT / 2 - 30, widthOfSwatch, SWATCH_HEIGHT);
                }
                
                // Top row
                if (rows == 2) {
                    // color the second row
                    for (int i = 0; i < NUM_SWATCHES; i++) {
                        fill(colors4[i]);
                        rect(CP_LEFT_INDENT + i * (widthOfSwatch + widthOfSpace), height / 2 - offset * pixelsPerCentimeter - SWATCH_HEIGHT / 2 - 30, widthOfSwatch, SWATCH_HEIGHT);
                    }
                }
            }else{
                if(colors2.length == NUM_SWATCHES){
                    colorMode(HSB, 360, 100, 100);
                    colors2 = append(colors2, color(0, 100, 100));
                    colorMode(RGB, 255, 255, 255);
                }
                // color the second row
                for (int i = 0; i < NUM_SWATCHES; i++) {
                    colorMode(HSB, 360, 100, 100);
                    // print(i);
                    // print(" | color 1: ");
                    // print(hue(colors2[i]));
                    // print(" | color 2: ");
                    // println(hue(colors2[i+1]));
                    colorMode(RGB, 255, 255, 255);
                    drawGradient((int)(CP_LEFT_INDENT + i * (widthOfSwatch + widthOfSpace)), 
                                (int)(height / 2 + offset * pixelsPerCentimeter - SWATCH_HEIGHT / 2 - 30), 
                                widthOfSwatch, (float)SWATCH_HEIGHT, colors2[i], colors2[i+1]);
                }
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
    float dist = abs(c2 - c1);
    if (dist > 180)
      dist = abs(dist-360);
    else if (dist < -180)
      dist = abs(dist+360);
    colorMode(HSB, 360, 100, 100);                          // Change color mode to HSB to make it easier to choose colors
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
    float dist = abs(hue(c2) - hue(c1));
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
        color c = color(selectedHue, saturation(c1), 100);      // Create the new color
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

    float coef = (float)NUM_SWATCHES/20;
    if(closestHue > hue(colour)){
        colors2 = getColors(closestHue - (coef + 1.05)*360/(NUM_SWATCHES), closestHue + (coef + 0.05)*360/(NUM_SWATCHES));

    }else{
        colors2 = getColors(closestHue - (coef + 0.05)*360/(NUM_SWATCHES), closestHue + (coef + 1.05)*360/(NUM_SWATCHES));
    }

    // extrapolate to add an extra color at the end of the array
    colorMode(HSB, 360, 100, 100);
    color newColor = color(2.0*hue(colors2[NUM_SWATCHES-1]) - hue(colors2[NUM_SWATCHES-2]), 100, 100);
    colors2 = append(colors2, newColor);
    colorMode(RGB, 255, 255, 255);                         // Return to RGB color mode
}

void determineSwatchIndex(){
    int cpWidth = width - CP_LEFT_INDENT - CP_RIGHT_INDENT;
    int xLoc = (int)(s.h_avatar.getX() * pixelsPerCentimeter) - CP_LEFT_INDENT;
    int swatchIDTemp = xLoc / (cpWidth/NUM_SWATCHES);

    if (swatchID != swatchIDTemp){
        swatchID = swatchIDTemp;
        newSwatchSelected = true;
        // println(swatchID);
    }
}

void animateGradientExpansion(){
    float widthOfSwatch = (width - CP_LEFT_INDENT - CP_RIGHT_INDENT - 1) / (NUM_SWATCHES + 1 / RATIO * (NUM_SWATCHES - 1));
    float widthOfSpace = 1 / RATIO * widthOfSwatch;
    float cpWidth = width - CP_LEFT_INDENT - CP_RIGHT_INDENT;
    int yValue = height/2-(int)(offset*pixelsPerCentimeter/2) - 30;

    
    int center = (int)(CP_LEFT_INDENT + swatchID * (widthOfSwatch + widthOfSpace) + widthOfSwatch/2);
    int destinationCenter = CP_LEFT_INDENT + (int)cpWidth/2;
    float steps = (cpWidth-animationSpeed)/(float)(animationSpeed);
    if(swatchWidth <= cpWidth){
        int newCenter = (int)(center + (destinationCenter - center)/(steps) * iter);
        int xValue = newCenter - swatchWidth/2;
        // int xValue = cpWidth/2 + CP_LEFT_INDENT - swatchWidth/2;

        drawGradient(xValue, yValue, swatchWidth, offset*pixelsPerCentimeter, colors2[0], colors2[NUM_SWATCHES]);
        // fill(0, 0, 0);
        // rect(destinationCenter, yValue, 5, 60);
        // rect(center, yValue, 5, 60);
        // rect(newCenter, yValue, 5, 60);
        swatchWidth += animationSpeed;
        iter += 1;
    }
    if(swatchWidth >= cpWidth){
        drawGradient(destinationCenter-(int)cpWidth/2, yValue+descent, cpWidth, offset*pixelsPerCentimeter, colors2[0], colors2[NUM_SWATCHES]);
        descent += animationSpeed/3;
        if(descent >= (int)(offset*pixelsPerCentimeter)){
            newSwatchSelected = false;
            swatchWidth = 30;
            iter = 0;
            descent = 0;
        }
    }
}


public void drawColourPicker() {
    
    // Computingthe width of a color swatch and the width of the space between swatches
    float widthOfSwatch = (width - CP_LEFT_INDENT - CP_RIGHT_INDENT - 1) / (NUM_SWATCHES + 1 / RATIO * (NUM_SWATCHES - 1));
    float widthOfSpace = 1 / RATIO * widthOfSwatch;
    float ppcm = pixelsPerCentimeter;  // shortcut for pixelsPerCentimeter to make for shorter functions
    
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
          
            a1 = new FBox(widthOfSpace / ppcm,(SWATCH_HEIGHT + 10) / ppcm);
            //g3.setPosition((CP_LEFT_INDENT + (i + 1) * (widthOfSwatch + widthOfSpace) - widthOfSpace / 2) / ppcm, edgeTopLeftY + worldHeight / 2.0 - offset); 
            a1.setPosition((CP_LEFT_INDENT + (i + 1) * (widthOfSwatch + widthOfSpace) - widthOfSpace / 2) / ppcm, worldHeight / 2.0 - offset - 2);
            a1.setFillColor(color(255,255,255));
            a1.setStrokeWeight(12);
            a1.setStrokeColor(color(0,0,0));
            a1.setStaticBody(true);
            a1.setName("swatchSpacer");
            seperators.add(a1);
            world.add(seperators.get(i));
        }
        
        // Draw Right edge
        b2 = new FBox(0.3, 13.0);
        b2.setPosition((width - CP_RIGHT_INDENT) / ppcm, edgeTopLeftY + worldHeight / 2.0 - 1.8); 
        b2.setFill(0);
        b2.setNoStroke();
        b2.setStaticBody(true);
        world.add(b2);

        
        // Second Row of swatches
        if (rows == 2) {
            for (int i =0; i < NUM_SWATCHES - 1; i++) {
                a1 = new FBox(widthOfSpace / ppcm,(SWATCH_HEIGHT + 10) / ppcm);
                a1.setPosition((CP_LEFT_INDENT + (i + 1) * (widthOfSwatch + widthOfSpace) - widthOfSpace / 2) / ppcm, worldHeight / 2.0 + offset - 2); 
                a1.setFillColor(color(255,255,255));
                a1.setStrokeWeight(12);
                a1.setStrokeColor(color(0,0,0));
                a1.setStaticBody(true);
                a1.setName("swatchSpacer");
                seperators2.add(a1);
                world.add(seperators2.get(i));
            }     
        }
        
        
        
        settings = new FCircle(5);
        settings.setPosition(setPos[0], setPos[1]);
        settings.setStatic(true);
        settings.setSensor(true);
        settings.setNoStroke();
        settings.setNoFill();
        settings.setName("semireserved");
        world.add(settings);
        
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
    PVector f = paintB.applyForces(5, 15, pos.x, fEE);

    if (page !=0 && s.h_avatar.getX()<=(nextPos[0]+11.3) && CP_PAGES>=2) {
        f = nextB.applyForces( -10, -20, pos.y, fEE);
    }
    if (page!=0 && sqrt(pow(s.h_avatar.getX()-0.7, 2)+pow(s.h_avatar.getY()+0.5, 2))<=4){
      f= setB.applyForces( -3, -5, pos.x, fEE);
    }
    fEE.set(f);
}

public void setPresets(){
  fill(255);
  noStroke();
  rect( 5 , worldHeight*pixelsPerCentimeter/4, CP_LEFT_INDENT-10, worldHeight*pixelsPerCentimeter/2, 10);
  textSize(35);
  fill(0);
  char[] mode={'M', 'O', 'D', 'E', ' ', Character.forDigit((preset+1), 10)};
  float[] textPos={15, worldHeight*pixelsPerCentimeter/2-100,  30};// last value is spacing between letters
  for (int i=0; i<mode.length; i++)
    {
      text(mode[i], textPos[0], textPos[1]);
      textPos[1]+= textPos[2];
    }
}

PVector device_to_graphics(PVector deviceFrame) {
    return deviceFrame.set( -deviceFrame.x, deviceFrame.y);
}


PVector graphics_to_device(PVector graphicsFrame) {
    return graphicsFrame.set( -graphicsFrame.x, graphicsFrame.y);
}


/* end helper functions section ***************************************************************************************/
