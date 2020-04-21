// =============================================================================
//
// Simple Wireframe Renderer for Nuts and Volts AGI
//
// Based largely on code from scratchpixel.com rasterization example
//
// =============================================================================

// ----------------------------------------------
// includes
// ----------------------------------------------
#include <DueTimer.h> 
#include <XYscope.h>

#include "cow.h"
#include "geometry.h"

// ----------------------------------------------
// defines
// ----------------------------------------------
#define SCREENW         4095
#define SCREENH         4095
#define HALFW           2048
#define HALFH           2048 

#define FOV              64

#define SKIP_TICKS       20.0                // 50fps
#define MAX_FRAMESKIP     1

XYscope XYscope;

//   +---------Begin Critical Interrupt Service Routines ------------+
//  |  These routines MUST be declared as shown at the top of the |
//  |      user's main line code for all XYscope Projects!      |
//  +---------------------------------------------------------------+
//  |                               |
//  V                               V

void DACC_Handler(void) {
  //  DACC_Handler Interrupt Service Routine. This routine
  //  provides a 'wrapper' to link the AVR DAC INTERRUPT
  //  to the 'XYscope-class' ISR routine.
  XYscope.dacHandler(); //Link the AVR DAC ISR/IRQ to the XYscope.
              //It is called whenever the DMA controller
              //'DAC_ready_for_More_data' event occurs
}

void paintCrt_ISR(void) {
  //  paintCrtISR  Interrupt Service Routine. This routine
  //  provides a 'wrapper' to link the Timer3.AttachedInterrupt()
  //  function to the 'XYscope-class' ISR routine.
  XYscope.initiateDacDma(); //Start the DMA transfer to paint the CRT screen
}

//  V                               V
//  |                               |
//  +---------- END Critical Interrupt Service Routines ------------+

//  Define/initialize critical global constants and variables

double dacClkRateHz, dacClkRateKHz;

int EndOfSetup_Ptr;
double TimeForRefresh;


char shiftVal = 0;
uint32_t dispTimer = 0;


//#define DEBUG                              // uncomment this for debugging output to serial (leave disabled to save memory if needed)

// ----------------------------------------------
// global variables
// ----------------------------------------------
static const float inchToMm = 25.4;
enum FitResolutionGate { kFill = 0, kOverscan };

static unsigned char i;
static int loops;
static double next_tick;


// ----------------------------------------------
// draw edges between projected nodes
// ----------------------------------------------
// XYscope.plotLine(x0, y0, x1, y1);


// ----------------------------------------------
// compute screen coordinates
// ----------------------------------------------
void computeScreenCoordinates(
    const float &filmApertureWidth,
    const float &filmApertureHeight,
    const uint32_t &imageWidth,
    const uint32_t &imageHeight,
    const FitResolutionGate &fitFilm,
    const float &nearClippingPLane,
    const float &focalLength,
    float &top, float &bottom, float &left, float &right
)
{
    float filmAspectRatio = filmApertureWidth / filmApertureHeight;
    float deviceAspectRatio = imageWidth / (float)imageHeight;

    top = ((filmApertureHeight * inchToMm / 2) / focalLength) * nearClippingPLane;
    right = ((filmApertureWidth * inchToMm / 2) / focalLength) * nearClippingPLane;

    // field of view (horizontal)
    float fov = 2 * 180 / M_PI * atan((filmApertureWidth * inchToMm / 2) / focalLength);
    std::cerr << "Field of view " << fov << std::endl;

    float xscale = 1;
    float yscale = 1;

    switch (fitFilm) {
        default:
        case kFill:
            if (filmAspectRatio > deviceAspectRatio) {
                xscale = deviceAspectRatio / filmAspectRatio;
            }
            else {
                yscale = filmAspectRatio / deviceAspectRatio;
            }
            break;
        case kOverscan:
            if (filmAspectRatio > deviceAspectRatio) {
                yscale = filmAspectRatio / deviceAspectRatio;
            }
            else {
                xscale = deviceAspectRatio / filmAspectRatio;
            }
            break;
    }

    right *= xscale;
    top *= yscale;

    bottom = -top;
    left = -right;
}


// ----------------------------------------------
// compute vertex raster screen coordinates
// ----------------------------------------------
void convertToRaster(
    const Vec3f &vertexWorld,
    const Matrix44f &worldToCamera,
    const float &l,
    const float &r,
    const float &t,
    const float &b,
    const float &near,
    const uint32_t &imageWidth,
    const uint32_t &imageHeight,
    Vec3f &vertexRaster
)
{
    Vec3f vertexCamera;

    worldToCamera.multVecMatrix(vertexWorld, vertexCamera);
   
    // convert to screen space
    Vec2f vertexScreen;
    vertexScreen.x = near * vertexCamera.x / -vertexCamera.z;
    vertexScreen.y = near * vertexCamera.y / -vertexCamera.z;
   
    // now convert point from screen space to NDC space (in range [-1,1])
    Vec2f vertexNDC;
    vertexNDC.x = 2 * vertexScreen.x / (r - l) - (r + l) / (r - l);
    vertexNDC.y = 2 * vertexScreen.y / (t - b) - (t + b) / (t - b);

    // convert to raster space
    vertexRaster.x = (vertexNDC.x + 1) / 2 * imageWidth;
    // in raster space y is down so invert direction
    vertexRaster.y = (1 - vertexNDC.y) / 2 * imageHeight;
    vertexRaster.z = -vertexCamera.z;
}


// ----------------------------------------------
// utility functions
// ----------------------------------------------
float min3(const float &a, const float &b, const float &c)
{ return std::min(a, std::min(b, c)); }

float max3(const float &a, const float &b, const float &c)
{ return std::max(a, std::max(b, c)); }

float edgeFunction(const Vec3f &a, const Vec3f &b, const Vec3f &c)
{ return (c[0] - a[0]) * (b[1] - a[1]) - (c[1] - a[1]) * (b[0] - a[0]); }


// ----------------------------------------------
// setup
// ----------------------------------------------
void setup() {
  #ifdef DEBUG
    Serial.begin(115200);
  #endif

  double DmaFreq=800000;    //800000 (Hz) is the default startup value for the DMA frequency.
                //You can try various alternate values to find an optimal
                //value that works best for your scope, setup, & application.
                //Use program menu option "c" & "C" to vary the frequency while
                //watching your scope display; Using "c" & "C" options will allow you
                //to see how frequency changes effect the display quality in real-time.

  XYscope.begin(DmaFreq);

  //Timer3 is used as the CRT refresh timer.  This timer is setup inside of XYscope.begin( ).
  //However, paintCRT_ISR must be "attached" to timer 3.  To be properly link to the
  //refresh-screen XYscope interupt service routine, it must be linked in the Arduino 
  //setup() code as follows:
  
  Timer3.attachInterrupt(paintCrt_ISR);
  XYscope.autoSetRefreshTime();
  
}

// ----------------------------------------------
// main loop
// ----------------------------------------------
void loop() {
  loops = 0;
  while( millis() > next_tick && loops < MAX_FRAMESKIP) {

    // ===============
    // input
    // ===============
    // push button on digital PIN 2 to change render type
//    if ( !(PIND & (1<<PD2)) && (millis()-last_btn) > 300 ) {
//      // clear screen
//      TFT.fillScreen(COLOR0);
//      // change draw type
//      draw_type++;
//      if (draw_type > NUMTYPES) draw_type = 0;
//      // print draw type on screen
//      draw_print(COLOR1);
//      // update last button push
//      last_btn = millis();
//    }

    // rotation
    m_world = mRotateX(mesh_rotation.x);
    m_world = mMultiply(mRotateY(mesh_rotation.y), m_world);
    m_world = mMultiply(mRotateZ(mesh_rotation.z), m_world);
    // scaling
    //m_world = mMultiply(mScale(1.6), m_world);

    // project nodes with world matrix
    Vector3i p;
    for (i=0; i<NODECOUNT; i++) {
      p.x = (m_world.m[0][0] * (NODE(i,0) >> PSHIFT)+
             m_world.m[1][0] * (NODE(i,1) >> PSHIFT) +
             m_world.m[2][0] * (NODE(i,2) >> PSHIFT) +
             m_world.m[3][0]) / PRES;
      
      p.y = (m_world.m[0][1] * (NODE(i,0) >> PSHIFT) +
             m_world.m[1][1] * (NODE(i,1) >> PSHIFT) +
             m_world.m[2][1] * (NODE(i,2) >> PSHIFT) +
             m_world.m[3][1]) / PRES;
            
      p.z = (m_world.m[0][2] * (NODE(i,0) >> PSHIFT) +
             m_world.m[1][2] * (NODE(i,1) >> PSHIFT) +
             m_world.m[2][2] * (NODE(i,2) >> PSHIFT) +
             m_world.m[3][2]) / PRES;

      // store projected node
      proj_nodes[i][0] = (FOV * p.x) / (FOV + p.z) + HALFW;
      proj_nodes[i][1] = (FOV * p.y) / (FOV + p.z) + HALFH;
    }

    #ifdef USE_ACCELEROMETER
      // if accelerometer is specified use it to rotate the mesh instead of the default rotation mode
      mesh_rotation.x = accel_get_value(&accel.index.x, accel.readings.x, &accel.total.x, ACCEL_XPIN);
      mesh_rotation.y = accel_get_value(&accel.index.y, accel.readings.y, &accel.total.y, ACCEL_YPIN);
      mesh_rotation.z = accel_get_value(&accel.index.z, accel.readings.z, &accel.total.z, ACCEL_ZPIN);
    #elif defined USE_JOYSTICK
      // if joystick is specified use it to rotate the mesh instead of the default rotation mode
      mesh_rotation.x = joystick_get_value(&joystick.index.x, joystick.readings.x, &joystick.total.x, JOYSTICK_XPIN);
      mesh_rotation.y = joystick_get_value(&joystick.index.y, joystick.readings.y, &joystick.total.y, JOYSTICK_YPIN);
    #else
      // default auto-rotation mode
      mesh_rotation.x+=3;
      mesh_rotation.y+=2;
      mesh_rotation.z++;
    #endif

    if (mesh_rotation.x > 360) mesh_rotation.x = 0;
    if (mesh_rotation.y > 360) mesh_rotation.y = 0;
    if (mesh_rotation.z > 360) mesh_rotation.z = 0;
    // ...
    next_tick += SKIP_TICKS;
    loops++;
  }    

  // ===============
  // draw
  // ===============
  // only redraw if nodes position have changed (less redraw - less flicker)
  //if (memcmp(old_nodes, proj_nodes, sizeof(proj_nodes))) {
    // render frame
//    switch(draw_type) {
//      case 0: draw_vertex(old_nodes, COLOR0);
//              draw_vertex(proj_nodes, COLOR1);
//              break;
//      case 1: if (TRICOUNT > 32) clear_dirty(old_nodes);
//              else draw_wireframe(old_nodes, COLOR0);
//              draw_wireframe(proj_nodes, COLOR1);
//              break;
//      case 2: clear_dirty(old_nodes);
//              draw_flat_color(proj_nodes, COLOR2);
//              break;
//      case 3: draw_flat_color(proj_nodes, COLOR2);
//              draw_wireframe(proj_nodes, COLOR0);
//              break;
//      case 4: draw_flat_color(proj_nodes, COLOR2);
//              break;
//    }
    XYscope.plotClear();
    XYscope.plotStart();
    draw_wireframe(proj_nodes);
    XYscope.plotEnd();
    // copy projected nodes to old_nodes to check if we need to redraw next frame
    //memcpy(old_nodes, proj_nodes, sizeof(proj_nodes));
  //}
}
