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
#include "types.h"

// ----------------------------------------------
// defines
// ---------------------------------------------
#define UPDATE_INTERVAL_MS 200
#define LUT(a) (long)(pgm_read_word(&lut[a]))

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

// ----------------------------------------------
// global variables
// ----------------------------------------------
static const float inchToMm = 25.4;

long updateTime = 0;

const unsigned int lut[] PROGMEM = {         // 0 to 90 degrees fixed point COSINE look up table
  16384, 16381, 16374, 16361, 16344, 16321, 16294, 16261, 16224, 16182, 16135, 16082, 16025, 15964, 15897, 15825, 15749, 15668, 15582, 15491, 15395, 15295, 15190, 15081, 14967, 14848, 14725, 14598, 14466, 14329, 14188, 14043, 13894, 13740, 13582, 13420, 13254, 13084, 12910, 12732, 12550, 12365, 12175, 11982, 11785, 11585, 11381, 11173, 10963, 10748, 10531, 10310, 10086, 9860, 9630, 9397, 9161, 8923, 8682, 8438, 8191, 7943, 7691, 7438, 7182, 6924, 6663, 6401, 6137, 5871, 5603, 5334, 5062, 4790, 4516, 4240, 3963, 3685, 3406, 3126, 2845, 2563, 2280, 1996, 1712, 1427, 1142, 857, 571, 285, 0
};

const uint32_t imageWidth = 10;
const uint32_t imageHeight = 10;
// | c00 c01 c02 c03 | -> x-axis
// | c10 c11 c12 c13 | -> y-axis
// | c20 c21 c22 c23 | -> z-axis
// | c30 c31 c32 c33 | -> translation
// represented by worldToCamera
const Matrix44f worldToCamera = {0.707107, -0.331295, 0.624695, 0, 0, 0.883452, 0.468521, 0, -0.707107, -0.331295, 0.624695, 0, -1.63871, -5.747777, -40.400412, 1};

const uint32_t ntris = 3156;
const float nearClippingPLane = 1;
const float farClippingPLane = 10;
float focalLength = 20; // in mm
// 35mm Full Aperture in inches
float filmApertureWidth = 0.980;
float filmApertureHeight = 0.735;

// ----------------------------------------------
// draw edges between projected nodes
// ----------------------------------------------
// XYscope.plotLine(x0, y0, x1, y1);

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
{ return min(a, min(b, c)); }

float max3(const float &a, const float &b, const float &c)
{ return max(a, max(b, c)); }

float edgeFunction(const Vec3f &a, const Vec3f &b, const Vec3f &c)
{ return (c[0] - a[0]) * (b[1] - a[1]) - (c[1] - a[1]) * (b[0] - a[0]); }

// ----------------------------------------------
// SIN/COS from 90 degrees LUT
// ----------------------------------------------
long SIN(unsigned int angle) {
  angle += 90;
  if (angle > 450) return LUT(0);
  if (angle > 360 && angle < 451) return -LUT(angle-360);
  if (angle > 270 && angle < 361) return -LUT(360-angle);
  if (angle > 180 && angle < 271) return  LUT(angle-180);
  return LUT(180-angle);
}

long COS(unsigned int angle) {
  if (angle > 360) return LUT(0);
  if (angle > 270 && angle < 361) return  LUT(360-angle);
  if (angle > 180 && angle < 271) return -LUT(angle-180);
  if (angle > 90  && angle < 181) return -LUT(180-angle);
  return LUT(angle);
}

// compute screen coordinates
void computeScreenCoordinates(
    const float &filmApertureWidth,
    const float &filmApertureHeight,
    const uint32_t &imageWidth,
    const uint32_t &imageHeight,
    const FitResolutionGate_t &fitFilm,
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
    

// define the frame-buffer and the depth-buffer. Initialize depth buffer
// to far clipping plane.
Vec3<unsigned char> *frameBuffer = new Vec3<unsigned char>[imageWidth * imageHeight];

float *depthBuffer = new float[imageWidth * imageHeight];


float xDeg = 0.0;
float yDeg = 0.0;
float zDeg = 0.0;

int firstTime = 1;


// ----------------------------------------------
// setup
// ----------------------------------------------
void setup() {
  #ifdef DEBUG
    Serial.begin(115200);
  #endif

  for (uint32_t i = 0; i < imageWidth * imageHeight; ++i) frameBuffer[i] = Vec3<unsigned char>(255);
  for (uint32_t i = 0; i < imageWidth * imageHeight; ++i) depthBuffer[i] = farClippingPLane;

  double DmaFreq=800000;    //800000 (Hz) is the default startup value for the DMA frequency.
                //You can try various alternate values to find an optimal
                //value that works best for your scope, setup, & application.
                //Use program menu option "c" & "C" to vary the frequency while
                //watching your scope display; USINg "c" & "C" options will allow you
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
    Matrix44f cameraToWorld = worldToCamera.inverse();

    // compute screen coordinates
    float t, b, l, r;
    
    computeScreenCoordinates(
        filmApertureWidth, filmApertureHeight,
        imageWidth, imageHeight,
        kOverscan,
        nearClippingPLane,
        focalLength,
        t, b, l, r);
    
    // define the frame-buffer and the depth-buffer. Initialize depth buffer
    // to far clipping plane.
    Vec3<unsigned char> *frameBuffer = new Vec3<unsigned char>[imageWidth * imageHeight];
    for (uint32_t i = 0; i < imageWidth * imageHeight; ++i) frameBuffer[i] = Vec3<unsigned char>(255);
    float *depthBuffer = new float[imageWidth * imageHeight];
    for (uint32_t i = 0; i < imageWidth * imageHeight; ++i) depthBuffer[i] = farClippingPLane;

    while (true) {
  
        // update the vector table ever so often
        // always do it the first time immediately
        if (((millis() - updateTime) > UPDATE_INTERVAL_MS) || (firstTime == 1)) {

          firstTime = 0;
          
          XYscope.plotClear();
          XYscope.plotStart();
      
          for (uint32_t i = 0; i < ntris; ++i) {
              // const Vec3f &v0 = vertices[nvertices[i * 3]];
              // const Vec3f &v1 = vertices[nvertices[i * 3 + 1]];
              // const Vec3f &v2 = vertices[nvertices[i * 3 + 2]];
              Vec3f &v0 = vertices[nvertices[i * 3]];
              Vec3f &v1 = vertices[nvertices[i * 3 + 1]];
              Vec3f &v2 = vertices[nvertices[i * 3 + 2]];
              
              float sx = SIN(xDeg);
              float sy = SIN(yDeg);
              float sz = SIN(zDeg);
              float cx = COS(xDeg);
              float cy = COS(yDeg);
              float cz = COS(zDeg);
              
              const Matrix44f mRotateX = {1, 0, 0, 0, 0, cx, -sx, 0, 0, sx, cx, 0, 0, 0, 0, 1};
              const Matrix44f mRotateY = {cy, 0, sy, 0, 0, 1, 0, 0, -sy, 0, cy, 0, 0, 0, 0, 1};
              const Matrix44f mRotateZ = {cz, -sz, 0, 0, sz, cz, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
              
              Vec3f vr0, vr1, vr2;
              mRotateY.multVecMatrix(v0, vr0);
              mRotateY.multVecMatrix(v1, vr1);
              mRotateY.multVecMatrix(v2, vr2);
              
              // [comment]
              // Convert the vertices of the triangle to raster space
              // [/comment]
              Vec3f v0Raster, v1Raster, v2Raster;
              convertToRaster(vr0, worldToCamera, l, r, t, b, nearClippingPLane, imageWidth, imageHeight, v0Raster);
              convertToRaster(vr1, worldToCamera, l, r, t, b, nearClippingPLane, imageWidth, imageHeight, v1Raster);
              convertToRaster(vr2, worldToCamera, l, r, t, b, nearClippingPLane, imageWidth, imageHeight, v2Raster);
              
              // [comment]
              // Precompute reciprocal of vertex z-coordinate
              // [/comment]
              v0Raster.z = 1 / v0Raster.z,
              v1Raster.z = 1 / v1Raster.z,
              v2Raster.z = 1 / v2Raster.z;
              
              
              // [comment]
              // Prepare vertex attributes. Divde them by their vertex z-coordinate
              // (though we use a multiplication here because v.z = 1 / v.z)
              // [/comment]
              Vec2f st0 = st[stindices[i * 3]];
              Vec2f st1 = st[stindices[i * 3 + 1]];
              Vec2f st2 = st[stindices[i * 3 + 2]];
      
              st0 *= v0Raster.z, st1 *= v1Raster.z, st2 *= v2Raster.z;
          
              float xmin = min3(v0Raster.x, v1Raster.x, v2Raster.x);
              float ymin = min3(v0Raster.y, v1Raster.y, v2Raster.y);
              float xmax = max3(v0Raster.x, v1Raster.x, v2Raster.x);
              float ymax = max3(v0Raster.y, v1Raster.y, v2Raster.y);
              
              // the triangle is out of screen
              if (xmin > imageWidth - 1 || xmax < 0 || ymin > imageHeight - 1 || ymax < 0) continue;
      
              // be careful xmin/xmax/ymin/ymax can be negative. Don't cast to uint32_t
              uint32_t x0 = max(int32_t(0), (int32_t)(floor(xmin)));
              uint32_t x1 = min(int32_t(imageWidth) - 1, (int32_t)(floor(xmax)));
              uint32_t y0 = max(int32_t(0), (int32_t)(floor(ymin)));
              uint32_t y1 = min(int32_t(imageHeight) - 1, (int32_t)(floor(ymax)));
      
              float area = edgeFunction(v0Raster, v1Raster, v2Raster);
              
              // [comment]
              // Inner loop
              // [/comment]
              for (uint32_t y = y0; y <= y1; ++y) {
                  for (uint32_t x = x0; x <= x1; ++x) {
                      Vec3f pixelSample(x + 0.5, y + 0.5, 0);
                      float w0 = edgeFunction(v1Raster, v2Raster, pixelSample);
                      float w1 = edgeFunction(v2Raster, v0Raster, pixelSample);
                      float w2 = edgeFunction(v0Raster, v1Raster, pixelSample);
                      if (w0 >= 0 && w1 >= 0 && w2 >= 0) {
                          w0 /= area;
                          w1 /= area;
                          w2 /= area;
                          float oneOverZ = v0Raster.z * w0 + v1Raster.z * w1 + v2Raster.z * w2;
                          float z = 1 / oneOverZ;
                          // [comment]
                          // Depth-buffer test
                          // [/comment]
                          if (z < depthBuffer[y * imageWidth + x]) {
                              depthBuffer[y * imageWidth + x] = z;
                              
                              Vec2f st = st0 * w0 + st1 * w1 + st2 * w2;
                              
                              st *= z;
                              
                              // [comment]
                              // If you need to compute the actual position of the shaded
                              // point in camera space. Proceed like with the other vertex attribute.
                              // Divide the point coordinates by the vertex z-coordinate then
                              // interpolate uSINg barycentric coordinates and finally multiply
                              // by sample depth.
                              // [/comment]
                              Vec3f v0Cam, v1Cam, v2Cam;
                              worldToCamera.multVecMatrix(v0, v0Cam);
                              worldToCamera.multVecMatrix(v1, v1Cam);
                              worldToCamera.multVecMatrix(v2, v2Cam);
                              
                              float px = (v0Cam.x/-v0Cam.z) * w0 + (v1Cam.x/-v1Cam.z) * w1 + (v2Cam.x/-v2Cam.z) * w2;
                              float py = (v0Cam.y/-v0Cam.z) * w0 + (v1Cam.y/-v1Cam.z) * w1 + (v2Cam.y/-v2Cam.z) * w2;
                              
                              Vec3f pt(px * z, py * z, -z); // pt is in camera space
                              
                              // [comment]
                              // Compute the face normal which is used for a simple facing ratio.
                              // Keep in mind that we are doing all calculation in camera space.
                              // Thus the view direction can be computed as the point on the object
                              // in camera space minus Vec3f(0), the position of the camera in camera
                              // space.
                              // [/comment]
                              Vec3f n = (v1Cam - v0Cam).crossProduct(v2Cam - v0Cam);
                              n.normalize();
                              Vec3f viewDirection = -pt;
                              viewDirection.normalize();
                              
                              float nDotView =  max(0.f, n.dotProduct(viewDirection));
                              
                              int val = 255;
                             
      
                              // draw each triangle if the face normal vector is facing the towards the camera` 
                              if (nDotView >= 0) {
                                  XYscope.plotLine(v0Raster.x, v0Raster.y, v1Raster.x, v1Raster.y);
                                  XYscope.plotLine(v1Raster.x, v1Raster.y, v2Raster.x, v2Raster.y);
                                  XYscope.plotLine(v2Raster.x, v2Raster.y, v0Raster.x, v0Raster.y);
                              }
                          }
                      }
                  }
              }
          }
      
          // spin around the y axis
          yDeg += 1;
          if (yDeg > 360) yDeg = 0;
          
          XYscope.plotEnd();
          delete [] frameBuffer;
          delete [] depthBuffer;
          updateTime = millis();

        }
    }
}
