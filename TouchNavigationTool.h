/***********************************************************************
TouchNavigationTool - Class encapsulating the navigation behaviour of a
touch interface. It is based on Vrui's MouseNavigationTool.
Copyright (c) 2016-2023 Thoams Wischgoll

***********************************************************************/

#ifndef VRUI_TOUCHNAVIGATIONTOOL_INCLUDED
#define VRUI_TOUCHNAVIGATIONTOOL_INCLUDED

#include <Geometry/Point.h>
#include <Geometry/Vector.h>
#include <Geometry/OrthogonalTransformation.h>
#include <Vrui/GUIInteractor.h>
#include <Vrui/NavigationTool.h>

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#define DEVICE "/dev/hidraw0"

/* Forward declarations: */
class GLContextData;
namespace Vrui {
  class InputDeviceAdapterMouse;
}

namespace Vrui {

  class TouchNavigationTool;

  class TouchNavigationToolFactory:public ToolFactory
  {
    friend class TouchNavigationTool;
	
    /* Elements: */
  private:
    Scalar rotatePlaneOffset; // Offset of rotation plane from screen plane
    Scalar rotateFactor; // Distance the device has to be moved to rotate by one radians
    bool invertDolly; // Flag whether to invert the switch between dollying/zooming
    Vector screenDollyingDirection; // Direction of dollying vector in screen's coordinates
    Vector screenScalingDirection; // Direction of scaling vector in screen's coordinates
    Scalar dollyFactor; // Distance the device has to be moved along the scaling line to dolly by one physical unit
    Scalar scaleFactor; // Distance the device has to be moved along the scaling line to scale by factor of e
    Scalar wheelDollyFactor; // Physical unit dolly amount for one wheel click
    Scalar wheelScaleFactor; // Scaling factor for one wheel click
    Scalar spinThreshold; // Distance the device has to be moved on the last step of rotation to activate spinning
    bool showScreenCenter; // Flag whether to draw the center of the screen during navigation
    bool interactWithWidgets; // Flag if the mouse navigation tool doubles as a widget tool (this is an evil hack)
	
    /* Constructors and destructors: */
  public:
    TouchNavigationToolFactory(ToolManager& toolManager);
    virtual ~TouchNavigationToolFactory(void);
	
    /* Methods from ToolFactory: */
    virtual const char* getName(void) const;
    virtual Tool* createTool(const ToolInputAssignment& inputAssignment) const;
    virtual void destroyTool(Tool* tool) const;
  };

  class TouchNavigationTool:public NavigationTool,public GUIInteractor
  {
    friend class TouchNavigationToolFactory;
	
    /* Embedded classes: */
  private:
    enum NavigationMode // Enumerated type for states the tool can be in
      {
	IDLE,WIDGETING,ROTATING,SPINNING,PANNING,DOLLYING,SCALING,DOLLYING_WHEEL,SCALING_WHEEL
      };
	
    /* Elements: */
    static TouchNavigationToolFactory* factory; // Pointer to the factory object for this class
    InputDeviceAdapterMouse* mouseAdapter; // Pointer to the mouse input device adapter owning the input device associated with this tool

    static short fingers[20], fingercount;

    /* Transient navigation state: */
    Point currentPos, currentPos2; // Current projected position of mouse input device on screen
    double lastMoveTime; // Application time at which the projected position last changed
    Scalar currentValue; // Value of the associated valuator
    bool dolly; // Flag whether to dolly instead of scale
    NavigationMode navigationMode; // The tool's current navigation mode
    Point screenCenter; // Center of screen; center of rotation and scaling operations
    Vector dollyDirection; // Transformation direction of dollying (vector from eye to screen center)
    Point motionStart; // Start position of mouse motion
    Vector rotateOffset; // Offset vector applied to device position during rotations
    Point lastRotationPos; // Last mouse position during rotation
    Vector spinAngularVelocity; // Angular velocity when spinning
    Scalar currentWheelScale; // Current scale factor during mouse wheel scaling
    NavTrackerState preScale; // Transformation to be applied to the navigation transformation before scaling
    NavTrackerState rotation; // Current accumulated rotation transformation
    NavTrackerState postScale; // Transformation to be applied to the navigation transformation after scaling
	
    /* Private methods: */
    Point calcScreenCenter(void) const; // Calculates the center of the screen containing the input device
    Point calcScreenPos(void) const; // Calculates the screen position of the input device
    void startRotating(void); // Sets up rotation
    void startPanning(void); // Sets up panning
    void startDollying(void); // Sets up dollying
    void startScaling(void); // Sets up scaling

    ONTransform getScreenTransform(Scalar viewport[4]) const;
	
    /* Constructors and destructors: */
  public:
    TouchNavigationTool(const ToolFactory* factory,const ToolInputAssignment& inputAssignment);

    static void swapshort (short *s) {
      static char tmp;
      static char *c;

      c = (char*)s;
      tmp = c[0];
      c[0] = c[1];
      c[1] = tmp;
    }
	
    /* Methods from Tool: */
    virtual const ToolFactory* getFactory(void) const;
    virtual void frame(void);
    virtual void display(GLContextData& contextData) const;

    static void createTouchTool ();


    static void* backgroundthread (void *);
    pthread_t background;

    static bool multitouch; /* multitouch support */
  };

}

#endif
