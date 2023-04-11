/***********************************************************************
TouchNavigationTool - Class encapsulating the navigation behaviour of a
mouse in the OpenInventor SoXtExaminerViewer.
Copyright (c) 2004-2011 Oliver Kreylos

This file is part of the Virtual Reality User Interface Library (Vrui).

The Virtual Reality User Interface Library is free software; you can
redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version.

The Virtual Reality User Interface Library is distributed in the hope
that it will be useful, but WITHOUT ANY WARRANTY; without even the
implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the GNU General Public License for more details.
96352.1
You should have received a copy of the GNU General Public License along
with the Virtual Reality User Interface Library; if not, write to the
Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
02111-1307 USA
***********************************************************************/

#include "TouchNavigationTool.h"

#include <Misc/StandardValueCoders.h>
#include <Misc/ConfigurationFile.h>
#include <Math/Math.h>
#include <Geometry/GeometryValueCoders.h>
#include <GL/gl.h>
#include <GL/GLColorTemplates.h>
#include <GL/GLGeometryWrappers.h>
#include <GL/GLTransformationWrappers.h>
#include <Vrui/Vrui.h>
#include <Vrui/InputGraphManager.h>
#include <Vrui/InputDeviceManager.h>
#include <Vrui/Viewer.h>
#include <Vrui/VRWindow.h>
#include <Vrui/ToolManager.h>
#include <Vrui/VRScreen.h>

#include <linux/input.h>

namespace Vrui {

  /*******************************************
Methods of class TouchNavigationToolFactory:
  *******************************************/

  TouchNavigationToolFactory::TouchNavigationToolFactory(ToolManager& toolManager)
    :ToolFactory("TouchNavigationTool",toolManager),
     rotatePlaneOffset(getInchFactor()*Scalar(3)),
     rotateFactor(getInchFactor()*Scalar(3)),
     invertDolly(false),
     screenDollyingDirection(0,-1,0),
     screenScalingDirection(0,-1,0),
     dollyFactor(Scalar(1)),
     scaleFactor(getInchFactor()*Scalar(3)),
     wheelDollyFactor(getInchFactor()*Scalar(-12)),
     wheelScaleFactor(Scalar(0.5)),
     spinThreshold(getUiSize()*Scalar(1)),
     showScreenCenter(true),
     interactWithWidgets(true)
  {
    /* Initialize tool layout: */
    layout.setNumButtons(1);
    layout.setNumValuators(0);
	
    /* Insert class into class hierarchy: */
    ToolFactory* navigationToolFactory=toolManager.loadClass("NavigationTool");
    navigationToolFactory->addChildClass(this);
    addParentClass(navigationToolFactory);
	
    /* Load class settings: */
    Misc::ConfigurationFileSection cfs=toolManager.getToolClassSection(getClassName());
    rotatePlaneOffset=cfs.retrieveValue<Scalar>("./rotatePlaneOffset",rotatePlaneOffset);
    rotateFactor=cfs.retrieveValue<Scalar>("./rotateFactor",rotateFactor);
    invertDolly=cfs.retrieveValue<bool>("./invertDolly",invertDolly);
    screenDollyingDirection=cfs.retrieveValue<Vector>("./screenDollyingDirection",screenDollyingDirection);
    screenScalingDirection=cfs.retrieveValue<Vector>("./screenScalingDirection",screenScalingDirection);
    dollyFactor=cfs.retrieveValue<Scalar>("./dollyFactor",dollyFactor);
    scaleFactor=cfs.retrieveValue<Scalar>("./scaleFactor",scaleFactor);
    wheelDollyFactor=cfs.retrieveValue<Scalar>("./wheelDollyFactor",wheelDollyFactor);
    wheelScaleFactor=cfs.retrieveValue<Scalar>("./wheelScaleFactor",wheelScaleFactor);
    spinThreshold=cfs.retrieveValue<Scalar>("./spinThreshold",spinThreshold);
    showScreenCenter=cfs.retrieveValue<bool>("./showScreenCenter",showScreenCenter);
    interactWithWidgets=cfs.retrieveValue<bool>("./interactWithWidgets",interactWithWidgets);
	
    /* Set tool class' factory pointer: */
    TouchNavigationTool::factory=this;
  }

  TouchNavigationToolFactory::~TouchNavigationToolFactory(void)
  {
    /* Reset tool class' factory pointer: */
    TouchNavigationTool::factory=0;
  }

  const char* TouchNavigationToolFactory::getName(void) const
  {
    return "Touch Navigation";
  }

  Tool* TouchNavigationToolFactory::createTool(const ToolInputAssignment& inputAssignment) const
  {
    return new TouchNavigationTool(this,inputAssignment);
  }

  void TouchNavigationToolFactory::destroyTool(Tool* tool) const
  {
    delete tool;
  }

  extern "C" void resolveTouchNavigationToolDependencies(Plugins::FactoryManager<ToolFactory>& manager)
  {
    /* Load base classes: */
    manager.loadClass("NavigationTool");
  }

  extern "C" ToolFactory* createTouchNavigationToolFactory(Plugins::FactoryManager<ToolFactory>& manager)
  {
    /* Get pointer to tool manager: */
    ToolManager* toolManager=static_cast<ToolManager*>(&manager);
	
    /* Create factory object and insert it into class hierarchy: */
    TouchNavigationToolFactory* mouseNavigationToolFactory=new TouchNavigationToolFactory(*toolManager);
	
    /* Return factory object: */
    return mouseNavigationToolFactory;
  }

  extern "C" void destroyTouchNavigationToolFactory(ToolFactory* factory)
  {
    delete factory;
  }

  /********************************************
Static elements of class TouchNavigationTool:
  ********************************************/

  TouchNavigationToolFactory* TouchNavigationTool::factory=0;

  /************************************
Methods of class TouchNavigationTool:
  ************************************/

  Point TouchNavigationTool::calcScreenCenter(void) const
  {
    /* Get the transformation of the screen currently containing the input device: */
    Scalar viewport[4];
    ONTransform screenT=getScreenTransform(viewport);
	
    /* Calculate the screen's center: */
    Point center;
    center[0]=Math::mid(viewport[0],viewport[1]);
    center[1]=Math::mid(viewport[2],viewport[3]);
    center[2]=Scalar(0);
	
    /* Transform the center position to physical coordinates: */
    return screenT.transform(center);
  }

  ONTransform TouchNavigationTool::getScreenTransform(Scalar viewport[4]) const
  {
    /* Check if the mouse adapter is valid: */
    VRScreen* screen=0;

    /* Use the main screen: */
    //screen=getMainScreen();
    screen = getScreen (getNumScreens () - 1);
    screen->getViewport(viewport);

    /* Return the screen's transformation: */
    return screen->getScreenTransformation();
  }

  Point TouchNavigationTool::calcScreenPos(void) const
  {
    /* Calculate the ray equation: */
    Ray ray1=getButtonDeviceRay(0);
    //printf ("mouse pos: (%f,%f)\n", ray1.getOrigin ()[0], ray1.getOrigin()[2]);
    //printf ("mouse dirs: (%f,%f,%f)\n", ray1.getDirection ()[0], ray1.getDirection ()[1], ray1.getDirection()[2]);

    /* Get the transformation of the screen currently containing the input device: */
    Scalar viewport[4];
    ONTransform screenT=getScreenTransform(viewport);

    VRWindow *window = Vrui::getWindow (0);
    Scalar winviewport[4];
    window->getScreenViewport (winviewport);
    //printf ("WinViewport: %f,%f,%f,%f\n", winviewport[0], winviewport[1], winviewport[2], winviewport[3]);

    const int *origin = window->getWindowOrigin ();
    const int *size = window->getViewportSize ();
    
    Geometry::Point<double, 3> touchpoint =
     Geometry::Point<double, 3> (viewport[3] * (0.492 * (float)(fingers[0]) - 160.0 - origin[0] - size[0]/2) / size[0],
				 viewport[4] * (0.274 * (float)(fingers[1]) - 12.0 - origin[1] - size[1]/2) / size[1],
				  0.0);
    double dist = sqrt (touchpoint[0]*touchpoint[0] +
		     touchpoint[1]*touchpoint[1]);

    if (dist > 0.9 * viewport[4]) {
      touchpoint = screenT.transform (touchpoint);
      return touchpoint;
    }

    touchpoint[2] = sqrt (0.81 * viewport[4] * viewport[4] - dist * dist);
    touchpoint = screenT.transform (touchpoint);
    return touchpoint;

    Ray ray = Ray(touchpoint,
		  screenT.getDirection(2));

    /* Intersect the device ray with the screen: */
    Vector normal=screenT.getDirection(2);
    Scalar d=normal*screenT.getOrigin();
    Scalar divisor=normal*ray.getDirection();
    if(divisor==Scalar(0))
      //return Point::origin;
      return touchpoint;
	
    Scalar lambda=(d-ray.getOrigin()*normal)/divisor;
    if(lambda<Scalar(0))
      //return Point::origin;
      return touchpoint;
	
    return ray(lambda);
  }

  void TouchNavigationTool::startRotating(void)
  {
    /* Calculate the rotation center: */
    screenCenter=calcScreenCenter();
	
    /* Calculate initial rotation position: */
    //lastRotationPos=calcScreenPos();
	
    /* Get the transformation of the screen currently containing the input device: */
    Scalar viewport[4];
    ONTransform screenT=getScreenTransform(viewport);
	
    /* Calculate the rotation offset vector: */
    rotateOffset=screenT.transform(Vector(0,0,factory->rotatePlaneOffset));
	
    preScale=NavTrackerState::translateFromOriginTo(screenCenter);
    rotation=NavTrackerState::identity;
    postScale=NavTrackerState::translateToOriginFrom(screenCenter);
    postScale*=getNavigationTransformation();
	
    /* Go to rotating mode: */
    navigationMode=ROTATING;
  }

  void TouchNavigationTool::startPanning(void)
  {
    /* Calculate initial motion position: */
    //motionStart=calcScreenPos();
	
    preScale=getNavigationTransformation();
	
    /* Go to panning mode: */
    navigationMode=PANNING;
  }

  void TouchNavigationTool::startDollying(void)
  {
    /* Calculate the dollying direction: */
    dollyDirection=getMainViewer()->getHeadPosition()-calcScreenCenter();
    dollyDirection.normalize();
	
    /* Calculate initial motion position: */
    //motionStart=calcScreenPos();
	
    preScale=getNavigationTransformation();
	
    /* Go to dollying mode: */
    navigationMode=DOLLYING;
  }

  void TouchNavigationTool::startScaling(void)
  {
    /* Calculate the scaling center: */
    screenCenter=calcScreenCenter();
	
    /* Calculate initial motion position: */
    //motionStart=calcScreenPos();

#if 0
    preScale=NavTrackerState::translateFromOriginTo(screenCenter);
    postScale=NavTrackerState::translateToOriginFrom(screenCenter);
    postScale*=getNavigationTransformation();
#else
    postScale=getNavigationTransformation();
#endif

    /* Go to scaling mode: */
    navigationMode=SCALING;
  }

  TouchNavigationTool::TouchNavigationTool(const ToolFactory* factory,const ToolInputAssignment& inputAssignment)
    :NavigationTool(factory,inputAssignment),
     GUIInteractor(false,Scalar(0),getButtonDevice(0)),
     mouseAdapter(0),
     currentPos(Point::origin),currentValue(0),
     dolly(TouchNavigationTool::factory->invertDolly),navigationMode(IDLE)
  {
    pthread_create (&background, 0, backgroundthread, this);
  }

  const ToolFactory* TouchNavigationTool::getFactory(void) const
  {
    return factory;
  }

#if 0
  void buttonCallback(int buttonSlotIndex,InputDevice::ButtonCallbackData* cbData)
  {
    /* Process based on which button was pressed: */
    switch(buttonSlotIndex)
      {
      case 0:
	if(cbData->newButtonState) // Button has just been pressed
	  {
	    /* Act depending on this tool's current state: */
	    switch(navigationMode)
	      {
	      case IDLE:
	      case SPINNING:
		if(factory->interactWithWidgets)
		  {
		    /* Check if the GUI interactor accepts the event: */
		    GUIInteractor::updateRay();
		    if(GUIInteractor::buttonDown(false))
		      {
			/* Deactivate this tool if it is spinning: */
			if(navigationMode==SPINNING)
			  deactivate();
								
			/* Go to widget interaction mode: */
			navigationMode=WIDGETING;
		      }
		    else
		      {
			/* Try activating this tool: */
			if(navigationMode==SPINNING||activate())
			  startRotating();
		      }
		  }
		else
		  {
		    /* Try activating this tool: */
		    if(navigationMode==SPINNING||activate())
		      startRotating();
		  }
		break;
					
	      case PANNING:
		if(dolly)
		  startDollying();
		else
		  startScaling();
		break;
					
	      default:
		/* This shouldn't happen; just ignore the event */
		break;
	      }
	  }
	else // Button has just been released
	  {
	    /* Act depending on this tool's current state: */
	    switch(navigationMode)
	      {
	      case WIDGETING:
		{
		  if(GUIInteractor::isActive())
		    {
		      /* Deliver the event: */
		      GUIInteractor::buttonUp();
		    }
						
		  /* Deactivate this tool: */
		  navigationMode=IDLE;
		  break;
		}
					
	      case ROTATING:
		{
		  /* Check if the input device is still moving: */
		  Point currentPos=calcScreenPos();
		  Vector delta=currentPos-lastRotationPos;
		  if(Geometry::mag(delta)>factory->spinThreshold)
		    {
		      /* Calculate spinning angular velocity: */
		      Vector offset=(lastRotationPos-screenCenter)+rotateOffset;
		      Vector axis=Geometry::cross(offset,delta);
		      Scalar angularVelocity=Geometry::mag(delta)/(factory->rotateFactor*(getApplicationTime()-lastMoveTime));
		      spinAngularVelocity=axis*(Scalar(0.5)*angularVelocity/axis.mag());
							
		      /* Go to spinning mode: */
		      navigationMode=SPINNING;
		    }
		  else
		    {
		      /* Deactivate this tool: */
		      deactivate();
							
		      /* Go to idle mode: */
		      navigationMode=IDLE;
		    }
		  break;
		}
					
	      case DOLLYING:
	      case SCALING:
		startPanning();
		break;
					
	      default:
		/* This shouldn't happen; just ignore the event */
		break;
	      }
	  }
	break;
		
      case 1:
	if(cbData->newButtonState) // Button has just been pressed
	  {
	    /* Act depending on this tool's current state: */
	    switch(navigationMode)
	      {
	      case IDLE:
	      case SPINNING:
		/* Try activating this tool: */
		if(navigationMode==SPINNING||activate())
		  startPanning();
		break;
					
	      case ROTATING:
		if(dolly)
		  startDollying();
		else
		  startScaling();
		break;
					
	      default:
		/* This shouldn't happen; just ignore the event */
		break;
	      }
	  }
	else // Button has just been released
	  {
	    /* Act depending on this tool's current state: */
	    switch(navigationMode)
	      {
	      case PANNING:
		/* Deactivate this tool: */
		deactivate();
						
		/* Go to idle mode: */
		navigationMode=IDLE;
		break;
					
	      case DOLLYING:
	      case SCALING:
		startRotating();
		break;
					
	      default:
		/* This shouldn't happen; just ignore the event */
		break;
	      }
	  }
	break;
		
      case 2:
	/* Set the dolly flag: */
	dolly=cbData->newButtonState;
	if(factory->invertDolly)
	  dolly=!dolly;
	if(dolly) // Dollying has just been enabled
	  {
	    /* Act depending on this tool's current state: */
	    switch(navigationMode)
	      {
	      case SCALING:
		startDollying();
		break;
					
	      default:
		/* Nothing to do */
		break;
	      }
	  }
	else
	  {
	    /* Act depending on this tool's current state: */
	    switch(navigationMode)
	      {
	      case DOLLYING:
		startScaling();
		break;
					
	      default:
		/* Nothing to do */
		break;
	      }
	  }
	break;
      }
  }

  void valuatorCallback(int,InputDevice::ValuatorCallbackData* cbData)
  {
    currentValue=Scalar(cbData->newValuatorValue);
    if(currentValue!=Scalar(0))
      {
	/* Act depending on this tool's current state: */
	switch(navigationMode)
	  {
	  case IDLE:
	  case SPINNING:
	    /* Try activating this tool: */
	    if(navigationMode==SPINNING||activate())
	      {
		if(dolly)
		  {
		    /* Calculate the dollying direction: */
		    if(mouseAdapter!=0)
		      dollyDirection=mouseAdapter->getWindow()->getViewer()->getHeadPosition()-calcScreenCenter();
		    else
		      dollyDirection=getMainViewer()->getHeadPosition()-calcScreenCenter();
		    dollyDirection.normalize();
						
		    /* Initialize the wheel dollying factor: */
		    currentWheelScale=Scalar(1);
						
		    preScale=getNavigationTransformation();
						
		    /* Go to wheel dollying mode: */
		    navigationMode=DOLLYING_WHEEL;
		  }
		else
		  {
		    /* Calculate the scaling center: */
		    screenCenter=calcScreenCenter();
						
		    /* Initialize the wheel scaling factor: */
		    currentWheelScale=Scalar(1);
						
		    preScale=NavTrackerState::translateFromOriginTo(screenCenter);
		    postScale=NavTrackerState::translateToOriginFrom(screenCenter);
		    postScale*=getNavigationTransformation();
						
		    /* Go to wheel scaling mode: */
		    navigationMode=SCALING_WHEEL;
		  }
	      }
	    break;
			
	  default:
	    /* This can definitely happen; just ignore the event */
	    break;
	  }
      }
    else
      {
	/* Act depending on this tool's current state: */
	switch(navigationMode)
	  {
	  case DOLLYING_WHEEL:
	  case SCALING_WHEEL:
	    /* Deactivate this tool: */
	    deactivate();
				
	    /* Go to idle mode: */
	    navigationMode=IDLE;
	    break;
			
	  default:
	    /* This can definitely happen; just ignore the event */
	    break;
	  }
      }
  }
#endif

  void TouchNavigationTool::frame(void)
  {
    if (fingercount == 0) {
      navigationMode=IDLE;
      return;
    }

    switch (fingercount) {
    case 1:
      if (navigationMode != ROTATING) {
	startRotating ();
	currentPos = Point (fingers[0], 0.0, -fingers[1]);
      }
      break;
    case 2:
      if (navigationMode != SCALING) {
	startScaling ();
	currentPos = Point (fingers[0], 0.0, -fingers[1]);
	if (multitouch)
	  currentPos2 = Point (fingers[2], 0.0, -fingers[3]);
	else
	  currentPos2 = Point (0.0, 0.0001, 0.0);
      }
      break;
    case 3:
      if (navigationMode != PANNING) {
	startPanning ();
	currentPos = Point (fingers[0], 0.0, -fingers[1]);
      }
      break;
    default:
      navigationMode=IDLE;
      break;
    }

    /* Update the current mouse position: */
    //Point newCurrentPos=calcScreenPos();
    Point newCurrentPos = Point (fingers[0], 0.0, -fingers[1]);
    Point newCurrentPos2;
    if (multitouch)
      newCurrentPos2 = Point (fingers[2], 0.0, -fingers[3]);
    else
      newCurrentPos2 = currentPos2;

    /* Act depending on this tool's current state: */
    switch(navigationMode)
      {
      case ROTATING:
	{
#if 0
	  printf ("angles %f %f\n",
		  0.1 * (newCurrentPos[0] - currentPos[0]),
		  0.1 * (newCurrentPos[2] - currentPos[2]));
#endif
	  if(fabs (newCurrentPos[0] - currentPos[0]) > 0.0001 ||
	     fabs (newCurrentPos[2] - currentPos[2]) > 0.0001) {
	    rotation.leftMultiply(NavTrackerState::rotate(NavTrackerState::Rotation::rotateAxis(Vector (0.0, 0.0, 1.0), 0.0007 * (newCurrentPos[0] - currentPos[0]))));
	    rotation.leftMultiply(NavTrackerState::rotate(NavTrackerState::Rotation::rotateAxis(Vector (1.0, 0.0, 0.0), -0.0007 * (newCurrentPos[2] - currentPos[2]))));

	    NavTrackerState t=preScale;
	    t*=rotation;
	    t*=postScale;
	    setNavigationTransformation(t);

	    currentPos = newCurrentPos;
	  }

	  break;
	}
		
      case PANNING:
	{
	  /* Update the navigation transformation: */
	  NavTrackerState t=NavTrackerState::translate(0.003 * (newCurrentPos - currentPos));
	  t*=preScale;
	  setNavigationTransformation(t);
	  break;
	}
		
      case SCALING:
	{
	  /* Calculate the current scaling direction: */
	  Scalar viewport[4];
	  ONTransform screenT=getScreenTransform(viewport);

	  /* Update the navigation transformation: */
	  Scalar scale=(Geometry::dist (newCurrentPos, newCurrentPos2) / Geometry::dist (currentPos, currentPos2));
	  //printf ("Scale: %f\n", scale);

	  NavTrackerState t = NavTrackerState::identity;

	  /* We may have to rotate around the view direction also */
	  Vector dir1, dir2;
	  dir1 = Vector (currentPos[0] - currentPos2[0],
			 currentPos[2] - currentPos2[2]);
	  dir1.normalize ();
	  dir2 = Vector (newCurrentPos[0] - newCurrentPos2[0],
			 newCurrentPos[2] - newCurrentPos2[2]);
	  dir2.normalize ();
	  double angle;
	  if (dir1[1]*dir2[0] -
	      dir1[0]*dir2[1] > 0.0)
	    angle = acos ((dir1[0] * dir2[0] + dir1[1] * dir2[1]));
	  else
	    angle = -acos ((dir1[0] * dir2[0] + dir1[1] * dir2[1]));

#if 0
	  printf ("Angle: %lf %lf %f %f %f %f %lf\n", angle, dir1[0] * dir2[0] + dir1[1] * dir2[1], currentPos[0], currentPos[2], newCurrentPos[0], newCurrentPos[2], dir1[1]*dir2[0] - dir1[0]*dir2[1]);
#endif
	  /* sometimes angle blows us and gives NaN, so just rule that
	     out */
	  if (fabs (angle) < 100.0) {
	    t *= NavTrackerState::translateFromOriginTo(screenCenter);
	    t *= NavTrackerState::rotate(NavTrackerState::Rotation::rotateAxis(Vector (0.0, 1.0, 0.0), angle));
	    t *= NavTrackerState::translateToOriginFrom(screenCenter);
	  }

#if 0
	  NavTrackerState t=preScale;
	  t*=NavTrackerState::scale(scale);
	  t*=postScale;
#else
	  t*=postScale;
	  t *= NavTrackerState::scale(scale);
#endif

	  setNavigationTransformation(t);
	  break;
	}
		
      default:
	;
      }
  }

  void* TouchNavigationTool::backgroundthread (void* pointer)
  {
#if 1
    char buffer[64], *p;
    int i;
    input_event event;
    int fd;
    short currentfinger;
    int capabilities;
    char device[32];

    fd = open ("/dev/touch", O_RDONLY);
    if (fd < 0)
      return 0;

    ioctl (fd, EVIOCGBIT(0, 4), &capabilities);
    ioctl (fd, EVIOCGNAME (31), device);

    if (strncmp (device, "eGalax", 6) == 0)
      multitouch = false;
    else
      multitouch = true;

    if (multitouch) {
      for (i=0; i<20; i++)
	fingers[i] = -1;

      currentfinger = 0;
      while (1) {
	do {
	  read (fd, &event, sizeof (event));
	  //printf ("type: %d code: %d value: %d\n", event.type, event.code, event.value);
	  if (event.type == EV_ABS) {
	    switch (event.code) {
	    case ABS_MT_POSITION_X:
	      fingers[2 * currentfinger] = event.value;
	      break;
	    case ABS_MT_POSITION_Y:
	      fingers[2 * currentfinger + 1] = event.value;
	      break;
	    case 47:
	      currentfinger = event.value;
	      break;
	    case ABS_MT_TRACKING_ID:
	      fingers[2 * currentfinger] = event.value;
	      break;
	    case ABS_MT_PRESSURE:
	      // pressure on contact area
	      break;
	    case 0:
	      break;
	    case 1:
	      break;
	    default:
	      printf ("TouchNavigation: unexpected code %x for absolute position\n", event.code);
	    }
	  }
	  else if (event.type == EV_KEY && event.code == BTN_TOUCH) {
	    // finger released
	  }

	  // find out how many fingers touched
	  fingercount = 0;
	  for (i=0; i<20; i+=2) {
	    if (fingers[i] >=0)
	      fingercount++;
	  }
	} while (event.type != EV_SYN);

#if 0
	/* print out results */
	printf ("fingers: %d", fingercount);
	short *fp = fingers;
	for (i=0; i<fingercount; i++) {
	  printf (" (%d,%d)", *fp++, *fp++);
	}
	printf ("\n");
#endif

	Vrui::requestUpdate ();
      }
    }
    else {
      int buttoncount = 0;
      short oldfingers[2];
      int oldfingercount = 0;

      oldfingers[0] = oldfingers[1] = 0;

      while (1) {
	read (fd, &event, sizeof (event));
	if (event.type == EV_ABS) {
#if 0
	  fingercount = buttoncount + 1;
#endif
	  switch (event.code) {
	  case ABS_X:
	    fingers[0] = event.value;
	    break;
	  case ABS_Y:
	    fingers[1] = -event.value;
	    break;
	  default:
	    printf ("TouchNavigation: unexpected code %x for absolute position\n", event.code);
	  }
	}
	else if (event.type == EV_KEY && event.code == BTN_TOUCH) {
#if 0
	  if (event.value == 0) {
	    // finger released
	    if (pow (oldfingers[0] - fingers[0], 2) +
		pow (oldfingers[1] - fingers[1], 2) > 25000) {
	      buttoncount = 0;
	      fingercount = 0;
	    }
	    else
	      buttoncount++;
	    oldfingers[0] = fingers[0];
	    oldfingers[1] = fingers[1];
	  }
#else
	  if (event.value == 1) {
	    // finger released
	    if (fingers[0] > 1800) {
	      if (fingers[1] > -700)
		fingercount = 1;
	      else if (fingers[1] > -1400)
		fingercount = 2;
	      else
		fingercount = 3;
	    }
	    else
	      fingercount = oldfingercount;
	  }
	  else {
	    ((Vrui::TouchNavigationTool*)pointer)->navigationMode=IDLE;
	    oldfingercount = fingercount;
	    fingercount = 0;
	  }
#endif
	}
	else if (event.type == EV_SYN) {
	}

	//printf ("type: %x code: %x value: %d position: (%d,%d) fingercount: %d\n", event.type, event.code, event.value, fingers[0], fingers[1], buttoncount);

	Vrui::requestUpdate ();
      }
    }

    close (fd);
#else
    short *fp;
    char buffer[64], *p;
    int i;
    size_t count;

    /* open device for touch screen */
    int fd = open (DEVICE, O_RDONLY);
    if (!fd)
      fprintf (stderr, "Could not open touch device\n");
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    /* Update touch coordinates and fingers */
    fd_set fds;
    struct timeval timeout;

    while (1) {
      timeout.tv_usec = 0;
      timeout.tv_sec = 0;
      FD_ZERO (&fds);
      FD_SET (fd, &fds);
      count = read (fd, buffer, 64);
      if (count == 64) {
	fingercount = 0;
	p = buffer + 1;
	for (i=0; i<10; i++) {
	  if (*p > 0)
	    fingercount++;
	  else
	    break;
	  p += 6;
	}

	p = buffer + 2;
	fp = fingers;
	for (i=0; i<fingercount; i++) {
	  p[0] = p[4]; /* for some weird reason we have to swap these */
	  *fp = *((short *)p);
	  swapshort (fp);
	  fp++;
	  p += 2;
	  *fp = *((short *)p);
	  swapshort (fp);
	  fp++;
	  p += 4;
	}

#if 1
	/* print out results */
	printf ("fingers: %d", fingercount);
	fp = fingers;
	for (i=0; i<fingercount; i++) {
	  printf (" (%d,%d)", *fp++, *fp++);
	}
	printf ("\n");
#endif
      }
    }
#endif
  }

  void TouchNavigationTool::display(GLContextData& contextData) const
  {
  }

  void TouchNavigationTool::createTouchTool ()
  {
    Vrui::TouchNavigationToolFactory *toolfactory;
    Vrui::ToolInputLayout layout;
    Vrui::ToolInputAssignment assignment (layout);
    Vrui::ToolManager *toolmanager = Vrui::getToolManager ();

    toolfactory =
      new Vrui::TouchNavigationToolFactory (*toolmanager);

    FILE *file;
    if (file = fopen ("/dev/touch", "r")) {
      fclose (file);

      toolmanager->createTool (toolfactory, assignment);
    }
    else
      toolmanager->addClass (toolfactory);
  }
}

short Vrui::TouchNavigationTool::fingers[20];
short Vrui::TouchNavigationTool::fingercount;
bool Vrui::TouchNavigationTool::multitouch;
