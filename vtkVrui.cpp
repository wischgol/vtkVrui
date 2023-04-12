/***********************************************************************
vtkVrui - Program for rendering on virtual reality display systems
using Vrui and VTK. It is based on Vrui's basic framework and VTK's
Mace example.
Copyright (c) 2016-2023 Thoams Wischgoll

***********************************************************************/
#include <vtkVersion.h>
#include <vtkVersionMacros.h>

#if (VTK_MAJOR_VERSION > 6)
#define VTK6_2
#else
#if (VTK_MAJOR_VERSION == 6)
#if (VTK_MINOR_VERSION == 2)
#define VTK6_2
#else
#define VTK6
#endif
#endif
#endif

//#define OSPRAY

#include <Math/Math.h>
#include <Geometry/OrthogonalTransformation.h>
#include <GL/gl.h>
#include <GL/glx.h>
#include <GL/glu.h>
#include <GL/GLColorTemplates.h>
#include <GL/GLVertexTemplates.h>
#include <GL/GLContextData.h>
#include <GL/GLGeometryWrappers.h>
#include <GLMotif/Button.h>
#include <GLMotif/Menu.h>
#include <GLMotif/PopupMenu.h>
#include <Vrui/Vrui.h>
#include <Vrui/Application.h>
#include <Vrui/VRWindow.h>

// these are the include files needed for the multipass rendering so
// that we can render into non-VTK windows
#include <vtkRenderer.h>
#include <vtkXOpenGLRenderWindow.h>
#include <vtkOpenGLRenderer.h>
#include <vtkOpaquePass.h>
#include <vtkLightsPass.h>
#include <vtkOverlayPass.h>
#include <vtkTranslucentPass.h>
#include <vtkVolumetricPass.h>
#include <vtkRenderPassCollection.h>
#include <vtkSequencePass.h>
//#include "vtkOSPRayPass.h"
//#include "vtkOSPRayRendererNode.h"

// these are the standard VTK includes
#include <vtkSmartPointer.h>
#include <vtkPointData.h>
#include <vtkSphereSource.h>
#include <vtkArrowSource.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkGlyph3D.h>
#include <vtkCellArray.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkCamera.h>
#include <vtkTransform.h>
#include <vtkMatrix4x4.h>
#ifdef VTK6_2
#include <vtkExternalOpenGLCamera.h>
#include <vtkExternalOpenGLRenderer.h>
#include <vtkExternalOpenGLRenderWindow.h>
#endif
#include "vtkPolyDataNormals.h"

#include "TouchNavigationTool.h"

class VruiDemo:public Vrui::Application,public GLObject
{
  /* Embedded classes: */
private:
  class DataItem:public GLObject::DataItem // Data structure storing OpenGL-dependent application data
  {
    /* Elements: */
  public:
    GLuint textureObjectId; // Texture object ID of some texture
    GLuint displayListId; // Display list ID of some display list
#ifdef VTK6_2
    vtkExternalOpenGLRenderer
#else
    vtkRenderer
#endif
      *m_Renderer;
#ifdef VTK6_2
    vtkExternalOpenGLRenderWindow
#else
    vtkRenderWindow
#endif
      *m_RenderWindow;
		
    /* Constructors and destructors: */
    DataItem(void)
    {
      /* Create a texture object to hold a texture: */
      glGenTextures(1,&textureObjectId);
			
      /* Create a display list: */
      displayListId=glGenLists(1);
    };
    ~DataItem(void)
    {
      /* Destroy the texture object: */
      glDeleteTextures(1,&textureObjectId);
			
      /* Destroy the display list: */
      glDeleteLists(displayListId,1);
    };

    void Setup()
    {
      vtkSmartPointer<vtkSphereSource> sphereSource =
	vtkSmartPointer<vtkSphereSource>::New();
      sphereSource->Update();
 
      vtkSmartPointer<vtkPolyData> input =
	vtkSmartPointer<vtkPolyData>::New();
      input->ShallowCopy(sphereSource->GetOutput());
 
      vtkSmartPointer<vtkArrowSource> arrowSource =
	vtkSmartPointer<vtkArrowSource>::New();
 
      vtkSmartPointer<vtkGlyph3D> glyph3D =
	vtkSmartPointer<vtkGlyph3D>::New();
      glyph3D->SetSourceConnection(arrowSource->GetOutputPort());
      glyph3D->SetVectorModeToUseNormal();
#if VTK_MAJOR_VERSION <= 5
      glyph3D->SetInput(input);
#else
      glyph3D->SetInputData(input);
#endif
      glyph3D->SetScaleFactor(.2);
      glyph3D->Update();
 
      // Visualize
      vtkSmartPointer<vtkPolyDataMapper> mapper =
	vtkSmartPointer<vtkPolyDataMapper>::New();
#ifdef OSPRAY
      vtkSmartPointer<vtkPolyDataNormals> normals =
	vtkSmartPointer<vtkPolyDataNormals>::New();
      normals->SetInputConnection(glyph3D->GetOutputPort());
      mapper->SetInputConnection(normals->GetOutputPort());
#else
      mapper->SetInputConnection(glyph3D->GetOutputPort());
#endif
 
      vtkSmartPointer<vtkActor> actor =
	vtkSmartPointer<vtkActor>::New();
      actor->SetMapper(mapper);

      vtkSmartPointer<vtkPolyDataMapper> mapper2 =
	vtkSmartPointer<vtkPolyDataMapper>::New();
#ifdef OSPRAY
      vtkSmartPointer<vtkPolyDataNormals> normals2 =
	vtkSmartPointer<vtkPolyDataNormals>::New();
      normals2->SetInputConnection(sphereSource->GetOutputPort());
      mapper2->SetInputConnection(normals->GetOutputPort());
#else
      mapper2->SetInputConnection(sphereSource->GetOutputPort());
#endif
      vtkSmartPointer<vtkActor> actor2 =
	vtkSmartPointer<vtkActor>::New();
      actor2->SetMapper(mapper2);
 
      // Everything below is needed for the multipass rendering to
      // render into non-VTK windows; you only need to add your actors
      // to the renderer

      // Setup renderer and render window
      m_Renderer =
#ifdef VTK6_2
	vtkExternalOpenGLRenderer::New ();
#else
	vtkRenderer::New();
#endif
      m_Renderer->AddActor(actor);
      m_Renderer->AddActor(actor2);

      m_Renderer->RemoveAllLights ();

#ifdef OSPRAY
      vtkSmartPointer<vtkOSPRayPass> ospray=
	vtkSmartPointer<vtkOSPRayPass>::New();
      vtkOSPRayRendererNode *ren = ospray->GetSceneGraph();
      ren->SetRendererType("scivis", m_Renderer);
      //ren->SetRendererType("pathtracer", m_Renderer);
      m_Renderer->SetPass(ospray);
#endif

#ifdef VTK6_2
      m_RenderWindow = vtkExternalOpenGLRenderWindow::New ();
      m_RenderWindow->AddRenderer(m_Renderer);
#else
#ifdef VTK6
      m_RenderWindow = vtkRenderWindow::New ();
      m_RenderWindow->AddRenderer(m_Renderer);
      // Here is the trick: we ask the RenderWindow to join the
      // current OpenGL context created by Vrui
      m_RenderWindow->InitializeFromCurrentContext();
      Vrui::VRWindow *window = Vrui::getWindow (0);
      m_RenderWindow->SetSize (window->getViewportSize (0),
			       window->getViewportSize (1));

      vtkOpaquePass  * opaquePass = vtkOpaquePass::New();
      vtkLightsPass * lights = vtkLightsPass::New();
      vtkOverlayPass  * overlayPass = vtkOverlayPass::New();
      vtkTranslucentPass  * translucentPass = vtkTranslucentPass::New();
      //vtkVolumetricPass  * volumetricPass = vtkVolumetricPass::New();
      vtkRenderPassCollection * opasses = vtkRenderPassCollection::New();
      opasses->AddItem(lights);
      opasses->AddItem(opaquePass);
      opasses->AddItem(overlayPass);
      opasses->AddItem(translucentPass);
      //opasses->AddItem(volumetricPass);

      vtkSequencePass * seq = vtkSequencePass::New();
      seq->SetPasses(opasses);

      ((vtkOpenGLRenderer*)m_Renderer)->SetPass(seq);
#else
      m_RenderWindow = vtkXOpenGLRenderWindow::New();
      GLXDrawable drawable = glXGetCurrentDrawable();
      m_RenderWindow->SetWindowId(&drawable);
      //m_RenderWindow->SetParentId(ParentId);
      //m_RenderWindow->SetContextId(glXGetCurrentContext());
      m_RenderWindow->SetDisplayId (glXGetCurrentDisplay());
      //m_RenderWindow->SetDeviceContext((HDC)DeviceContext);
      m_RenderWindow->AddRenderer(m_Renderer);

      vtkOpaquePass  * opaquePass = vtkOpaquePass::New();
      vtkLightsPass * lights = vtkLightsPass::New();
      vtkOverlayPass  * overlayPass = vtkOverlayPass::New();
      vtkTranslucentPass  * translucentPass = vtkTranslucentPass::New();
      //vtkVolumetricPass  * volumetricPass = vtkVolumetricPass::New();
      vtkRenderPassCollection * opasses = vtkRenderPassCollection::New();
      opasses->AddItem(lights);
      opasses->AddItem(opaquePass);
      opasses->AddItem(overlayPass);
      opasses->AddItem(translucentPass);
      //opasses->AddItem(volumetricPass);

      vtkSequencePass * seq = vtkSequencePass::New();
      seq->SetPasses(opasses);

      m_Renderer->SetPass(seq);
#endif
#endif
    }

    void render ()
    {
      double modelview[16];
      double projection[16];
      GLint viewport[4];
      double vtkviewport[4];

#ifdef VTK6_2
#if 0
      projection[0] *= (double)viewport[2]/(double)viewport[3];
      glGetDoublev (GL_MODELVIEW_MATRIX, modelview);
      glGetDoublev (GL_PROJECTION_MATRIX, projection);
      vtkExternalOpenGLCamera *camera =
	(vtkExternalOpenGLCamera *)m_Renderer->GetActiveCamera ();
      camera->SetViewTransformMatrix (modelview);
      camera->SetProjectionTransformMatrix (projection);
      // glGetIntegerv (GL_VIEWPORT, viewport);
      // m_RenderWindow->SetPosition (viewport[0], viewport[1]);
      // m_RenderWindow->SetSize (viewport[2], viewport[3]);
      // //m_Renderer->SetAspect (1.0, 0.1);
      // vtkTransform *transform=vtkTransform::New();
      // vtkMatrix4x4 *mat=vtkMatrix4x4::New();
      // mat->Zero ();
      // mat->SetElement(0,0,viewport[2]/viewport[3]);
      // mat->SetElement(1,1,1);
      // mat->SetElement(2,2,1);
      // mat->SetElement(3,3,1);
      // transform->SetMatrix(mat);
      // camera->SetUserTransform(transform);
#endif
#if (VTK_MAJOR_VERSION < 7)
      glGetIntegerv (GL_VIEWPORT, viewport);

      // side-by-side rendering is messed up so we need to set things
      // up manually for VTK if that is the case
      if (viewport[1] > 0) {
	vtkviewport[0] = 0.0;
	vtkviewport[1] = 0.5;
	vtkviewport[2] = 1.0;
	vtkviewport[3] = 1.0;
	viewport[3] *= 2;
      }
      else if (viewport[0] > 0) {
	vtkviewport[0] = 0.5;
	vtkviewport[1] = 0.0;
	vtkviewport[2] = 1.0;
	vtkviewport[3] = 1.0;
	viewport[2] *= 2;
      }
      else {
	vtkviewport[0] = 0.0;
	vtkviewport[1] = 0.0;
	vtkviewport[2] = 1.0;
	vtkviewport[3] = 1.0;
      }
      m_Renderer->SetViewport (vtkviewport);
      m_RenderWindow->SetPosition (viewport[0], viewport[1]);
      m_RenderWindow->SetSize (viewport[2], viewport[3]);
#endif
      m_RenderWindow->Render ();
#else
#ifdef VTK6
      //glMatrixMode (GL_PROJECTION);
      //glPushMatrix ();
      //glMatrixMode (GL_MODELVIEW);
      //glPushMatrix ();
      //glPushAttrib(GL_ALL_ATTRIB_BITS);
      //glGetDoublev (GL_MODELVIEW_MATRIX, modelview);
      //glGetDoublev (GL_PROJECTION_MATRIX, projection);
      //m_Renderer->GetActiveCamera ()->SetModelTransformMatrix (modelview);
      //m_Renderer->GetActiveCamera ()->GetProjectionTransformObject ()
      //->SetMatrix (projection);
      m_RenderWindow->Render ();
      //glPopAttrib();
      //glMatrixMode (GL_PROJECTION);
      //glPopMatrix ();
      //glMatrixMode (GL_MODELVIEW);
      //glPopMatrix ();
#else
      //glPushAttrib(GL_ALL_ATTRIB_BITS);
      m_RenderWindow->Start ();
      //m_Renderer->Render();
      //glPopAttrib();
#endif
#endif
    }
  };
	
  /* Elements: */
private:
	
  /* Vrui parameters: */
  GLMotif::PopupMenu* mainMenu; // The program's main menu
	
  /* Private methods: */
  GLMotif::PopupMenu* createMainMenu(void); // Creates the program's main menu
	
  /* Constructors and destructors: */
public:
  VruiDemo(int& argc,char**& argv,char**& appDefaults); // Initializes the Vrui toolkit and the application
  ~VruiDemo(void); // Shuts down the Vrui toolkit
	
  /* Methods: */
  virtual void initDisplay(GLContextData& contextData) const; // Called once upon creation of each OpenGL context
  virtual void initContext(GLContextData& contextData) const;
  virtual void display(GLContextData& contextData) const; // Called for every eye and every window on every frame
  virtual void frame(void); // Called exactly once per frame
  void resetNavigationCallback(Misc::CallbackData* cbData); // Method to reset the Vrui navigation transformation to its default
};

/*************************
Methods of class VruiDemo:
*************************/

GLMotif::PopupMenu* VruiDemo::createMainMenu(void)
{
  /* Create a popup shell to hold the main menu: */
  GLMotif::PopupMenu* mainMenuPopup=new GLMotif::PopupMenu("MainMenuPopup",Vrui::getWidgetManager());
  mainMenuPopup->setBorderWidth(0.0f);
  mainMenuPopup->setBorderType(GLMotif::Widget::RAISED);
  mainMenuPopup->setBorderColor(Vrui::getUiBgColor());
  mainMenuPopup->setBackgroundColor(Vrui::getUiBgColor());
  mainMenuPopup->setForegroundColor(Vrui::getUiFgColor());
  mainMenuPopup->setMarginWidth(Vrui::getUiSize());
  mainMenuPopup->setTitleSpacing(Vrui::getUiSize());
  mainMenuPopup->setTitle("Vrui Demonstration",Vrui::getUiFont());
	
  /* Create the main menu itself: */
  GLMotif::Menu* mainMenu=new GLMotif::Menu("MainMenu",mainMenuPopup,false);
  mainMenu->setBorderWidth(0.0f);
  mainMenu->setOrientation(GLMotif::Menu::VERTICAL);
  mainMenu->setNumMinorWidgets(1);
  mainMenu->setMarginWidth(0.0f);
  mainMenu->setSpacing(Vrui::getUiSize());
	
  /* Create a button: */
  GLMotif::Button* resetNavigationButton=new GLMotif::Button("ResetNavigationButton",mainMenu,"Reset Navigation",Vrui::getUiFont());
	
  /* Add a callback to the button: */
  resetNavigationButton->getSelectCallbacks().add(this,&VruiDemo::resetNavigationCallback);
	
  /* Finish building the main menu: */
  mainMenu->manageChild();
	
  return mainMenuPopup;
}

VruiDemo::VruiDemo(int& argc,char**& argv,char**& appDefaults)
  :Vrui::Application(argc,argv,appDefaults),
   mainMenu(0)
{
  /* Create the user interface: */
  mainMenu=createMainMenu();
	
  /* Install the main menu: */
  Vrui::setMainMenu(mainMenu);
	
  /* Set the navigation transformation: */
  resetNavigationCallback(0);
}

VruiDemo::~VruiDemo(void)
{
  delete mainMenu;
}

void VruiDemo::initDisplay(GLContextData& contextData) const
{
  /*********************************************************************
	This method is called once for every OpenGL context created for the
	program. It must not change application state, but only create
	OpenGL-dependent application data and store them in the GLContextData
	object for retrieval in the display method.
  *********************************************************************/

  glDepthFunc(GL_LESS);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_COLOR_MATERIAL);
  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glLightModeli (GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
}
	
void VruiDemo::initContext(GLContextData& contextData) const
{
  /* Create context data item and store it in the GLContextData object: */
  DataItem* dataItem=new DataItem;
  int left, right, bottom, top, near, far;
  contextData.addDataItem(this,dataItem);
	
  /* Now is the time to upload all needed texture data: */
  glBindTexture(GL_TEXTURE_2D,dataItem->textureObjectId);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
  GLfloat texImage[4][4]=
    {
      {0.0f, 0.0f, 1.0f, 1.0f},
      {0.0f, 0.0f, 1.0f, 1.0f},
      {1.0f, 1.0f, 0.0f, 0.0f},
      {1.0f, 1.0f, 0.0f, 0.0f}
    };
  glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA,4,4,0,GL_LUMINANCE,GL_FLOAT,texImage);
  glBindTexture(GL_TEXTURE_2D,0);

  dataItem->Setup ();
	
#if !defined(VTK6) && !defined(VTK6_2)
  /* Now is also the time to upload all display lists' contents: */
  glNewList(dataItem->displayListId,GL_COMPILE);

  //glScale (2.5, 2.5, 2.5);
  glScale (7.5, 7.5, 7.5);
  glRotatef (10.0, 1.0, 0.0, 0.0);

  /* Draw the scene */
  //glPushAttrib(GL_LIGHTING_BIT);
  dataItem->render ();
  //glPopAttrib();

  /* Finish the display list: */
  glEndList();
#endif
}

void VruiDemo::display(GLContextData& contextData) const
{
  /*********************************************************************
	This method is called once for every eye in every window on every
	frame. It must not change application state, as it is called an
	unspecified number of times. It also must not clear the screen or
	initialize the OpenGL transformation matrices. When this method is
	called, Vrui will already have rendered its own state (menus etc.) and
	have set up the transformation matrices so that all rendering in this
	method happens in navigation (i.e., model) coordinates.
  *********************************************************************/
	
  /* Get the OpenGL-dependent application data from the GLContextData object: */
  DataItem* dataItem=contextData.retrieveDataItem<DataItem>(this);
	
  /* Insert generic OpenGL code here... */
  // ...
	
  /* Use the texture uploaded in the initDisplay() method: */
  glBindTexture(GL_TEXTURE_2D,dataItem->textureObjectId);
  // ...
  glBindTexture(GL_TEXTURE_2D,0);
	
  /* Set up the animation transformation: */
  glPushMatrix();
	
  GLfloat mat_ambient[] = { 0.7, 0.7, 0.7, 1.0 };
  GLfloat mat_diffuse[] = { 0.8, 0.8, 0.8, 1.0 };
  GLfloat mat_specular[] = { 0.5, 0.5, 0.5, 0.5 };
  GLfloat high_shininess[] = { 50.0 };

  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, high_shininess);

  glDepthFunc(GL_LESS);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_COLOR_MATERIAL);
  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glLightModeli (GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

  /* Call the display list created in the initDisplay() method: */
  //glDisable (GL_CULL_FACE);
#if defined(VTK6) || defined (VTK6_2)
  dataItem->render ();
#else
  glCallList(dataItem->displayListId);
#endif
  //glEnable (GL_CULL_FACE);
	
  /* Reset the animation transformation: */
  glPopMatrix();
}

void VruiDemo::frame(void)
{
  /*********************************************************************
	This function is called exactly once per frame, no matter how many
	eyes or windows exist. It is the perfect place to change application
	state (run simulations, animate models, etc.).
  *********************************************************************/
	
  /* Get the time since the last frame: */
  double frameTime=Vrui::getCurrentFrameTime();
	
}

void VruiDemo::resetNavigationCallback(Misc::CallbackData* cbData)
{
  /* Reset the Vrui navigation transformation: */
  Vrui::NavTransform t=Vrui::NavTransform::identity;
  t*=Vrui::NavTransform::translateFromOriginTo(Vrui::getDisplayCenter());
  t*=Vrui::NavTransform::scale(Vrui::getInchFactor());
  Vrui::setNavigationTransformation(t);
	
  /*********************************************************************
	Now the coordinate system's origin is in the middle of the
	environment, e.g., in the middle of the workbench, and one coordinate
	unit is one inch long.
  *********************************************************************/
}

int main(int argc,char* argv[])
{
  /* Create an application object: */
  char** appDefaults=0; // This is an additional parameter no one ever uses
  VruiDemo app(argc,argv,appDefaults);
	
  Vrui::TouchNavigationTool::createTouchTool ();
  
  /* Run the Vrui main loop: */
  app.run();
	
  /* Exit to OS: */
  return 0;
}
