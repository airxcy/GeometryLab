#include "GlfwOcctView.h"
#include "STEPCAFControl_Reader.hxx"
#include "Poly_CoherentTriangulation.hxx"
#include "TopExp_Explorer.hxx"
#include "TopoDS.hxx"
#include "TopoDS_Face.hxx"
#include "BRep_Tool.hxx"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>

#include <unordered_map>


GlfwOcctView anApp;
GLFWwindow* GLFWwin;

void init()
{
    glfwInit();
    const bool toAskCoreProfile = true;
    if (toAskCoreProfile)
    {
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
#if defined (__APPLE__)
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    }
    GLFWwin = anApp.initWindow(800, 600, "step file");
    anApp.initViewer();
    // Setup Dear ImGui context
    const char* glsl_version = "#version 330";
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    // Setup Platform/Renderer bindings
    ImGui_ImplGlfw_InitForOpenGL(GLFWwin, true);

    ImGui_ImplOpenGL3_Init(glsl_version);
    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    anApp.initDemoScene();
    if (anApp.myView.IsNull())
        return ;

    anApp.myView->MustBeResized();
    anApp.myOcctWindow->Map();
}

void cleanup()
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    anApp.cleanup();
}



int main (int, char**)
{
  init();
  STEPControl_Reader reader;
  IFSelect_ReturnStatus stat = reader.ReadFile("D:/projects/GeometryLab/src/occSTEP/test_model_step203.stp");
  Standard_Integer NbRoots = reader.NbRootsForTransfer();
  Standard_Integer num = reader.TransferRoots();
  TopoDS_Shape shape = reader.OneShape();
  //anApp.addShape(shape);
  while (!glfwWindowShouldClose(anApp.myOcctWindow->getGlfwWindow()))
  {
      // glfwPollEvents() for continuous rendering (immediate return if there are no new events)
      // and glfwWaitEvents() for rendering on demand (something actually happened in the viewer)
      //glfwPollEvents();
      glfwWaitEvents();
      if (!anApp.myView.IsNull())
      {
          anApp.Flush();
          ImGui_ImplOpenGL3_NewFrame();
          ImGui_ImplGlfw_NewFrame();
          Standard_Integer w, h;
          anApp.myOcctWindow->Size(w, h);
          ImGui::NewFrame();
          ImGui::SetNextWindowPos({ (float)w - 80, 0 });
          ImGui::Begin("main", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground);
          ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1, 0, 0, 1));
          if(ImGui::Button("classify"))
          {
              Handle(Poly_CoherentTriangulation) cohTris = new Poly_CoherentTriangulation;
              for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
              {
                  const TopoDS_Face& face = TopoDS::Face(exp.Current());
                  TopLoc_Location loc;
                  Handle(Poly_Triangulation) faceTris = BRep_Tool::Triangulation(face, loc);

                  std::unordered_map<int, int> nodesMap;
                  for (int iNode = 1; iNode <= faceTris->NbNodes(); ++iNode)
                  {
                      gp_Pnt P = faceTris->Node(iNode);
                      const int cohNodeIndex = cohTris->SetNode(P.XYZ());

                      nodesMap.insert({ iNode,cohNodeIndex });
                  }

                  for (int iTri = 1; iTri <= faceTris->NbTriangles(); ++iTri)
                  {
                      const Poly_Triangle& tri = faceTris->Triangle(iTri);
                      
                      int iNodes[3];
                      tri.Get(iNodes[0], iNodes[1], iNodes[2]);

                      int iCohNodes[3] = { nodesMap.at(iNodes[0]),nodesMap.at(iNodes[1]),nodesMap.at(iNodes[2]) };
                      cohTris->AddTriangle(iCohNodes[0], iCohNodes[1], iCohNodes[2]);

                  }
              }
              std::cout << "here" << std::endl;
              Handle(Poly_Triangulation) tris = cohTris->GetTriangulation();

              anApp.addTri(tris);
          }
          ImGui::End();
          ImGui::Render();
          ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
          glfwSwapBuffers(anApp.myOcctWindow->getGlfwWindow());
      }
  }
  cleanup();
  return 0;
}
