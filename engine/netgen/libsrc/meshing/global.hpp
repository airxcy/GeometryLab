#ifndef FILE_GLOBAL
#define FILE_GLOBAL


/**************************************************************************/
/* File:   global.hh                                                      */
/* Author: Joachim Schoeberl                                              */
/* Date:   01. Okt. 95                                                    */
/**************************************************************************/

/*
  global functions and variables
*/

namespace netgen
{

  ///
   extern double GetTime ();
   extern void ResetTime ();

  ///
   extern int testmode;

  /// calling parameters
  // extern Flags parameters;

  // extern  MeshingParameters mparam;

   extern mutex tcl_todo_mutex;

  class  multithreadt
  {
  public:
    int pause;
    int testmode;
    int redraw;
    int drawing;
    int terminate;
    int running;
    double percent;
    const char * task;
    bool demorunning;
    string * tcl_todo = new string("");  // tcl commands set from parallel thread
    multithreadt();
  };

   extern volatile multithreadt multithread;

   extern string ngdir;
   extern DebugParameters debugparam;
   extern bool verbose;

   extern int h_argc;
   extern char ** h_argv;


   extern weak_ptr<Mesh> global_mesh;
   void SetGlobalMesh (shared_ptr<Mesh> m);

  // global communicator for netgen (dummy if no MPI)
  // extern  NgMPI_Comm ng_comm;
  
}

#endif
