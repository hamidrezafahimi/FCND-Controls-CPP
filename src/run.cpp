#include "Common.h"
#include "Trajectory.h"
#include "Utility/Timer.h"

#include "Drawing/Visualizer_GLUT.h"
#include "Simulation/QuadDynamics.h"
#include "Simulation/Simulator.h"
#include "Utility/SimpleConfig.h"
#include "Utility/StringUtils.h"
#include "Drawing/GraphManager.h"
#include "MavlinkNode/MavlinkTranslation.h"

#include <unistd.h>

using SLR::Quaternion;
using SLR::ToUpper;

void KeyboardInteraction(V3F& force, shared_ptr<Visualizer_GLUT> vis);
bool receivedResetRequest = true;
bool paused = false;
void PrintHelpText();
void ProcessConfigCommands(shared_ptr<Visualizer_GLUT> vis);
void LoadScenario(string scenarioFile);
void ResetSimulation();

vector<QuadcopterHandle> quads;

shared_ptr<Visualizer_GLUT> visualizer;
shared_ptr<GraphManager> grapher;

float dtSim = 0.001f;
const int NUM_SIM_STEPS_PER_TIMER = 5;
Timer lastDraw;
V3F force, moment;

float simulationTime=0;
int randomNumCarry=-1;

void OnTimer(int v);

vector<QuadcopterHandle> CreateVehicles();

#include "MavlinkNode/MavlinkNode.h"
shared_ptr<MavlinkNode> mlNode;
string _scenarioFile="../config/1_Intro.txt";

/* (ISay-0001) These is where everything happens. A simple main function:
	- A help-text-print 
	- Making visualization and graph handle
	- Loading the all-over-code-unique config file
	- Setting episode callback and running main loop under a single 'glut' thread
About the single-threading the author say in 'README.md':
	" The simulation (including visualization) is implemented in a single thread. This is so that
	  you can safely breakpoint code at any point and debug, without affecting any part of the 
	  simulation. "
TODO: Separate visualization thread from the main thread (My simulation will be different!) */
int main(int argcp, char **argv)
{
  PrintHelpText();

  // initialize visualizer
  visualizer.reset(new Visualizer_GLUT(&argcp, argv));
  grapher.reset(new GraphManager(false));

  LoadScenario("");
  /* (ISay-0032) trace */
  glutTimerFunc(1,&OnTimer,0);
  
  glutMainLoop();

  return 0;
}


void LoadScenario(string)
{
	/* (ISay-0002) Loading config file - I suppose this is the main creation of the static (thus 
	unique) shared ptr because I removed the one author had written in the beginning of 'main' function*/	// load parameters
	ParamsHandle config = SimpleConfig::GetInstance();
    /* (ISay-0006) trace */
	config->Reset(_scenarioFile);
	config->PrintAll();
	printf("print all of config done \n");

	grapher->_sources.clear();
	grapher->graph1->RemoveAllElements();
	grapher->graph2->RemoveAllElements();
	grapher->RegisterDataSource(visualizer);

	/* (ISay-0011) Creating the vehicles */
	quads = CreateVehicles();

  /* (ISay-0027) For now I skip the grapher and visualizer stuff. In this round I'm just 
  seeking for the dynamics simulation */
	visualizer->Reset();
	visualizer->InitializeMenu(grapher->GetGraphableStrings());
	visualizer->quads = quads;
	visualizer->graph = grapher;

  	ProcessConfigCommands(visualizer);

  /* (ISay-0028) Seems handling of communication protocol */
	mlNode.reset();
	if(config->Get("Mavlink.Enable",0)!=0)
		mlNode.reset(new MavlinkNode());
  /* (ISay-0029) trace */
  	ResetSimulation();
}

int _simCount = 0;

/* (ISay-0030) Reseting stuff. Most of them done once at the beginning of the simulation */
void ResetSimulation()
{
  _simCount++;
  ParamsHandle config = SimpleConfig::GetInstance();

  printf("Simulation #%d (%s)\n", _simCount, _scenarioFile.c_str());

  randomNumCarry = -1;

  receivedResetRequest = false;
  simulationTime = 0;
  config->Reset(_scenarioFile); /*(ISay-0031) Read the file again to encounter user's online changes*/
  dtSim = config->Get("Sim.Timestep", 0.005f);
  
  for (unsigned i = 0; i<quads.size(); i++)
  {
    quads[i]->Reset();
  }
  grapher->Clear();
}

/*

 
This can be considered a recursive function because it calls itself within its own body. However, 
since it is called by the glutTimerFunc function, which is part of the GLUT library, it is not a 
direct recursion within the main thread of the program. Instead, it is a callback function that 
is repeatedly called by the GLUT library's event loop.
*/
void OnTimer(int)
{ /* (ISay-0033) The main loop of the simulation is performed within this function */
  ParamsHandle config = SimpleConfig::GetInstance();
  
  // logic to reset the simulation based on key input or reset conditions
  float endTime = config->Get("Sim.EndTime",-1.f);
  if(receivedResetRequest ==true ||
     (ToUpper(config->Get("Sim.RunMode", "Continuous"))=="REPEAT" && endTime>0 && simulationTime >= endTime))
  {
    ResetSimulation();
  }
  
  visualizer->OnMainTimer();
  
  /* (ISay-0034) If the simulation is not paused, the function runs a for loop to simulate the 
  quadcopters for a certain number of time steps. Within this loop, each quadcopter's Run function 
  is called with the current simulation time, time step size, and other parameters. After simulating 
  all the quadcopters, the simulation time is updated and the grapher object is updated with the 
  new data. The function then checks for any keyboard interactions and updates the visualizer 
  accordingly. */
  // main loop
  if (!paused)
  {
    for (int i = 0; i < NUM_SIM_STEPS_PER_TIMER; i++)
    {
      for (unsigned i = 0; i < quads.size(); i++)
      {
        quads[i]->Run(dtSim, simulationTime, randomNumCarry, force, moment);
      }
      simulationTime += dtSim;
    }
    grapher->UpdateData(simulationTime);
  }
  KeyboardInteraction(force, visualizer);
  
  /* (ISay-0040) If enough time has passed since the last draw, the function updates the visualizer 
  and grapher and sends some MAVLink packets if mlNode is not null */
  if (lastDraw.ElapsedSeconds() > 0.030)
  {
    if (quads.size() > 0)
    {
      visualizer->SetArrow(quads[0]->Position() - force, quads[0]->Position());
    }
    visualizer->Update();
    grapher->DrawUpdate();
    lastDraw.Reset();

    // temporarily here
    if (mlNode)
    {
      mlNode->Send(MakeMavlinkPacket_Heartbeat());
      mlNode->Send(MakeMavlinkPacket_Status());
      mlNode->Send(MakeMavlinkPacket_LocalPose(simulationTime, quads[0]->Position(), quads[0]->Velocity()));
      mlNode->Send(MakeMavlinkPacket_Attitude(simulationTime, quads[0]->Attitude(), quads[0]->Omega()));
    }
    
  }
  
  /* (ISay-0041) Sets a timer for 5 milliseconds and calls OnTimer again after that time has 
  elapsed This creates a loop that continuously calls OnTimer every 5 milliseconds until the 
  program is terminated */
  glutTimerFunc(5,&OnTimer,0);
}

vector<QuadcopterHandle> CreateVehicles()
{
  vector<QuadcopterHandle> ret;

	/* (ISay-0012) This is happened everywhere. It's just a simple accessing to a unique handle 
	to a unique config object */
  	ParamsHandle config = SimpleConfig::GetInstance();
	int i = 1;
	/* (ISay-0013) Loop until no vehicle is found in config. To add a vehicle, you should write:
	`Sim.Vehicle<number> = <vehicle-name>` in the config file */
	while (1)
	{
		char buf[100];
		sprintf_s(buf, 100, "Sim.Vehicle%d", i);
		if (config->Exists(buf)) {
			/* (ISay-0014) Creation of a drone: */
			QuadcopterHandle q = QuadDynamics::Create(config->Get(buf, "Quad"), (int)ret.size());
			grapher->RegisterDataSource(q);
			ret.push_back(q);
		} else
			break;
		i++;
	}
	return ret; /* (ISay-0026) trace */
}

void KeyboardInteraction(V3F& force, shared_ptr<Visualizer_GLUT> visualizer)
{
  bool keyPressed = false;
  const float forceStep = 0.04f;

  if (visualizer->IsSpecialKeyDown(GLUT_KEY_LEFT))
  {
    force += V3F(0, -forceStep, 0);
    keyPressed = true;
  }
  if (visualizer->IsSpecialKeyDown(GLUT_KEY_UP))
  {
    force += V3F(0, 0, -forceStep);
    keyPressed = true;
  }
  if (visualizer->IsSpecialKeyDown(GLUT_KEY_RIGHT))
  {
    force += V3F(0, forceStep, 0);
    keyPressed = true;
  }
  if (visualizer->IsSpecialKeyDown(GLUT_KEY_DOWN))
  {
    force += V3F(0, 0, forceStep);
    keyPressed = true;
  }
  if (visualizer->IsKeyDown('w') || visualizer->IsKeyDown('W'))
  {
    force += V3F(forceStep, 0, 0);
    keyPressed = true;
  }
  if (visualizer->IsKeyDown('s') || visualizer->IsKeyDown('S'))
  {
    force += V3F(-forceStep, 0, 0);
    keyPressed = true;
  }

  if (!keyPressed)
  {
    force = V3F();
  }
  if (force.mag() > 2.f)
  {
    force = force / force.mag() * 2.f;
  }

  if (visualizer->IsKeyDown('c') || visualizer->IsKeyDown('C'))
  {
    visualizer->graph->graph1->RemoveAllElements();
    visualizer->graph->graph2->RemoveAllElements();
  }

  if (visualizer->IsKeyDown('r') || visualizer->IsKeyDown('R'))
  {
    receivedResetRequest = true;
  }

  static bool key_space_pressed = false;

  if (visualizer->IsKeyDown(' '))
  {
    if (!key_space_pressed)
    {
      key_space_pressed = true;
      paused = !paused;
      visualizer->paused = paused;
    }
  }
  else
  {
    key_space_pressed = false;
  }
}

void ProcessConfigCommands(shared_ptr<Visualizer_GLUT> vis)
{
  ParamsHandle config = SimpleConfig::GetInstance();
  int i = 1;
  while (1)
  {
    char buf[100];
    sprintf_s(buf, 100, "Commands.%d", i);
    string cmd = config->Get(buf, "");
    if (cmd == "") break;
    vis->OnMenu(cmd);
    i++;
  }
}

void PrintHelpText()
{
  printf("SIMULATOR!\n");
  printf("Select main window to interact with keyboard/mouse:\n");
  printf("LEFT DRAG / X+LEFT DRAG / Z+LEFT DRAG = rotate, pan, zoom camera\n");
  printf("W/S/UP/LEFT/DOWN/RIGHT - apply force\n");
  printf("C - clear all graphs\n");
  printf("R - reset simulation\n");
  printf("Space - pause simulation\n");
}
