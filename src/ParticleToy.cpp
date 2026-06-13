// ParticleToy.cpp defines the entry point for the console application.

#include "CircularWireConstraint.h"
#include "Particle.h"
#include "RodConstraint.h"
#include "CircularWireConstraint.h"
#include "GravityForce.h"
#include "SpringForce.h"
#include "MouseSpringForce.h"
#include "Constraint.h"
#include "PointConstraint.h"
#include "WindForce.h"
#include "CollisionHandler.h"
#include "AngularSpring.h"
#include "TouchSphereForce.h"
#include "FluidScene.h"   // Project 2: Stable Fluids scene and feature toggles
#include "imageio.h"      // Project 1 PNG screenshot helper kept for Project 2

#include "GLCompat.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <cmath>
#include <iostream>

/* macros */
/* external definitions (from solver) */
extern void simulation_step(std::vector<Particle *> pVector,
							std::vector<Force *> fVector,
							std::vector<Constraint *> cVector,
							float dt);

/* global variables */

static int N;
static float d;
static int dsim;
static int dump_frames;
static int frame_number;
int solver_type = 0;
enum SceneType
{
	SCENE_CLOTH = 0,
	SCENE_HAIR = 1,
	SCENE_PENDULUM = 2,
	SCENE_BALL = 3,
	SCENE_BRIDGE = 4,
	SCENE_FLUID = 5,
};
float dt = 0.01f;
bool use_sqrt_rodConstraint = true;	   // toggle between squared and sqrt formula for RodConstraint
static double mouse_ks = 0.50;		   // spring stiffness for mouse interaction
static double mouse_kd = 0.10;		   // damping
static WindForce *windForce = NULL;	   // global pointer to the wind force
static Vec2f windDirection(-1.0, 0.0); // blow left
static float windStrength = 0.15f;
static bool enableWind = false; // toggle wind force
static bool enableGravity = true;
static bool enableSpring = true;
static bool enableRod = true;
static bool enableWire = true;
static int scene_type = SCENE_FLUID; // starts in the Project 2 fluid scene
const Vec2f gravityDirection(0.0, -1.0); // gravity points down
const float gravityStrength = 0.1f;
static float cameraX = 0.0f; // track camera X position

// cloth variables
int cloth_rows = 5; // rows
int cloth_columns = 5; // columns
float cloth_spacing = 0.045f; // spacing between particles in the cloth
bool fixRowBool = true; // whether to fix in space the top row of the cloth
bool fixCornersBool = false; // whether to fix in space the corners of the cloth
double spring_ks = 10.0f; // spring stiffness for cloth springs
double spring_kd = 3.5f; // damping for cloth springs
float wall_restitution = 0.5f; // restitution coefficient for wall collisions
float wall_friction_coeff = 0.1f; // friction coefficient for wall collisions
float particle_diameter = 0.045f; // diameter of each particle

// hair variables
int hair_segments = 5; // number of segments in the hair
float hair_segment_length = 0.05f; // length of each hair segment

// sphere force
float touchSphereRadius = 0.2f;
float touchSphereMagnitude = 3.0f;

// bridge
bool use_angular_springs_bridge = true;

// static Particle *pList;
static std::vector<Particle *> pVector;
static std::vector<Wall> wallVector;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int mouse_release[3];
static int mouse_shiftclick[3];
static int omx, omy, mx, my;
// static int hmx, hmy;

static std::vector<Constraint *> cVector;

static MouseSpringForce* mouseSpring = NULL;
static TouchSphereForce* touchSphereForce = NULL;
static std::vector<Force *> fVector;
// Single Project 2 scene object. The existing Project 1 framework calls into it
// for update/draw/input whenever scene_type == SCENE_FLUID.
static FluidScene fluidScene;

/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/

static void free_data(void)
{
	// Clean up particles, forces and constraints
	for (size_t i = 0; i < pVector.size(); i++)
	{
		delete pVector[i];
	}
	pVector.clear();

	for (size_t i = 0; i < fVector.size(); i++)
	{
		delete fVector[i];
	}
	fVector.clear();
	windForce = NULL;

	for (size_t i = 0; i < cVector.size(); i++)
	{
		delete cVector[i];
	}
	cVector.clear();
}

static void clear_data(void)
{
	int ii, size = pVector.size();

	for (ii = 0; ii < size; ii++)
	{
		pVector[ii]->reset();
	}
}

/* Scenes */
static void cloth_scene() {
	// initialize walls
	wallVector.clear();
	wallVector.emplace_back(Vec2f(-1.0f, -0.92f), Vec2f(1.0f, -0.92f));
	wallVector.emplace_back(Vec2f(-0.92f, -1.0f), Vec2f(-0.92f, 1.0f));

	// create cloth particles in row major order
	for (int i = 0; i < cloth_rows; ++i)
	{
		for (int j = 0; j < cloth_columns; ++j)
		{
			float x = j * cloth_spacing - (cloth_columns - 1) * cloth_spacing / 2.0f; // center the cloth
			float y = 0.9f - i * cloth_spacing; // start from almost top and go down
			pVector.push_back(new Particle(Vec2f(x, y)));
		}
	}

	// orient wall normals consistently toward the cloth
	Vec2f clothCenter(0.0f, 0.0f);
	for (Particle* p : pVector) {
		clothCenter += p->m_Position;
	}
	clothCenter /= float(pVector.size());
	for (Wall& wall : wallVector) {
		wall.orientNormalToPoint(clothCenter);
	}

	// cloth spring connectivity
	for (int i = 0; i < cloth_rows; ++i)
	{
		for (int j = 0; j < cloth_columns; ++j)
		{
			int idx = i * cloth_columns + j;
			// structural springs
			// connect to particle on the right
			if (j < cloth_columns - 1)
			{
				fVector.push_back(new SpringForce(pVector[idx], pVector[idx + 1], cloth_spacing, spring_ks, spring_kd));
			}
			// connect to particle below
			if (i < cloth_rows - 1)
			{
				fVector.push_back(new SpringForce(pVector[idx], pVector[idx + cloth_columns], cloth_spacing, spring_ks, spring_kd));
			}
			// shear springs
			// connect to particle diagonally down-right
			if (i < cloth_rows - 1 && j < cloth_columns - 1)
			{
				fVector.push_back(new SpringForce(pVector[idx], pVector[idx + cloth_columns + 1], cloth_spacing * sqrt(2), spring_ks, spring_kd));
			}
			// connect to particle diagonally down-left
			if (i < cloth_rows - 1 && j > 0)
			{
				fVector.push_back(new SpringForce(pVector[idx], pVector[idx + cloth_columns - 1], cloth_spacing * sqrt(2), spring_ks, spring_kd));
			}
			// flexion springs
			// horizontal (right + 2)
			if (j < cloth_columns - 2)
			{
				fVector.push_back(new SpringForce(pVector[idx], pVector[idx + 2], cloth_spacing * 2, spring_ks, spring_kd));
			}
			// vertical (down + 2)
			if (i < cloth_rows - 2)
			{
				fVector.push_back(new SpringForce(pVector[idx], pVector[idx + 2 * cloth_columns], cloth_spacing * 2, spring_ks, spring_kd));
			}
			// horizontal (right + 2) and vertical (down + 2)
			if (i < cloth_rows - 2 && j < cloth_columns - 2)
			{
				fVector.push_back(new SpringForce(pVector[idx], pVector[idx + 2 * cloth_columns + 2], cloth_spacing * sqrt(8), spring_ks, spring_kd));
			}
		}
	}

	// add gravity to all particles
	fVector.push_back(new GravityForce(pVector, gravityDirection * gravityStrength));

	if (fixRowBool) {
		// fix top row particles
		for (int j = 0; j < cloth_columns; ++j) {
			Particle* p = pVector[j];
			// x dir fix
			cVector.push_back(new PointConstraintX(p, p->m_ConstructPos[0]));
			// y dir fix
			cVector.push_back(new PointConstraintY(p, p->m_ConstructPos[1]));
		}
	} else if (fixCornersBool) {
		// fix top row corners
		cVector.push_back(new PointConstraintX(pVector[0], pVector[0]->m_ConstructPos[0]));
		cVector.push_back(new PointConstraintY(pVector[0], pVector[0]->m_ConstructPos[1]));
		cVector.push_back(new PointConstraintX(pVector[cloth_columns - 1], pVector[cloth_columns - 1]->m_ConstructPos[0]));
		cVector.push_back(new PointConstraintY(pVector[cloth_columns - 1], pVector[cloth_columns - 1]->m_ConstructPos[1]));
	}

	// add wind force
	windForce = new WindForce(pVector, windDirection, windStrength, enableWind);
	fVector.push_back(windForce);
}

static void hair_scene() {
    float rest_len = hair_segment_length;
    float perturb_offset = rest_len * 0.5f;

    // generate main axis particles and perturbed midpoint particles
    std::vector<Particle*> main_nodes;
    std::vector<Particle*> offset_nodes;

    for (int i = 0; i < hair_segments; ++i) {
        float y = 0.9f - i * rest_len;
        Particle* p = new Particle(Vec2f(0.0f, y));
        pVector.push_back(p);
        main_nodes.push_back(p);

        if (i < hair_segments - 1) {
            // create perturbed particle at the midpoint, offset along X
            Particle* p_off = new Particle(Vec2f(perturb_offset, y - rest_len * 0.5f));
            pVector.push_back(p_off);
            offset_nodes.push_back(p_off);
        }
    }

    // connect structural and triangle-forming springs
    for (int i = 0; i < hair_segments - 1; ++i) {
        // structural edge
        fVector.push_back(new SpringForce(main_nodes[i], main_nodes[i+1], rest_len, spring_ks, spring_kd));
        
        // triangle edges
		// main node to offset node
        float diag_len = sqrt(pow(perturb_offset, 2) + pow(rest_len * 0.5f, 2));
        fVector.push_back(new SpringForce(main_nodes[i], offset_nodes[i], diag_len, spring_ks, spring_kd));
        fVector.push_back(new SpringForce(main_nodes[i+1], offset_nodes[i], diag_len, spring_ks, spring_kd));
    }

    // angular torsion springs across the triangles
    for (int i = 0; i < hair_segments - 2; ++i) {
        double angular_ks = spring_ks * 0.005; 
        double angular_kd = spring_kd * 0.005;
        fVector.push_back(new AngularSpring(main_nodes[i], main_nodes[i+1], main_nodes[i+2], M_PI, angular_ks, angular_kd));
    }

    // fix root
    cVector.push_back(new PointConstraintX(main_nodes[0], main_nodes[0]->m_ConstructPos[0]));
    cVector.push_back(new PointConstraintY(main_nodes[0], main_nodes[0]->m_ConstructPos[1]));

	// gravity force
    fVector.push_back(new GravityForce(pVector, gravityDirection * gravityStrength));

	// wind force
	windForce = new WindForce(pVector, windDirection, windStrength, enableWind);
	fVector.push_back(windForce);
}

static void pendulum_scene()
{
	const double dist = 0.2;
	const Vec2f center(0.0, 0.3);
	const Vec2f offset(dist, 0.0);

	// 3 particles in a line
	pVector.push_back(new Particle(center + offset));
	pVector.push_back(new Particle(center + offset + offset));
	pVector.push_back(new Particle(center + offset + offset + offset));

	// gravity and spring
	fVector.push_back(new GravityForce(pVector, gravityDirection * gravityStrength));
	fVector.push_back(new SpringForce(pVector[0], pVector[1], dist, spring_ks, spring_kd));

	// circular wire constraint to first particle
	cVector.push_back(new CircularWireConstraint(pVector[0], center, dist));
	// rod between 1st and 2nd; 2nd and 3rd
	cVector.push_back(new RodConstraint(pVector[1], pVector[2], dist, use_sqrt_rodConstraint));

	// wind force
	windForce = new WindForce(pVector, windDirection, windStrength, enableWind);
	fVector.push_back(windForce);
}

static void ball_scene() {
	wallVector.clear();

	// procedural terrain for the floor
	float x_start = -2.0f;
	float x_end = 1000.0f;
	float step = 0.2f;
	float prev_x = x_start;
	float prev_y = -0.6f; // start height
	
	for (float x = x_start + step; x <= x_end; x += step) {
		// rolling hills
		float y = -0.7f + sin(x * 1.5f) * 0.15f + sin(x * 0.5f) * 0.15f;
		
		Wall floor_segment(Vec2f(prev_x, prev_y), Vec2f(x, y));
		floor_segment.orientNormalToPoint(Vec2f((prev_x + x) / 2.0f, y + 1.0f)); // normal points up
		wallVector.push_back(floor_segment);
		
		prev_x = x;
		prev_y = y;
	}
	
	// endless ceiling
	Wall ceiling(Vec2f(x_start, 0.9f), Vec2f(x_end, 0.9f)); // normal points down
	ceiling.orientNormalToPoint(Vec2f(0.0f, 0.0f));
	wallVector.push_back(ceiling);
	
	// left boundary wall
	Wall leftWall(Vec2f(x_start, 1.0f), Vec2f(x_start, -1.0f)); // normal points right
	leftWall.orientNormalToPoint(Vec2f(0.0f, 0.0f)); 
	wallVector.push_back(leftWall);

	// ball particles
	int num_nodes = 14;
	float radius = 0.2f;
	Vec2f center(0.0f, 0.0f);
	
	// center particle
	Particle* pCenter = new Particle(center);
	pVector.push_back(pCenter);
	
	// ring particles
	for (int i = 0; i < num_nodes; i++) {
		float angle = i * 2.0f * M_PI / num_nodes;
		pVector.push_back(new Particle(center + Vec2f(cos(angle) * radius, sin(angle) * radius)));
	}
	
	// use springs to form rigid structure
	for (int i = 0; i < num_nodes; i++) {
		int p_idx = i + 1; // start from next prtcl
		
		// spoke
		fVector.push_back(new SpringForce(pCenter, pVector[p_idx], radius, spring_ks * 5.0f, spring_kd));
		
		// connect to every other boundary particle
		for (int j = i + 1; j < num_nodes; j++) {
			int other_idx = j + 1;
			Vec2f dist_vec = pVector[p_idx]->m_ConstructPos - pVector[other_idx]->m_ConstructPos;
			float dist_len = sqrt(dist_vec[0]*dist_vec[0] + dist_vec[1]*dist_vec[1]);
			fVector.push_back(new SpringForce(pVector[p_idx], pVector[other_idx], dist_len, spring_ks * 1.5f, spring_kd));
		}
	}

	// gravity force
	fVector.push_back(new GravityForce(pVector, gravityDirection * gravityStrength));

	// wind force
	windForce = new WindForce(pVector, windDirection, windStrength, enableWind);
	fVector.push_back(windForce);
}

static void bridge_scene() {
	wallVector.clear();

	int num_bridge_nodes = 20;
	float bridge_length = 1.6f; // from -0.8 to 0.8
	float segment_length = bridge_length / (num_bridge_nodes - 1);
	float start_x = -0.8f;
	float y_pos = 0.5f;

	// create particles
	for (int i = 0; i < num_bridge_nodes; ++i) {
		pVector.push_back(new Particle(Vec2f(start_x + i * segment_length, y_pos)));
	}

	// create normal springs
	for (int i = 0; i < num_bridge_nodes - 1; ++i) {
		fVector.push_back(new SpringForce(pVector[i], pVector[i+1], segment_length, spring_ks, spring_kd));
	}

	// create angular springs
	for (int i = 0; i < num_bridge_nodes - 2; ++i) {
		double angular_ks = spring_ks * 0.005; 
		double angular_kd = spring_kd * 0.005;
		if (use_angular_springs_bridge) {
			fVector.push_back(new AngularSpring(pVector[i], pVector[i+1], pVector[i+2], M_PI, angular_ks, angular_kd));
		}
	}

	// anchor the ends
	cVector.push_back(new PointConstraintX(pVector[0], pVector[0]->m_ConstructPos[0]));
	cVector.push_back(new PointConstraintY(pVector[0], pVector[0]->m_ConstructPos[1]));
	
	cVector.push_back(new PointConstraintX(pVector[num_bridge_nodes - 1], pVector[num_bridge_nodes - 1]->m_ConstructPos[0]));
	cVector.push_back(new PointConstraintY(pVector[num_bridge_nodes - 1], pVector[num_bridge_nodes - 1]->m_ConstructPos[1]));

	// gravity force
	fVector.push_back(new GravityForce(pVector, gravityDirection * gravityStrength));

	// wind force
	windForce = new WindForce(pVector, windDirection, windStrength, enableWind);
	fVector.push_back(windForce);
}

static const char* sceneName(int scene)
{
	switch (scene)
	{
		case SCENE_CLOTH: return "cloth";
		case SCENE_HAIR: return "hair";
		case SCENE_PENDULUM: return "pendulum";
		case SCENE_BALL: return "ball";
		case SCENE_BRIDGE: return "bridge";
		case SCENE_FLUID: return "Project 2 fluid";
		default: return "unknown";
	}
}

// Helper keeps the Project 2 delegation checks readable.
static bool inFluidScene()
{
	return scene_type == SCENE_FLUID;
}

static void init_system(void)
{
	free_data();
	if (inFluidScene())
	{
		fluidScene.init();
		return;
	}
	switch (scene_type)
	{
		case SCENE_CLOTH: cloth_scene(); break;
		case SCENE_HAIR: hair_scene(); break;
		case SCENE_PENDULUM: pendulum_scene(); break;
		case SCENE_BALL: ball_scene(); break;
		case SCENE_BRIDGE: bridge_scene(); break;
	}

	for (Force *f : fVector) {
		if (dynamic_cast<GravityForce*>(f)) f->setEnabled(enableGravity);
		if (dynamic_cast<SpringForce*>(f) || dynamic_cast<AngularSpring*>(f)) f->setEnabled(enableSpring);
	}
	for (Constraint *c : cVector) {
		if (dynamic_cast<RodConstraint*>(c)) c->setEnabled(enableRod);
		if (dynamic_cast<CircularWireConstraint*>(c)) c->setEnabled(enableWire);
	}
}

/*
----------------------------------------------------------------------
OpenGL specific drawing routines
----------------------------------------------------------------------
*/


// Screenshot helper used by the old Project 1 frame-dumping shortcut.
// Screenshot support:
// Project 1 already had imageio.cpp/imageio.h for saving PNG frames.  We keep
// the same interface in Project 2 so the demo-video workflow remains familiar.
// The implementation has been made CLion-safe: with the old 32-bit course
// toolchain it can still use libpng, while with CLion's usual 64-bit MinGW it
// writes PNG files through a small built-in fallback and does not need libpng12.

static void pre_display(void)
{
	glViewport(0, 0, win_x, win_y);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	
	if (inFluidScene()) {
		cameraX = 0.0f;
		gluOrtho2D(0.0, 1.0, 0.0, 1.0);
	} else {
		// track ball during its scene
		if (scene_type == SCENE_BALL && !pVector.empty()) {
			cameraX = pVector[0]->m_Position[0]; // track center particle of the ball
		} else {
			cameraX = 0.0f;
		}
		gluOrtho2D(cameraX - 1.0, cameraX + 1.0, -1.0, 1.0);
	}
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
}

static void post_display(void)
{
	// Write frames if necessary.
	if (dump_frames)
	{
		const int FRAME_INTERVAL = 4;
		if ((frame_number % FRAME_INTERVAL) == 0)
		{
			const unsigned int w = glutGet(GLUT_WINDOW_WIDTH);
			const unsigned int h = glutGet(GLUT_WINDOW_HEIGHT);
			unsigned char *buffer = (unsigned char *)malloc(w * h * 4 * sizeof(unsigned char));
			if (!buffer)
				exit(-1);
			// glRasterPos2i(0, 0);
			glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
			static char filename[80];
			sprintf(filename, "snapshots/img%.5i.png", frame_number / FRAME_INTERVAL);
			if (saveImageRGBA(filename, buffer, w, h))
			{
				printf("Dumped %s.\n", filename);
			}
			else
			{
				printf("Could not dump %s. Make sure the snapshots/ folder exists and the working directory is the project root.\n", filename);
			}

			free(buffer);
		}
	}
	frame_number++;

	glutSwapBuffers();
}

static void draw_particles(void)
{
	int size = pVector.size();

	for (int ii = 0; ii < size; ii++)
	{
		pVector[ii]->draw();
	}
}

static void draw_forces(void)
{
	for (size_t i = 0; i < fVector.size(); i++)
	{
		fVector[i]->draw();
	}
}

static void draw_constraints(void)
{
	for (Constraint *c : cVector)
	{
		c->draw();
	}
}

/*
----------------------------------------------------------------------
relates mouse movements to particle toy construction
----------------------------------------------------------------------
*/

static void get_from_UI()
{
	int i, j;
	// int size, flag;
	// int hi, hj;
	// float x, y;
	if (!mouse_down[0] && !mouse_down[2] && !mouse_release[0] && !mouse_shiftclick[0] && !mouse_shiftclick[2])
		return;

	i = (int)((mx / (float)win_x) * N);
	j = (int)(((win_y - my) / (float)win_y) * N);

	if (i < 1 || i > N || j < 1 || j > N)
		return;

	if (mouse_down[0])
	{
	}

	if (mouse_down[2])
	{
	}

	// hi = (int)((hmx / (float)win_x) * N);
	// hj = (int)(((win_y - hmy) / (float)win_y) * N);

	if (mouse_release[0])
	{
	}

	omx = mx;
	omy = my;
}

static void remap_GUI()
{
	int ii, size = pVector.size();
	for (ii = 0; ii < size; ii++)
	{
		pVector[ii]->m_Position[0] = pVector[ii]->m_ConstructPos[0];
		pVector[ii]->m_Position[1] = pVector[ii]->m_ConstructPos[1];
		pVector[ii]->m_Velocity = Vec2f(0.0, 0.0);
		pVector[ii]->m_Force = Vec2f(0.0, 0.0);
	}
}

/*
----------------------------------------------------------------------
GLUT callback routines
----------------------------------------------------------------------
*/

static void key_func(unsigned char key, int x, int y)
{
	if (inFluidScene() && key != 's' && key != 'S' && key != 'q' && key != 'Q' && key != ' ')
	{
		if (fluidScene.handleKey(key)) return;
	}
	switch (key)
	{
	case '1':
		if (scene_type == SCENE_HAIR) {
			printf("Euler is unstable for hair angular springs.\n");
			break;
		}
		solver_type = 0;
		printf("Switched to Euler solver.\n");
		break;
	case '2':
		if (scene_type == SCENE_HAIR) {
			printf("Midpoint is unstable for hair angular springs.\n");
			break;
		}
		solver_type = 1;
		printf("Switched to Midpoint solver.\n");
		break;
	case '3':
		solver_type = 2;
		printf("Switched to RK4 solver.\n");
		break;
	case '4':
		solver_type = 3;
		printf("Switched to Implicit Euler solver.\n");
		break;
	case '5':
		solver_type = 4;
		printf("Switched to Verlet solver.\n");
		break;
	case 'p':
		dt += 0.001f;
		printf("dt: %f\n", dt);
		break;
	case 'o':
		dt -= 0.001f;
		printf("dt: %f\n", dt);
		break;
	case 'c':
	case 'C':
		clear_data();
		break;

	case 'd':
	case 'D':
		dump_frames = !dump_frames;
		break;

	case 'q':
	case 'Q':
		free_data();
		exit(0);
		break;

	case 's':
	case 'S':
		scene_type = (scene_type + 1) % 6;
		init_system();
		printf("Switched to %s scene.\n", sceneName(scene_type));
		if (inFluidScene()) fluidScene.printHelp();
		break;

	case 'r':
		use_sqrt_rodConstraint = !use_sqrt_rodConstraint;
		for (Constraint *c : cVector)
		{
			RodConstraint *rod = dynamic_cast<RodConstraint *>(c);
			if (rod)
			{
				rod->m_useSqrt = use_sqrt_rodConstraint;
			}
		}
		printf("Rod constraint now uses %s.\n", use_sqrt_rodConstraint ? "square root" : "squared distance");
		break;

	case 'g':
		enableGravity = !enableGravity;
		for (Force *f : fVector) {
			if (dynamic_cast<GravityForce*>(f)) f->setEnabled(enableGravity);
		}
		printf("Gravity %s\n", enableGravity ? "enabled" : "disabled");
		break;

	case 't':
		enableSpring = !enableSpring;
		for (Force *f : fVector) {
			if (dynamic_cast<SpringForce*>(f) || dynamic_cast<AngularSpring*>(f)) f->setEnabled(enableSpring);
		}
		printf("Springs %s\n", enableSpring ? "enabled" : "disabled");
		break;

	case 'y':
		enableRod = !enableRod;
		for (Constraint *c : cVector) {
			if (dynamic_cast<RodConstraint*>(c)) c->setEnabled(enableRod);
		}
		printf("Rod Constraint %s\n", enableRod ? "enabled" : "disabled");
		break;

	case 'x':
		enableWire = !enableWire;
		for (Constraint *c : cVector) {
			if (dynamic_cast<CircularWireConstraint*>(c)) c->setEnabled(enableWire);
		}
		printf("Wire Constraint %s\n", enableWire ? "enabled" : "disabled");
		break;
	
	case 'a':
		use_angular_springs_bridge = !use_angular_springs_bridge;
		init_system();
		printf("Angular Springs %s\n", use_angular_springs_bridge ? "enabled" : "disabled");
		break;

	case ' ':
		dsim = !dsim;
		// Project 1 clears dynamic state when returning to construction mode.
		// Project 2 keeps the current smoke/solid configuration visible while paused.
		if (!dsim && !inFluidScene())
			clear_data();
		printf("Simulation %s.\n", dsim ? "running" : "paused");
		break;

	case 'i':
		spring_ks += 0.5;
		printf("spring_ks (spring stiffness): %f\n", spring_ks);
		break;

	case 'u':
		spring_ks -= 0.5;
		if (spring_ks < 0)
			spring_ks = 0;
		printf("spring_ks (spring damping): %f\n", spring_ks);
		break;

	case 'k':
		spring_kd += 0.5;
		printf("spring_kd (damping): %f\n", spring_kd);
		break;

	case 'j':
		spring_kd -= 0.5;
		if (spring_kd < 0)
			spring_kd = 0;
		printf("spring_kd (damping): %f\n", spring_kd);
		break;

	case 'w':
		enableWind = !enableWind;
		if (windForce)
		{
			windForce->setEnabled(enableWind);
		}
		if (enableWind && solver_type == 0) {
			solver_type = 2;
			printf("Wind enabled. Auto-switched to RK4 for stability.\n");
		} else {
			printf("Wind %s\n", enableWind ? "enabled" : "disabled");
		}
		break;

	case 'f':
		fixRowBool = !fixRowBool;
		printf("Row fixing %s\n", fixRowBool ? "enabled" : "disabled");
		init_system(); // rebuild scene to apply changes
		break;

	case 'v':
	{
		int choice;
		printf("\n=== Modify Simulation Parameters ===\n");
		printf("1. Cloth Rows & Columns (current: %d; %d)\n", cloth_rows, cloth_columns);
		printf("2. Spring Stiffness [ks] and Damping [kd] (current: %f; %f)\n", spring_ks, spring_kd);
		printf("3. Cloth Spacing (current: %f)\n", cloth_spacing);
		printf("4. Time Step [dt] (current: %f)\n", dt);
		printf("5. Hair Segments and Length(current: %d; %f)\n", hair_segments, hair_segment_length);
		printf("6. Mouse Spring Stiffness [mouse_ks] and Damping [mouse_kd] (current: %f; %f)\n", mouse_ks, mouse_kd);
		printf("7. Touch Force Radius and Magnitude(current: %f; %f)\n", touchSphereRadius, touchSphereMagnitude);
		printf("Select an option (1-7): ");
		
		std::cin >> choice;

		if (choice == 1) {
			printf("Enter new number of cloth rows: ");
			std::cin >> cloth_rows;
			printf("Enter new number of cloth columns: ");
			std::cin >> cloth_columns;
			if (cloth_columns < 2) cloth_columns = 2;
			if (cloth_rows < 2) cloth_rows = 2;
			init_system(); // rebuilds scene 
			printf("Scene rebuilt with %d rows and %d cols.\n", cloth_rows, cloth_columns);
		} 
		else if (choice == 2) {
			printf("Enter new spring stiffness (ks): ");
			std::cin >> spring_ks;
			printf("Enter new spring damping (kd): ");
			std::cin >> spring_kd;
			init_system(); // rebuilds scene
			printf("Scene rebuilt with ks = %f and kd = %f.\n", spring_ks, spring_kd);
		} 
		else if (choice == 3) {
			printf("Enter new cloth spacing: ");
			std::cin >> cloth_spacing;
			if (cloth_spacing <= 0) cloth_spacing = 0.01f; // prevent non-positive spacing
			init_system(); // rebuilds scene
			printf("Scene rebuilt with cloth spacing = %f.\n", cloth_spacing);
		}
		else if (choice == 4) {
			printf("Enter new time step (dt): ");
			std::cin >> dt;
			printf("Time step updated to dt = %f.\n", dt);
		} 
		else if (choice == 5) {
			printf("Enter new hair segments amount.\n");
			std::cin >> hair_segments;
			if (hair_segments < 2) hair_segments = 2;
			printf("Enter new hair segment length.\n");
			std::cin >> hair_segment_length;
			if (hair_segment_length <= 0) hair_segment_length = 0.01f;
			init_system();
			printf("Scene rebuilt with hair segments = %d and length = %f.\n", hair_segments, hair_segment_length);
		} 
		else if (choice == 6) {
			printf("Enter new mouse spring stiffness (mouse_ks): ");
			std::cin >> mouse_ks;
			printf("Enter new mouse spring damping (mouse_kd): ");
			std::cin >> mouse_kd;
			init_system();
			printf("Scene rebuilt with mouse_ks = %f, and mouse_kd = %f.\n", mouse_ks, mouse_kd);
		}
		else if (choice == 7) {
			printf("Enter new touch force radius: ");
			std::cin >> touchSphereRadius;
			printf("Enter new touch force magnitude: ");
			std::cin >> touchSphereMagnitude;
			init_system();
			printf("Scene rebuilt with touch force radius = %f, and magnitude = %f.\n", touchSphereRadius, touchSphereMagnitude);
		}
		else {
			printf("Invalid selection.\n");
		}
		break;
	}

	case 'h':
		printf("\n=== Keyboard Controls ===\n");
		printf("v         - Open parameter modification menu in console\n");
		printf("1/2/3/4/5 - Switch solver (Euler/Midpoint/RK4/ImplicitEuler/Verlet)\n");
		printf("p/o       - Increase/decrease dt (timestep)\n");
		printf("i/u       - Increase/decrease spring stiffness\n");
		printf("k/j       - Increase/decrease spring damping\n");
		printf("w         - Toggle wind force\n");
		printf("g         - Toggle gravity force\n");
		printf("t         - Toggle spring forces\n");
		printf("y         - Toggle rod constraints\n");
		printf("x         - Toggle circular wire constraints\n");
		printf("s         - Switch between scenes\n");
		printf("r         - Toggle sqrt formula for rod constraints\n");
		printf("c         - Clear/reset simulation\n");
		printf("d         - Toggle frame dumping\n");
		printf("space     - Toggle simulation/construction mode\n");
		printf("q         - Quit\n");
		
		printf("Fluid scene starts by default. Press h in the fluid scene for Project 2 controls.\n");
	printf("Current values:\n");
		printf("  dt: %f\n", dt);
		printf("  spring_ks: %f\n", spring_ks);
		printf("  spring_kd: %f\n", spring_kd);
		printf("========================\n\n");
		break;
	}
}

// Convert screen coordinates to world coordinates in the range [-1, 1] shifted by camera
Vec2f screenToWorld(int x, int y)
{
	if (inFluidScene())
	{
		return Vec2f(x / (float)win_x, 1.0f - (y / (float)win_y));
	}
	float wx = (2.0f * x) / (float)win_x - 1.0f + cameraX;
	float wy = 1.0f - (2.0f * y) / (float)win_y;
	return Vec2f(wx, wy);
}

static void mouse_func(int button, int state, int x, int y)
{
	if (inFluidScene())
	{
		fluidScene.mouse(button, state, x, y, win_x, win_y);
		return;
	}
	Vec2f worldPos = screenToWorld(x, y);

	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
	{
		// find nearest particle
		Particle *nearest = NULL;
		float minDist = 0.5f; // selection threshold
		for (auto *p : pVector)
		{
			float d = sqrt(pow(p->m_Position[0] - worldPos[0], 2) + pow(p->m_Position[1] - worldPos[1], 2));
			if (d < minDist)
			{
				minDist = d;
				nearest = p;
			}
		}
		if (nearest)
		{
			if (scene_type == SCENE_BALL) {
				// push ball if scene ball 
				Vec2f dir = nearest->m_Position - worldPos;
				float dist = sqrt(dir[0]*dir[0] + dir[1]*dir[1]);
				if (dist > 0.001f) {
					Vec2f impulse = (dir / dist) * 2.5f;
					// apply velocity to all particles
					for (auto* p : pVector) {
						p->m_Velocity += impulse;
					}
				}
			} else {
				mouseSpring = new MouseSpringForce(nearest, mouse_ks, mouse_kd);
				mouseSpring->updateMousePosition(worldPos[0], worldPos[1]);
				fVector.push_back(mouseSpring);
			}
		}
	}
	else if (button == GLUT_LEFT_BUTTON && state == GLUT_UP)
	{
		// remove mouse spring
		if (mouseSpring)
		{
			for (auto it = fVector.begin(); it != fVector.end(); ++it)
			{
				if (*it == mouseSpring)
				{
					fVector.erase(it);
					break;
				}
			}
			delete mouseSpring;
			mouseSpring = NULL;
		}
	}
	else if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN) 
	{
		// mouse touch force with radius and max force magnitude
		touchSphereForce = new TouchSphereForce(pVector, touchSphereRadius, touchSphereMagnitude); 
		touchSphereForce->updateMousePosition(worldPos[0], worldPos[1]);
		fVector.push_back(touchSphereForce);
	}
	else if (button == GLUT_RIGHT_BUTTON && state == GLUT_UP) 
	{
		if (touchSphereForce)
		{
			for (auto it = fVector.begin(); it != fVector.end(); ++it)
			{
				if (*it == touchSphereForce)
				{
					fVector.erase(it);
					break;
				}
			}
			delete touchSphereForce;
			touchSphereForce = NULL;
		}
	}
}

static void motion_func(int x, int y)
{
	if (inFluidScene())
	{
		fluidScene.motion(x, y, win_x, win_y);
		return;
	}
	Vec2f worldPos = screenToWorld(x,y);
	if (mouseSpring)
	{
		mouseSpring->updateMousePosition(worldPos[0], worldPos[1]);
	}
	if (touchSphereForce) 
	{
		touchSphereForce->updateMousePosition(worldPos[0], worldPos[1]);
	}
}

static void reshape_func(int width, int height)
{
	glutSetWindow(win_id);
	glutReshapeWindow(width, height);

	win_x = width;
	win_y = height;
}

static void idle_func(void)
{
	if (inFluidScene())
	{
		if (dsim) fluidScene.step(dt);
		glutSetWindow(win_id);
		glutPostRedisplay();
		return;
	}
	if ( dsim ) {
		simulation_step( pVector, fVector, cVector, dt );
		CollisionHandler::handleWallCollisions(pVector, wallVector, wall_restitution, wall_friction_coeff);
		CollisionHandler::handleParticleCollisions(pVector, particle_diameter, wall_restitution);

		if (scene_type == 1) {
			const float hair_drag = 5.0f;
			for (auto* p : pVector) {
				if (!p->m_Pinned)
					p->m_Velocity *= (1.0f - hair_drag * dt);
			}
		}

		bool explosion_detected = false;
		for (auto* p : pVector) {
			if (!std::isfinite(p->m_Position[0]) || !std::isfinite(p->m_Position[1]) ||
			    !std::isfinite(p->m_Velocity[0]) || !std::isfinite(p->m_Velocity[1])) {
				explosion_detected = true;
				break;
			}
			float dx = p->m_Position[0] - p->m_ConstructPos[0];
			float dy = p->m_Position[1] - p->m_ConstructPos[1];
			if (scene_type != SCENE_BALL && dx*dx + dy*dy > 25.0f) { 
				explosion_detected = true;
				break;
			}
		}
		if (explosion_detected) {
			printf("Explosion detected — resetting scene.\n");
			clear_data();
		}
	} else {
		get_from_UI();
		remap_GUI();
	}

	glutSetWindow(win_id);
	glutPostRedisplay();
}

static void display_func(void)
{
	pre_display();

	if (inFluidScene())
	{
		fluidScene.draw();
	}
	else
	{
		draw_forces();
		draw_constraints();
		draw_particles();
		CollisionHandler::drawWalls(wallVector);
	}

	post_display();
}

/*
----------------------------------------------------------------------
open_glut_window --- open a glut compatible window and set callbacks
----------------------------------------------------------------------
*/

static void open_glut_window(void)
{
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);

	glutInitWindowPosition(0, 0);
	glutInitWindowSize(win_x, win_y);
	win_id = glutCreateWindow("2IMV15 Project 2 - Stable Fluids and Coupled Solids");

	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	glutSwapBuffers();
	glClear(GL_COLOR_BUFFER_BIT);
	glutSwapBuffers();

	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);

	pre_display();

	glutKeyboardFunc(key_func);
	glutMouseFunc(mouse_func);
	glutMotionFunc(motion_func);
	glutReshapeFunc(reshape_func);
	glutIdleFunc(idle_func);
	glutDisplayFunc(display_func);
}

/*
----------------------------------------------------------------------
main --- main routine
----------------------------------------------------------------------
*/

int main(int argc, char **argv)
{
	glutInit(&argc, argv);

	if (argc == 1)
	{
		N = 80;
		dt = 0.03f;
		d = 5.f;
		fprintf(stderr, "Using defaults : N=%d dt=%g d=%g\n",
				N, dt, d);
	}
	else
	{
		N = atoi(argv[1]);
		dt = atof(argv[2]);
		d = atof(argv[3]);
	}

	printf("\n\nHow to use this application:\n\n");
	printf("\t Toggle construction/simulation display with the spacebar key\n");
	printf("\t Dump frames by pressing the 'd' key\n");
	printf("\t Quit by pressing the 'q' key\n");

	printf("\n=== Keyboard Controls ===\n");
	printf("v         - Open parameter modification menu in console\n");
	printf("1/2/3/4/5 - Switch solver (Euler/Midpoint/RK4/ImplicitEuler/Verlet)\n");
	printf("p/o       - Increase/decrease dt (timestep)\n");
	printf("i/u       - Increase/decrease spring stiffness\n");
	printf("k/j       - Increase/decrease damping\n");
	printf("s         - Switch between scenes\n");
	printf("r		  - Toggle between sqrt and squared formula for RodConstraint\n");
	printf("g         - Toggle gravity force\n");
	printf("t         - Toggle spring forces\n");
	printf("y         - Toggle rod constraints\n");
	printf("x         - Toggle circular wire constraints\n");
	printf("w         - Toggle wind force\n");
	printf("f         - Toggle fixing top row of cloth\n");
	printf("c         - Clear/reset simulation\n");
	printf("d         - Toggle frame dumping\n");
	printf("space     - Toggle simulation/construction mode\n");
	printf("q         - Quit\n");
	printf("Fluid scene starts by default. Press h in the fluid scene for Project 2 controls.\n");
	printf("Current values:\n");
	printf("  dt: %f\n", dt);
	printf("  spring_ks: %f\n", spring_ks);
	printf("  spring_kd: %f\n", spring_kd);
	printf("========================\n\n");

	dsim = 0;
	dump_frames = 0;
	frame_number = 0;

	init_system();
	if (inFluidScene()) fluidScene.printHelp();

	win_x = 512;
	win_y = 512;
	open_glut_window();

	glutMainLoop();

	exit(0);
}
