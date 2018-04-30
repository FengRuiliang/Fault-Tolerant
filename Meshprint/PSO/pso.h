#pragma once
#include <vector>
#include "Library/clipper.hpp"
#include "Library/clipper.hpp"
using namespace std;
	// CONSTANTS
#define PSO_MAX_SIZE 100 // max swarm size
#define PSO_INERTIA 0.7298 // default value of w (see clerc02)

#define MAX_FITNESS 1000000
	// === NEIGHBORHOOD SCHEMES ===

	// global best topology
#define PSO_NHOOD_GLOBAL 0

	// ring topology
#define PSO_NHOOD_RING 1

	// Random neighborhood topology
	// **see http://clerc.maurice.free.fr/pso/random_topology.pdf**
#define PSO_NHOOD_RANDOM 2



	// === INERTIA WEIGHT UPDATE FUNCTIONS ===
#define PSO_W_CONST 0
#define PSO_W_LIN_DEC 1

using namespace ClipperLib;
class PSO
{
public:
	
	PSO();
	PSO(std::vector<IntPoint> original_bird_, ClipperLib::Paths polygon, IntPoint dense, int size);
	~PSO();
	void solver_init();
public:
	// PSO SETTINGS
	struct pso_settings_t{

		int dim; // problem dimensionality
		double x_lo; // lower range limit
		double x_hi; // higher range limit
		double goal; // optimization goal (error threshold)

		int size; // swarm size (number of particles)
		int print_every; // ... N steps (set to 0 for no output)
		int steps; // maximum number of iterations
		int step; // current PSO step
		double c1; // cognitive coefficient
		double c2; // social coefficient
		double w_max; // max inertia weight value
		double w_min; // min inertia weight value

		IntPoint clamp_pos[2]; // whether to keep particle position within defined bounds (TRUE)
					   // or apply periodic boundary conditions (FALSE)
		int nhood_strategy; // neighborhood strategy (see PSO_NHOOD_*)
		int nhood_size; // neighborhood size
		int w_strategy; // inertia weight strategy (see PSO_W_*)

		//gsl_rng *rng; // pointer to random number generator (use NULL to create a new RNG)
		//long seed; // seed for the generator

	}settings;
	// PSO SOLUTION -- Initialized by the user
	struct pso_result_t {
		int error, error_last;
		double fit_b;
		std::pair<int, int> gbest; // should contain DIM elements!!
		std::vector<int> isavailable;
	}solution;
	// Particles
	struct particle {
		std::pair<int, int> pos;// position matrix
		std::pair<int, int> 	vel;// velocity matrix
		std::pair<int, int>pos_b;// best position matrix
		std::pair<int, int>pos_nb;	// what is the best informed position for each particle

		double fit, fit_b;		// particle fitness 
								// best fitness 

	};
	//Swarm

		std::vector<std::vector<int>> comm;	// communications:who informs who
											// rows : those who inform
											// cols : those who are informed
		std::vector<particle> swarm;
		std::vector < int> inform_;
	

	int improved; // whether solution->error was improved during
				 // the last iteration

	void pso_swarm_init();
public:
	void inform_global();
	void init_comm_ring();
	void inform_ring();
	void inform();
	void inform_random();
	void init_comm_random();
	void pso_set_default_settings();
	double calc_inertia_lin_dec(int step);
	int pso_calc_swarm_size(int dim);
	std::vector<std::pair<int, int>> pso_solve();
public:
	double pso_obj_fun_t(particle bird);
	ClipperLib::Paths remain_paths_,grid_paths_;
	IntPoint dense_;
	std::vector<IntPoint> original;
};

