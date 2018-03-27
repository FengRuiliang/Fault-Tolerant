#include "PSO.h"
#include <math.h>
#include <ctime>
#include <stdlib.h>

PSO::PSO()
{
	pso_set_default_settings();
	init();
}


PSO::~PSO()
{
}

void PSO::init()
{

	// CHECK RANDOM NUMBER GENERATOR
	srand((unsigned)time(NULL));
	improved = false;
	// INITIALIZE SOLUTION
	solution.error = DBL_MAX;

	pso_swarm_init();

}

void PSO::pso_swarm_init()
{
	// SWARM INITIALIZATION
	// for each particle

}

//==============================================================
//          NEIGHBORHOOD (COMM) MATRIX STRATEGIES
//==============================================================
// global neighborhood
void PSO::inform_global()
{
// all particles have the same attractor (gbest)
	// copy the contents of gbest to pos_nb
	for (int i=0;i<population.birds.size;i++)
	{

		population.birds[i].pos_nb = p.gbest;

	}
}
// topology initialization :: this is a static (i.e. fixed) topology
void PSO::init_comm_ring() {
	int i;
	// reset array
	for (int i = 0; i < population.comm.size(); i++)
	{
		for (int j = 0; j < population.comm[i].size(); j++)
		{
			population.comm[i][j] = 0;
		}
	}

	// choose informers
	for (i = 0; i < settings.size; i++) {
		// set diagonal to 1
		population.comm[i][i] = 1;
		if (i == 0) {
			// look right
			population.comm[i][i + 1] = 1;
			// look left
			population.comm[i][settings.size - 1] = 1;
		}
		else if (i == settings.size - 1) {
			// look right
			population.comm[i][0] = 1;
			// look left
			population.comm[i][i - 1] = 1;
		}
		else {
			// look right
			population.comm[i][i + 1] = 1;
			// look left
			population.comm[i][i - 1] = 1;
		}

	}

}

void PSO::inform_ring()
{
	// update pos_nb matrix
	inform();

}
// ===============================================================
// general inform function :: according to the connectivity
// matrix COMM, it copies the best position (from pos_b) of the
// informers of each particle to the pos_nb matrix
void PSO::inform()
{
	int i, j;
	int b_n; // best neighbor in terms of fitness

			 // for each particle
	for (j = 0; j < settings.size; j++) {
		b_n = j; // self is best
				 // who is the best informer??
		for (i = 0; i < settings.size; i++)
			// the i^th particle informs the j^th particle
			if (population.comm[i][j] && population.birds[i].fit_b < population.birds[b_n].fit_b)
				// found a better informer for j^th particle
				b_n = i;
		// copy pos_b of b_n^th particle to pos_nb[j]
		population.birds[j].pos_nb = population.birds[b_n].pos_b;
	}
}

void PSO::inform_random()
{

	// regenerate connectivity??
	if (!improved)
		init_comm_random();
	inform();

}

// ============================
// random neighborhood topology
// ============================
void PSO::init_comm_random() {

	int i, j, k;
	// reset array
	for (i = 0; i < population.comm.size(); i++)
	{
		for (j = 0; j < population.comm[i].size(); j++)
		{
			population.comm[i][j] = 0;
		}
	}

	// choose informers
	for (i = 0; i < settings.size; i++) {
		// each particle informs itself
		population.comm[i][i] = 1;
		// choose kappa (on average) informers for each particle
		for (k = 0; k < settings.nhood_size; k++) {
			// generate a random index
			j = rand()%settings.size;
			// particle i informs particle j
			population.comm[i][j] = 1;
		}
	}
}


void PSO::pso_set_default_settings()
{

	// set some default values
	settings.dim = 30;
	settings.x_lo = -20;
	settings.x_hi = 20;
	settings.goal = 1e-5;

	settings.size = pso_calc_swarm_size(settings.dim);
	settings.print_every = 1000;
	settings.steps = 100000;
	settings.c1 = 1.496;
	settings.c2 = 1.496;
	settings.w_max = PSO_INERTIA;
	settings.w_min = 0.3;

	settings.clamp_pos = 1;
	settings.nhood_strategy = PSO_NHOOD_RING;
	settings.nhood_size = 5;
	settings.w_strategy = PSO_W_LIN_DEC;
}

//==============================================================
//          INERTIA WEIGHT UPDATE STRATEGIES
//==============================================================
// calculate linearly decreasing inertia weight
double PSO::calc_inertia_lin_dec(int step) {

	int dec_stage = 3 * settings.steps / 4;
	if (step <= dec_stage)
		return settings.w_min + (settings.w_max - settings.w_min) *
		(dec_stage - step) / dec_stage;
	else
		return settings.w_min;
}

void PSO::pso_solve()
{
	int improved; // whether solution->error was improved during
				  // the last iteration

	int i, d, step;
	double a, b; // for matrix initialization
	double rho1, rho2; // random numbers (coefficients)
	double w; // current omega
	void(PSO::*inform_fun)();
	double(PSO::*calc_inertia_fun)(int); // inertia weight update function


	// SELECT APPROPRIATE NHOOD UPDATE FUNCTION
	switch (settings.nhood_strategy)
	{
	case PSO_NHOOD_GLOBAL:
		// comm matrix not used
		inform_fun = inform_global;
		break;
	case PSO_NHOOD_RING:
		init_comm_ring();
		inform_fun = inform_ring;
		break;
	case PSO_NHOOD_RANDOM:
		init_comm_random();
		inform_fun = inform_random;
		break;
	}

	// SELECT APPROPRIATE INERTIA WEIGHT UPDATE FUNCTION
	switch (settings.w_strategy)
	{
		/* case PSO_W_CONST : */
		/*     calc_inertia_fun = calc_inertia_const; */
		/*     break; */
	case PSO_W_LIN_DEC:
		calc_inertia_fun = calc_inertia_lin_dec;
		break;
	}



	// initialize omega using standard value
	w = PSO_INERTIA;
	// RUN ALGORITHM
	for (step = 0; step < settings.steps; step++) {
		// update current step
		settings.step = step;
		// update inertia weight
		// do not bother with calling a calc_w_const function
		if (settings.w_strategy)
			w = calc_inertia_fun(step);
		calc_inertia_fun(step);
		// check optimization goal
		if (solution.error <= settings.goal) {
			// SOLVED!!
			if (settings.print_every)
				printf("Goal achieved @ step %d (error=%.3e) :-)\n", step, solution.error);
			break;
		}

		// update pos_nb matrix (find best of neighborhood for all particles)
		inform_fun(comm, pos_nb, pos_b, fit_b, solution->gbest,
			improved, settings);
		// the value of improved was just used; reset it
		improved = 0;

		// update all particles
		for (i = 0; i < settings->size; i++) {
			// for each dimension
			for (d = 0; d < settings->dim; d++) {
				// calculate stochastic coefficients
				rho1 = settings->c1 * gsl_rng_uniform(settings->rng);
				rho2 = settings->c2 * gsl_rng_uniform(settings->rng);
				// update velocity
				vel[i][d] = w * vel[i][d] + \
					rho1 * (pos_b[i][d] - pos[i][d]) + \
					rho2 * (pos_nb[i][d] - pos[i][d]);
				// update position
				pos[i][d] += vel[i][d];
				// clamp position within bounds?
				if (settings->clamp_pos) {
					if (pos[i][d] < settings->x_lo) {
						pos[i][d] = settings->x_lo;
						vel[i][d] = 0;
					}
					else if (pos[i][d] > settings->x_hi) {
						pos[i][d] = settings->x_hi;
						vel[i][d] = 0;
					}
				}
				else {
					// enforce periodic boundary conditions
					if (pos[i][d] < settings->x_lo) {

						pos[i][d] = settings->x_hi - fmod(settings->x_lo - pos[i][d],
							settings->x_hi - settings->x_lo);
						vel[i][d] = 0;

					}
					else if (pos[i][d] > settings->x_hi) {

						pos[i][d] = settings->x_lo + fmod(pos[i][d] - settings->x_hi,
							settings->x_hi - settings->x_lo);
						vel[i][d] = 0;
					}
				}

			}

			// update particle fitness
			fit[i] = obj_fun(pos[i], settings->dim, obj_fun_params);
			// update personal best position?
			if (fit[i] < fit_b[i]) {
				fit_b[i] = fit[i];
				// copy contents of pos[i] to pos_b[i]
				memmove((void *)&pos_b[i], (void *)&pos[i],
					sizeof(double) * settings->dim);
			}
			// update gbest??
			if (fit[i] < solution->error) {
				improved = 1;
				// update best fitness
				solution->error = fit[i];
				// copy particle pos to gbest vector
				memmove((void *)solution->gbest, (void *)&pos[i],
					sizeof(double) * settings->dim);
			}
		}

		if (settings->print_every && (step % settings->print_every == 0))
			printf("Step %d (w=%.2f) :: min err=%.5e\n", step, w, solution->error);

	}

	// free RNG??
	if (free_rng)
		gsl_rng_free(settings->rng);
}

//==============================================================
// calculate swarm size based on dimensionality
int PSO:: pso_calc_swarm_size(int dim) {
	int size = 10. + 2. * sqrt(dim);
	return (size > PSO_MAX_SIZE ? PSO_MAX_SIZE : size);
}