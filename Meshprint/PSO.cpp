#include "PSO.h"
double  PSO::calc_inertia_lin_dec(int step) {

	int dec_stage = 3 * settings.steps / 4;
	if (step <= dec_stage)
		return settings.w_min + (settings.w_max - settings.w_min) *	\
		(dec_stage - step) / dec_stage;
	else
		return settings.w_min;
}

double PSO::PSOobjfunction(vector<double> pos)
{
	//the percent of merge points to all points


	return 0;

}

//==============================================================
//          NEIGHBORHOOD (COMM) MATRIX STRATEGIES
//==============================================================
// global neighborhood
void  PSO::inform_global(VVECTORDOUBLE& pos_nb,vector<double> gbest)
{

	int i;
	// all particles have the same attractor (gbest)
	// copy the contents of gbest to pos_nb
	for (i = 0; i < settings.size; i++)
		pos_nb[i] = gbest;

}

int PSO::pso_calc_swarm_size(int dim)
{
	int size = 10. + 2. * sqrt(dim);
	return (size > PSO_MAX_SIZE ? PSO_MAX_SIZE : size);
}

void PSO::pso_set_default_settings()
{  // set some default values
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

	settings.clamp_pos = 0;
	settings.nhood_strategy = PSO_NHOOD_RING;
	settings.nhood_size = 5;
	settings.w_strategy = PSO_W_LIN_DEC;

	settings.rng = NULL;
	settings.seed = time(0);
}

void PSO::pso_solve(pso_obj_fun_t obj_fun, void * obj_fun_params)
{

	int free_rng = 0; // whether to free settings.rng when finished
					  // Particles
	vector<vector<double>> pos(settings.size, vector<double>(settings.dim));// position matrix
	vector<vector<double>> vel(settings.size, vector<double>(settings.dim));// velocity matrix
	vector<vector<double>> pos_b(settings.size, vector<double>(settings.dim));// best position matrix
	vector<double> fit(settings.size); // particle fitness vector
	vector<double> fit_b(settings.size);// best fitness vector
	  // Swarm
	vector<vector<double>> pos_nb(settings.size, vector<double>(settings.dim)); // what is the best informed
												  // position for each particle
	vector<vector<int>> comm(settings.size, vector<int>(settings.size));// communications:who informs who
											  // rows : those who inform
											  // cols : those who are informed
	int improved; // whether solution->error was improved during
				  // the last iteration

	int i, d, step;
	double a, b; // for matrix initialization
	double rho1, rho2; // random numbers (coefficients)
	double w; // current omega
	void(*inform_fun)(); // neighborhood update function
	double(*calc_inertia_fun)(); // inertia weight update function


	// SELECT APPROPRIATE NHOOD UPDATE FUNCTION
	switch (settings.nhood_strategy)
	{
	case PSO_NHOOD_GLOBAL:
		// comm matrix not used
		break;
	case PSO_NHOOD_RING:
		break;
	case PSO_NHOOD_RANDOM:
		break;
	}

	// SELECT APPROPRIATE INERTIA WEIGHT UPDATE FUNCTION
	switch (settings.w_strategy)
	{
		/* case PSO_W_CONST : */
		/*     calc_inertia_fun = calc_inertia_const; */
		/*     break; */
	case PSO_W_LIN_DEC:
		break;
	}

	// INITIALIZE SOLUTION
	solution.error = DBL_MAX;

	// SWARM INITIALIZATION
	// for each particle
	for (i = 0; i < settings.size; i++) {
		// for each dimension
		for (d = 0; d < settings.dim; d++) {
			// generate two numbers within the specified range
			a = settings.x_lo + (settings.x_hi - settings.x_lo) * \
				rand() / double(RAND_MAX);
			b = settings.x_lo + (settings.x_hi - settings.x_lo) *	\
				rand() / double(RAND_MAX);
			// initialize position
			pos[i][d] = a;
			// best position is the same
			pos_b[i][d] = a;
			// initialize velocity
			vel[i][d] = (a - b) / 2.;
		}
		// update particle fitness
		fit[i] = obj_fun(pos[i], settings.dim, obj_fun_params);
		fit_b[i] = fit[i]; // this is also the personal best
						   // update gbest??
		if (fit[i] < solution.error) {
			// update best fitness
			solution.error = fit[i];
			// copy particle pos to gbest vector
			solution.gbest = pos[i];
		}
	}

	// initialize omega using standard value
	w = PSO_INERTIA;
	// RUN ALGORITHM
	for (step = 0; step < settings.steps; step++) {
		// update current step
		settings.step = step;
		// update inertia weight
		// do not bother with calling a calc_w_const function
		w = calc_inertia_lin_dec(step);
	
		// check optimization goal
		if (solution.error <= settings.goal) {
			// SOLVED!!
			if (settings.print_every)
				printf("Goal achieved @ step %d (error=%.3e) :-)\n", step, solution.error);
			break;
		}

		// update pos_nb matrix (find best of neighborhood for all particles)
		inform_global(pos_nb, solution.gbest);
		// the value of improved was just used; reset it
		improved = 0;

		// update all particles
		for (i = 0; i < settings.size; i++) {
			// for each dimension
			for (d = 0; d < settings.dim; d++) {
				// calculate stochastic coefficients
				rho1 = settings.c1 * rand() / double(RAND_MAX);
				rho2 = settings.c2 *rand() / double(RAND_MAX);
				// update velocity
				vel[i][d] = w * vel[i][d] + \
					rho1 * (pos_b[i][d] - pos[i][d]) + \
					rho2 * (pos_nb[i][d] - pos[i][d]);
				// update position
				pos[i][d] += vel[i][d];
			}

			// update particle fitness
			fit[i] = PSOobjfunction(pos[i]);
			// update personal best position?
			if (fit[i] < fit_b[i]) {
				fit_b[i] = fit[i];
				// copy contents of pos[i] to pos_b[i]
				pos_b[i] = pos[i];
			}
			// update gbest??
			if (fit[i] < solution.error) {
				improved = 1;
				// update best fitness
				solution.error = fit[i];
				// copy particle pos to gbest vector
				solution.gbest = pos[i];
			}
		}
		if (settings.print_every && (step % settings.print_every == 0))
			printf("Step %d (w=%.2f) :: min err=%.5e\n", step, w, solution.error);
	}
}

PSO::PSO()
{
}

PSO::~PSO()
{
}
