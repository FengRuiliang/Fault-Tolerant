#include "PSO.h"
#include <math.h>
#include <ctime>
#include <stdlib.h>
#include <qdebug.h>

PSO::PSO()
{
	pso_set_default_settings();
	settings.dim = 2;
	solution.error = MAX_FITNESS;
	solution.fit_b = MAX_FITNESS;

}

PSO::PSO(std::vector<IntPoint> original_bird_, ClipperLib::Paths polygon, IntPoint dense, int size)
{
	pso_set_default_settings();
	settings.dim = 2;
	settings.size = size;
	remain_paths_ = polygon;
	dense_ = dense;
	original = original_bird_;
	solution.error = MAX_FITNESS;
	solution.fit_b = MAX_FITNESS;
}


PSO::~PSO()
{
}

void PSO::solver_init()
{

	// CHECK RANDOM NUMBER GENERATOR
	srand((unsigned)time(NULL));
	improved = false;
	// INITIALIZE SOLUTION
	solution.error = DBL_MAX;
}


void PSO::pso_swarm_init()
{
	// SWARM INITIALIZATION
	srand((unsigned)time(NULL));
	swarm.resize(settings.size);
	qDebug() << "start initialize swarm";
	for (int i = 0; i < settings.size; i++)
	{


		swarm[i].pos.first = rand() % 101 - 50;
		swarm[i].pos.second = rand() % 101 - 50;
		swarm[i].fit = pso_obj_fun_t(swarm[i]);

		if (swarm[i].fit < solution.fit_b)
		{
			solution.fit_b = swarm[i].fit;
			solution.gbest = swarm[i].pos;
		}
		swarm[i].vel.first = swarm[i].pos.first;
		swarm[i].vel.second = swarm[i].pos.second;

		swarm[i].fit_b = swarm[i].fit;
		swarm[i].pos_b = swarm[i].pos;
		swarm[i].pos_nb = solution.gbest;
		qDebug() << i << swarm[i].fit;
	}
	qDebug() << "end initialize swarm";

}



void PSO::inform_global()
{
}

void PSO::init_comm_ring()
{
}

void PSO::inform_ring()
{
}

void PSO::inform()
{
}

void PSO::inform_random()
{
}

void PSO::init_comm_random()
{
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

	settings.nhood_strategy = PSO_NHOOD_RING;
	settings.nhood_size = 5;
	settings.w_strategy = PSO_W_LIN_DEC;
}
//==============================================================
//          INERTIA WEIGHT UPDATE STRATEGIES
//==============================================================
// calculate linearly decreasing inertia weight
double PSO::calc_inertia_lin_dec(int step)
{
	int dec_stage = 3 * settings.steps / 4;
	if (step <= dec_stage)
		return settings.w_min + (settings.w_max - settings.w_min) *
		(dec_stage - step) / dec_stage;
	else
		return settings.w_min;
}



std::vector<std::pair<int, int>> PSO::pso_solve()
{
	qDebug() << "star pso solution";
	// initialize omega using standard value
	double  w = PSO_INERTIA;// current omega
							// RUN ALGORITHM

	for (int step = 0; step < settings.steps; step++)
	{
		// update current step
		settings.step = step;
		// update inertia weight
		// do not bother with calling a calc_w_const function
		w = calc_inertia_lin_dec(step);

		// update pos_nb matrix (find best of neighborhood for all particles)


		improved = 0;	// the value of improved was just used; reset it
		double rho1, rho2; // random numbers (coefficients)
		// update all particles
		for (int i = 0; i < settings.size; i++)
		{

			//qDebug() << "the" << step << "th iteration";
			srand((unsigned)time(NULL));
			// for each dimension
			// calculate stochastic coefficients
			rho1 = settings.c1 * rand() / RAND_MAX;
			rho2 = settings.c2 * rand() / RAND_MAX;
			// update velocity
			swarm[i].vel.first = w*swarm[i].vel.first
				+ rho1*(swarm[i].pos_b.first - swarm[i].pos.first)
				+ rho2*(swarm[i].pos_nb.first - swarm[i].pos.first);
			swarm[i].vel.second = w*swarm[i].vel.second
				+ rho1*(swarm[i].pos_b.second - swarm[i].pos.second)
				+ rho2*(swarm[i].pos_nb.second - swarm[i].pos.second);
			// update position
			swarm[i].pos.first += swarm[i].vel.first;
			swarm[i].pos.second += swarm[i].vel.second;
			// update particle fitness
			swarm[i].fit = pso_obj_fun_t(swarm[i]);
			// update personal best position?
			if (swarm[i].fit < swarm[i].fit_b) {
				swarm[i].fit_b = swarm[i].fit;
				// copy contents of pos[i] to pos_b[i]
				swarm[i].pos_b = swarm[i].pos;
			}
			// update gbest??

			if (swarm[i].fit < solution.fit_b) //more large more better
			{
				improved = 1;
				// update best fitness
				solution.fit_b = swarm[i].fit;
				// copy particle pos to gbest vector
				solution.gbest = swarm[i].pos;
			}

		}
		qDebug() << step << solution.fit_b;

	}
	std::vector<pair<int, int>> resualt;
	IntPoint p;
	Path rec(4);
	Paths sol;
	Clipper solver;

	for (p.X = settings.clamp_pos[1].X+solution.gbest.first; p.X < settings.clamp_pos[0].X; p.X += dense_.X)
	{
		for (p.Y = settings.clamp_pos[1].Y+ solution.gbest.second; p.Y < settings.clamp_pos[0].Y; p.Y += dense_.Y)
		{

			for (int k = 0; k < remain_paths_.size(); k++)
			{
				if (PointInPolygon(p, remain_paths_[k]) == 0)
				{
					rec[0].X = p.X - dense_.X / 2;
					rec[0].Y = p.Y - dense_.Y / 2;
					rec[1].X = p.X + dense_.X / 2;
					rec[1].Y = p.Y - dense_.Y / 2;
					rec[2].X = p.X + dense_.X / 2;
					rec[2].Y = p.Y + dense_.Y / 2;
					rec[3].X = p.X - dense_.X / 2;
					rec[3].Y = p.Y + dense_.Y / 2;
					solver.Clear();
					solver.AddPaths(remain_paths_, ptClip, true);
					solver.AddPath(rec, ptSubject, true);
					solver.Execute(ctIntersection, sol, pftNonZero, pftNonZero);
					if (sol.size())
					{

						resualt.push_back(make_pair(p.X, p.Y));
					}
				}
				else
				{
					resualt.push_back(make_pair(p.X, p.Y));
				}
			}
		}

	}
	qDebug() << "end run solver and we have" << resualt.size() << "points";
	return resualt;
}

double PSO::pso_obj_fun_t(particle bird)
{
	//calculate the non-overlap area
	IntPoint p;
	Path rec(4);
	Paths solution;
	Clipper solver;
	int fitness = 0;
	
	
	for (p.X = settings.clamp_pos[1].X + bird.pos.first; p.X < settings.clamp_pos[0].X; p.X += dense_.X)
	{
		for (p.Y = settings.clamp_pos[1].Y + bird.pos.second; p.Y <settings.clamp_pos[0].Y; p.Y += dense_.Y)
		{

			for (int k=0;k<remain_paths_.size();k++)
			{
				if (PointInPolygon(p, remain_paths_[k]) == 0)
				{
					rec[0].X = p.X - dense_.X / 2;
					rec[0].Y = p.Y - dense_.Y / 2;
					rec[1].X = p.X + dense_.X / 2;
					rec[1].Y = p.Y - dense_.Y / 2;
					rec[2].X = p.X + dense_.X / 2;
					rec[2].Y = p.Y + dense_.Y / 2;
					rec[3].X = p.X - dense_.X / 2;
					rec[3].Y = p.Y + dense_.Y / 2;
					solver.Clear();
					solver.AddPaths(remain_paths_, ptClip, true);
					solver.AddPath(rec, ptSubject, true);
					solver.Execute(ctIntersection, solution, pftNonZero, pftNonZero);
					if (solution.size())
					{

						fitness++;
					}
				}
				else
				{
					fitness++;
				}
			}
		}

	}
	return fitness;
}

//==============================================================
// calculate swarm size based on dimensionality
int PSO::pso_calc_swarm_size(int dim) {
	int size = 10. + 2. * sqrt(dim);
	return (size > PSO_MAX_SIZE ? PSO_MAX_SIZE : size);
}