#include "PSO.h"
#include <math.h>
#include <ctime>
#include <stdlib.h>

PSO::PSO()
{
	pso_set_default_settings();
	
}

PSO::PSO(std::vector<std::pair<int,int>> original_bird_, ClipperLib::Paths polygon,std::pair<int,int>dense,int size)
{
	pso_set_default_settings();
	settings.dim = original_bird_.size();
	settings.size = size;
	remain_paths_ = polygon;
	dense_ = dense;
	pso_swarm_init(original_bird_);
	
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


void PSO::pso_swarm_init(std::vector<std::pair<int, int>> first_particle_)
{
	// SWARM INITIALIZATION
	srand((unsigned)time(NULL));
	swarm.resize(settings.size);
	for (int i=0;i<settings.size;i++)
	{
		swarm[i].pos.resize(settings.dim);
		swarm[i].pos_b.resize(settings.dim);
		swarm[i].pos_nb.reserve(settings.dim);
	}
	double useless_area = 1e20;
	for (int i = 0; i < settings.size; i++)
	{
		for (int j = 0; j < settings.dim; j++)
		{
			swarm[i].pos[j].first = first_particle_[j].first + rand() % 2001-1000;
			swarm[i].pos[j].second = first_particle_[j].second +rand() % 2001-1000;
		}
		double area = pso_obj_fun_t(swarm[i]);
		if (area<useless_area)
		{
			useless_area = area;
			gbest = swarm[i].pos;
		}
	}
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

	settings.clamp_pos = 1;
	settings.nhood_strategy = PSO_NHOOD_RING;
	settings.nhood_size = 5;
	settings.w_strategy = PSO_W_LIN_DEC;
}

double PSO::calc_inertia_lin_dec(int step)
{
	return 0.0;
}



std::vector<std::pair<int, int>> PSO::pso_solve()
{
	return gbest;

}

double PSO::pso_obj_fun_t(particle bird)
{
	//calculate the non-overlap area
	using namespace ClipperLib; 
	Path rectangle_;
	Clipper solver;
	Paths useless_paths_;
	double useless_area_value_ = {0.0};
	for (int i=0;i<settings.dim;i++)
	{
		solver.Clear();
		rectangle_.clear();
		rectangle_ << IntPoint(bird.pos[i].first - dense_.first / 2, bird.pos[i].second - dense_.second / 2);
		rectangle_ << IntPoint(bird.pos[i].first + dense_.first / 2, bird.pos[i].second - dense_.second / 2);
		rectangle_ << IntPoint(bird.pos[i].first + dense_.first / 2, bird.pos[i].second + dense_.second / 2);
		rectangle_ << IntPoint(bird.pos[i].first - dense_.first / 2, bird.pos[i].second + dense_.second / 2);
		solver.AddPath(rectangle_,ptSubject,true);
		solver.AddPaths(remain_paths_, ptClip, true);
		solver.Execute(ctDifference, useless_paths_, pftNonZero, pftNonZero);
		solver.Clear();
		solver.AddPaths(remain_paths_, ptSubject, true);
		solver.AddPath(rectangle_, ptClip, true);
		solver.Execute(ctDifference, remain_paths_, pftNonZero, pftNonZero);
		solver.Clear();
		for (int j=0;j<useless_paths_.size();j++)
			useless_area_value_ = +Area(useless_paths_[j]);	
	}
	if (remain_paths_.size())
		return 1e20;

	return useless_area_value_;
}

//==============================================================
// calculate swarm size based on dimensionality
int PSO:: pso_calc_swarm_size(int dim) {
	int size = 10. + 2. * sqrt(dim);
	return (size > PSO_MAX_SIZE ? PSO_MAX_SIZE : size);
}