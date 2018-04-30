#include "PSO.h"
#include <math.h>
#include <ctime>
#include <stdlib.h>
#include <qdebug.h>

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

	solution.error = MIN_FITNESS;
	solution.fit_b = MIN_FITNESS;
	solution.gbest.clear();
	solution.gbest.resize(settings.dim);
	solution.isavailable.clear();
	solution.isavailable.resize(settings.dim);
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
	qDebug() << "uniform num" << first_particle_.size();
	srand((unsigned)time(NULL));
	swarm.resize(settings.size);
	inform_.resize(settings.dim);
	qDebug() << "start initialize swarm";
	for (int i = 0; i < settings.size; i++)
	{
		swarm[i].pos.resize(settings.dim);
		swarm[i].vel.resize(settings.dim);
		do 
		{
			for (int j = 0; j < settings.dim; j++)
			{
				swarm[i].pos[j].first = first_particle_[j].first + rand() % 501 - 250;
				swarm[i].pos[j].second = first_particle_[j].second + rand() % 501 - 250;
				
			}
			swarm[i].fit = pso_obj_fun_t(swarm[i]);
		} while (swarm[i].fit == MIN_FITNESS); 
		if (swarm[i].fit > solution.fit_b)
		{
			solution.fit_b = swarm[i].fit;
			solution.gbest = swarm[i].pos;
			solution.isavailable = inform_;
		}
		for (int j=0;j<settings.dim;j++)
		{
			swarm[i].vel[j].first = swarm[i].pos[j].first - first_particle_[j].first;
			swarm[i].vel[j].second = swarm[i].pos[j].second - first_particle_[j].second;
		}
		
		swarm[i].fit_b = swarm[i].fit;
		swarm[i].pos_b = swarm[i].pos;
		swarm[i].pos_nb = solution.gbest;
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

	settings.clamp_pos = 1;
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
		for (int i=0;i<settings.size;i++)
		{

			//qDebug() << "the" << step << "th iteration";
			srand((unsigned)time(NULL));
			// for each dimension
			for (int d=0;d<settings.dim;d++)
			{
				// calculate stochastic coefficients
				rho1 = settings.c1 * rand()/RAND_MAX;
				rho2 = settings.c2 * rand()/RAND_MAX;
				// update velocity
				swarm[i].vel[d].first = w*swarm[i].vel[d].first 
					+ rho1*(swarm[i].pos_b[d].first-swarm[i].pos[d].first)
					+ rho2*(swarm[i].pos_nb[d].first - swarm[i].pos[d].first);
				swarm[i].vel[d].second = w*swarm[i].vel[d].second
					+ rho1*(swarm[i].pos_b[d].second - swarm[i].pos[d].second)
					+ rho2*(swarm[i].pos_nb[d].second - swarm[i].pos[d].second);
				// update position
			swarm[i].pos[d].first += swarm[i].vel[d].first;
			swarm[i].pos[d].second += swarm[i].vel[d].second;
			}
			// update particle fitness
			swarm[i].fit = pso_obj_fun_t(swarm[i]);
			// update personal best position?
			if (swarm[i].fit < swarm[i].fit_b) {
				swarm[i].fit_b = swarm[i].fit;
				// copy contents of pos[i] to pos_b[i]
				swarm[i].pos_b = swarm[i].pos;
			}
			// update gbest??

			if (swarm[i].fit > solution.fit_b) //more large more better
			{
				improved = 1;
				// update best fitness
				solution.fit_b = swarm[i].fit;
				// copy particle pos to gbest vector
				solution.gbest = swarm[i].pos;
				solution.isavailable = inform_;
				qDebug() << step<<i<<"solution is improved" << solution.fit_b;
			}
			int er = 0;
			for (int j = 0; j < solution.isavailable.size(); j++)
				er += solution.isavailable[j];
			if (er < settings.goal)
			{
				qDebug() << "we success and the answer is"<<er;
				break;
			}
// 			else
// 			{
// 				if (er == solution.error_last)
// 				{
// 					if (solution.error++ > 100)
// 					{
// 						step = settings.steps;
// 						qDebug() << "we have try our best,the final resualt is " << er;
// 					
// 					}
// 				}
// 				else
// 					solution.error_last = er;
// 
// 			}
		}
		
		
	}
	std::vector<pair<int, int>> resualt;
	for (int i=0;i<solution.isavailable.size();i++)
	{
		if (solution.isavailable[i])
		{
			resualt.push_back(solution.gbest[i]);
		}
	}
	qDebug() << "end run solver and we have"<< resualt.size()<<"points";
	return resualt;

}

double PSO::pso_obj_fun_t(particle bird)
{
	//calculate the non-overlap area
	using namespace ClipperLib; 
	Paths target ;
	Path rectangle_;
	Clipper solver;
	solver.AddPaths(remain_paths_, ptSubject, true);
	for (int i=0;i<settings.dim;i++)
	{
		solver.Clear();
		rectangle_.clear();
		rectangle_ << IntPoint(bird.pos[i].first - dense_.first / 2, bird.pos[i].second - dense_.second / 2);
		rectangle_ << IntPoint(bird.pos[i].first + dense_.first / 2, bird.pos[i].second - dense_.second / 2);
		rectangle_ << IntPoint(bird.pos[i].first + dense_.first / 2, bird.pos[i].second + dense_.second / 2);
		rectangle_ << IntPoint(bird.pos[i].first - dense_.first / 2, bird.pos[i].second + dense_.second / 2);
		solver.AddPath(rectangle_, ptClip, true);
	}
	solver.Execute(ctDifference, target, pftNonZero, pftNonZero);
	if (target.size())
	{
		return MIN_FITNESS;
	}
	target = remain_paths_;
	Paths isec_path;
	double useless_area_value_ = { 0.0 };
	double area_rec_ = dense_.first*dense_.second;

	Paths  clip;
	solver.AddPaths(remain_paths_, ptClip, true);
	rectangle_.resize(4);
	for (int i=0;i<settings.dim;i++)
	{
		rectangle_[0]= IntPoint(bird.pos[i].first - dense_.first / 2, bird.pos[i].second - dense_.second / 2);
		rectangle_[1] = IntPoint(bird.pos[i].first + dense_.first / 2, bird.pos[i].second - dense_.second / 2);
		rectangle_[2] = IntPoint(bird.pos[i].first + dense_.first / 2, bird.pos[i].second + dense_.second / 2);
		rectangle_[3] = IntPoint(bird.pos[i].first - dense_.first / 2, bird.pos[i].second + dense_.second / 2);
		solver.Clear();
		solver.AddPath(rectangle_, ptSubject, true);
		solver.AddPaths(clip, ptClip, true);
		solver.Execute(ctIntersection, isec_path, pftNonZero, pftNonZero);//相交面积越大越好
		solver.Execute(ctUnion, clip, pftNonZero, pftNonZero);//将并区域作为下一次的clipper
		
		
		double t_area_ = 0.0;
		for (int j = 0; j < isec_path.size(); j++)
		{
			useless_area_value_ +=Area(isec_path[j]);
			t_area_ += Area(isec_path[j]);
		}
		if (t_area_-area_rec_==0)
		{
			inform_[i] = 0;
		}
		else
		{
			inform_[i] = 1;
		}
	}
	int error = 0;
	for (int i = 0; i < inform_.size(); i++)
	{

		error += inform_[i];
	}
		return useless_area_value_+pow(error,2.0)*area_rec_;//越大越好

}

//==============================================================
// calculate swarm size based on dimensionality
int PSO:: pso_calc_swarm_size(int dim) {
	int size = 10. + 2. * sqrt(dim);
	return (size > PSO_MAX_SIZE ? PSO_MAX_SIZE : size);
}