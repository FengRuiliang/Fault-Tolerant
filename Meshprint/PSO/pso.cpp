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
	dense = dense;
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
		for (int m=0;m<meshs_.size();m++)
		{
			std::vector < std:: pair<int, int> > m_degree_center;
			for (int n=0;n<meshs_[m].size();n++)
			{
				m_degree_center.push_back(make_pair(rand() % 101 - 50, rand() % 101 - 50));
			}
			swarm[i].pos.insert(swarm[i].pos.end(), m_degree_center.begin(), m_degree_center.end());
		}

		
		swarm[i].fit = pso_obj_fun_t(swarm[i]);

		if (swarm[i].fit < solution.fit_b)
		{
			solution.fit_b = swarm[i].fit;
			solution.gbest = swarm[i].pos;
		}
		swarm[i].vel = swarm[i].pos;
		swarm[i].vel = swarm[i].pos;

		swarm[i].fit_b = swarm[i].fit;
		swarm[i].pos_b = swarm[i].pos;
		swarm[i].pos_nb = solution.gbest;
		qDebug() << i << swarm[i].fit;
	}
	qDebug() << "end initialize swarm"<<"best fitness is "<<solution.fit_b;

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



std::vector<Vec3f> PSO::pso_solve()
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
			
			for (int j=0;j<swarm[i].pos.size();j++)
			{
				// update velocity
				swarm[i].vel[j].first = w*swarm[i].vel[j].first
					+ rho1*(swarm[i].pos_b[j].first - swarm[i].pos[j].first)
					+ rho2*(swarm[i].pos_nb[j].first - swarm[i].pos[j].first);
				swarm[i].vel[j].second = w*swarm[i].vel[j].second
					+ rho1*(swarm[i].pos_b[j].second - swarm[i].pos[j].second)
					+ rho2*(swarm[i].pos_nb[j].second - swarm[i].pos[j].second);
				// update position

				swarm[i].pos[j].first += swarm[i].vel[j].first;
				swarm[i].pos[j].second += swarm[i].vel[j].second;
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

			if (swarm[i].fit < solution.fit_b) //more large more better
			{
				improved = 1;
				// update best fitness
				solution.fit_b = swarm[i].fit;
				// copy particle pos to gbest vector
				solution.gbest = swarm[i].pos;
				solution.resualt = swarm[i].resualt;
			}

		}
		qDebug() << step << solution.fit_b;

	}
	
	qDebug() << "end run solver and we have" << solution.resualt.size() << "points";
	return solution.resualt;
}

double PSO::pso_obj_fun_t(particle& bird)
{
	bird.resualt.clear();
	std::vector<std::pair<int, int>> que;
	for (auto iter=bird.pos.rbegin();iter!=bird.pos.rend();iter++)
	{
		que.push_back(*iter);
	}
	
	//calculate the non-overlap area
	IntPoint p;
	Path rec(4);
	Paths solution, lastclipper;
	Clipper tsolver;
	int fitness = 0;
	for (int i=0;i<meshs_.size();i++)
	{

		for (int j = 0; j <meshs_[i].size();j++)
		{
			
			std::vector<Vec3f> box = meshs_[i][j]->getBoundingBox();
			auto loop_list_ = meshs_[i][j]->GetBLoop();
			using namespace ClipperLib;
			ClipperLib::Paths polygon;
			polygon.resize(loop_list_.size());
			IntPoint p;
			for (int m = 0; m < loop_list_.size(); m++)
			{
				for (int n = 0; n < loop_list_[m].size(); n++)
				{
					p.X = (int)(loop_list_[m][n]->pvert_->position().x()*1e3);
					p.Y = (int)(loop_list_[m][n]->pvert_->position().y()*1e3);
					polygon[m] << p;
				}
			}
			Paths sub,union_rec;
			Clipper solver;
			solver.AddPaths(lastclipper, ptClip, true);
			solver.AddPaths(polygon, ptSubject, true);
			solver.Execute(ctDifference, sub, pftNonZero, pftNonZero);

			int min_x_, min_y_, max_x_, max_y_;
			min_x_ = ((int)box[1].x() - 2) * 1000;
			min_y_ = ((int)box[1].y() - 2) * 1000;
			max_x_ = ((int)box[0].x() + 2) * 1000;
			max_y_ = ((int)box[0].y() + 2) * 1000;

			Path rec(4);
			pair<int, int> location = que.back();
			que.pop_back();
		
			for (p.X = min_x_; p.X < max_x_; p.X += dense.X)
			{
				for (p.Y = min_y_; p.Y < max_y_; p.Y += dense.Y)
				{
					p.X += location.first;
					p.Y += location.second;
				
					solver.Clear();
					Paths solution;
					rec[0].X = p.X - dense.X / 2;
					rec[0].Y = p.Y - dense.Y / 2;
					rec[1].X = p.X + dense.X / 2;
					rec[1].Y = p.Y - dense.Y / 2;
					rec[2].X = p.X + dense.X / 2;
					rec[2].Y = p.Y + dense.Y / 2;
					rec[3].X = p.X - dense.X / 2;
					rec[3].Y = p.Y + dense.Y / 2;
					solver.AddPaths(sub, ptClip, true);
					solver.AddPath(rec, ptSubject, true);
					solver.Execute(ctIntersection, solution, pftNonZero, pftNonZero);
					if (solution.size())
					{
						
						bird.resualt.push_back(Vec3f(p.X / 1000, p.Y / 1000, 0));
						fitness++;
						union_rec<<rec;
					}
				}
			}
			tsolver.Clear();
			tsolver.AddPaths(lastclipper, ptSubject, true);
			tsolver.AddPaths(union_rec, ptClip, true);
			tsolver.Execute(ctUnion, lastclipper, pftNonZero, pftNonZero);
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