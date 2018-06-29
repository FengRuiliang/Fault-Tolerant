#include "PSO.h"
#include <math.h>
#include <ctime>
#include <stdlib.h>
#include <qdebug.h>
#include "Support.h"
#include <omp.h>
#include "Library/Octree.h"

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
	
	for (int id=0;id<settings.size;id++)
	{	
		
		if (id == 0)
		{
			swarm[id].pos.push_back(Vec2f(0, 0));
			//swarm[id].pos.push_back(Vec2f(0, 0));
		}
		else
		{
			dense = SupportLib::get_dense(0 * 5);
			swarm[id].pos.push_back(Vec2f((float)rand() / RAND_MAX*dense.x(), (float)rand() / RAND_MAX*dense.y()));
			//swarm[id].pos.push_back(Vec2f((float)rand() / RAND_MAX*dense.x(), (float)rand() / RAND_MAX*dense.y()));

		}
		

		
		//for (int i=0;i<component_regions_mesh.size();i++)
		//{
		//	for (int j=0;j<component_regions_mesh[i].size();j++)
		//	{
	
		//		dense = SupportLib::get_dense(j * 5);
		//		for (int k=0;k<component_regions_mesh[i][j].size();k++)
		//		{
		//			if (id == 0)
		//			{
		//				swarm[id].pos.push_back(Vec2f(0, 0));
		//			}
		//			else
		//			{
		//				swarm[id].pos.push_back(Vec2f((float)rand()/ RAND_MAX*dense.x(), (float)rand() / RAND_MAX*dense.y()));
		//			}
		//		}
		//	}
		//}
		swarm[id].fit = pso_obj_fun_t(swarm[id]);
		if (swarm[id].fit < solution.fit_b)
		{
			solution.fit_b = swarm[id].fit;
			solution.gbest = swarm[id].pos;
			solution.resualt = swarm[id].resualt;
			solution.angle_paths = test_path;

		}
		swarm[id].vel = swarm[id].pos;
		swarm[id].vel = swarm[id].pos;

		swarm[id].fit_hist_b = swarm[id].fit;
		swarm[id].pos_hist_b = swarm[id].pos;
		
		//qDebug() << " the " << id << "st fitness is" << swarm[id].fit;
	}
	for (int i = 0; i < settings.size; i++)
	{
		swarm[i].pos_nb = solution.gbest;
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



std::map<int,std::set<Vec3f>> PSO::pso_solve()
{
	qDebug() << "star pso solution";

	while (solution.fit_b == MAX_FITNESS)
		pso_swarm_init();


	// initialize omega using standard value
	float  w = PSO_INERTIA;// current omega
							// RUN ALGORITHM
	int count = 0;
	for (int step = 0; step < settings.steps; step++)
	{
		test_path.clear();
		// update current step
		settings.step = step;
		// update inertia weight
		// do not bother with calling a calc_w_const function
		w = calc_inertia_lin_dec(step);

		// update pos_nb matrix (find best of neighborhood for all particles)


		improved = 0;	// the value of improved was just used; reset it
		float rho1, rho2; // random numbers (coefficients)
		// update all particles

		for (int i = 0; i < settings.size; i++)
		{

			//qDebug() << "the" << step << "th iteration";
			srand((unsigned)time(NULL));
			// for each dimension
			// calculate stochastic coefficients
			rho1 = settings.c1 * rand() / RAND_MAX;
			rho2 = settings.c2 * rand() / RAND_MAX;
		/*	float rho3 = pow(-1.0, (float)(rand() % 3));
			rho1 *= rho3;
			rho2 *= rho3;*/
			for (int j=0;j<swarm[i].pos.size();j++)
			{
				// update velocity
				
				swarm[i].vel[j] =
					w*swarm[i].vel[j] 
					+ rho1*(swarm[i].pos_hist_b[j] - swarm[i].pos[j])
					+ rho2*(swarm[i].pos_nb[j] - swarm[i].pos[j]);

				// update position
				swarm[i].pos[j] += swarm[i].vel[j];
			}
			
			
			// update particle fitness
			swarm[i].fit = pso_obj_fun_t(swarm[i]);
			// update personal best position?
			if (swarm[i].fit < swarm[i].fit_hist_b) {
				swarm[i].fit_hist_b = swarm[i].fit;
				// copy contents of pos[i] to pos_b[i]
				swarm[i].pos_hist_b = swarm[i].pos;
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
				solution.angle_paths = test_path;
			}

		}
		
		if (improved)
		{
			count = 0;
		}
		else
			count++;
		if (count == 500)
		{
			break;
		}
		
	qDebug() <<step<<solution.fit_b;
	}
	
	qDebug() << solution.gbest[0].x() << solution.gbest[0].y();
	qDebug() << "end run solver and we have" << "points";
	return solution.resualt;
}

double PSO::pso_obj_fun_t(particle& bird)
{
	bird.resualt.clear();
	std::vector<Vec2f> que;
	for (auto iter=bird.pos.rbegin();iter!=bird.pos.rend();iter++)
	{
		que.push_back(*iter);
	}
	
	float int_aera = 0;
	std::map<int, std::set<Vec3f>> temp_int_set;
	dense = SupportLib::get_dense(0 * 5);
	auto sampleresualt1 = SupportLib::single_area_sampling(component_regions_mesh[0][0][1], dense, temp_int_set, 0, que.back());
	que.pop_back();
	//dense = SupportLib::get_dense(0 * 5);
	//auto sampleresualt12 = SupportLib::single_area_sampling(component_regions_mesh[0][1][0], dense, temp_int_set, 1, que.back());
	//que.pop_back();
	//int_aera = int_aera1>int_aera2? int_aera1:int_aera2;

	//////////////////////////////////////////////////////////////////////////

//	for (int i = 0; i < component_regions_mesh.size(); i++)
//	{
//	
//		if (i!=0)
//		{
//			continue;
//		}
//		std::map<int, std::set<Vec3f>> temp_int_set;
//// 		for (int j = 0; j < component_local_sup_point[i].size(); j++)
//// 		{
//// 			temp_int_set[0].insert(component_local_sup_point[i][j]);
//// 		}
//
//
//		for (int j = 0; j < component_regions_mesh[i].size(); j++)
//		{
//			if (j!=0)
//			{
//				continue;
//			}
//			//Vec2f dense = SupportLib::get_dense(j * 5);
//			dense = SupportLib::get_dense(j * 5);
//			for (int k = 0; k < component_regions_mesh[i][j].size(); k++)
//			{
//				if (k!=1)
//				{
//					continue;
//				}
//				int_aera = SupportLib::single_area_sampling(component_regions_mesh[i][j][k], dense, temp_int_set, j,que.back());
//				que.pop_back();
//			}
//		}	
//		for (int ii = 0; ii < COUNTOFANGLE; ii++)
//		{
//			for (auto iter = temp_int_set[ii].begin(); iter != temp_int_set[ii].end(); iter++)
//			{
//				bird.resualt[ii].insert(*iter);
//			}
//
//		}
//	}


	for (int ii = 0; ii < COUNTOFANGLE; ii++)
	{
		for (auto iter = temp_int_set[ii].begin(); iter != temp_int_set[ii].end(); iter++)
		{
			bird.resualt[ii].insert(*iter);
		}

	}
	//float max_ = sampleresualt1.second > sampleresualt12.second ? sampleresualt1.second : sampleresualt12.second;
	float max_ = sampleresualt1.second;
	if (max_>0.5*2*2)
	{
		return 1e6;
	}


	for (int i=0;i<COUNTOFANGLE;i++)
	{
		max_+= bird.resualt[i].size();
	}
	return max_;
}

//==============================================================
// calculate swarm size based on dimensionality
int PSO::pso_calc_swarm_size(int dim) {
	int size = 10. + 2. * sqrt(dim);
	return (size > PSO_MAX_SIZE ? PSO_MAX_SIZE : size);
}
Vec2f PSO::get_dense(int angle)
{

	Vec2f d_;
	if (angle < 15)
	{
		d_.x() = 2.5;

		if (angle < 5)
		{
			d_.y() = 2.5;
		}
		else if (angle < 10)
		{
			d_.y() = 2.5;
		}
		else if (angle < 15)
		{
			d_.y() = 8.0;
		}
	}
	else
	{
		d_.x() = 3.0;

		if (angle < 18)
		{
			d_.y() = 10.0;
		}
		else
			d_.y() = 15.0;
	}
	return d_;
}
