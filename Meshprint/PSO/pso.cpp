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
	swarm.clear();
	swarm.resize(settings.size);
	qDebug() << "start initialize swarm";
	solution.fit_b = Vec2f(1e6, 1e6);
	for (int id=0;id<settings.size;id++)
	{	
		for (int i = 0; i < component_regions_mesh.size(); i++)
		{
			for (int j = 0; j < component_regions_mesh[i].size(); j++)
			{

				dense = SupportLib::get_dense(j);
				for (int k = 0; k < component_regions_mesh[i][j].size(); k++)
				{
					if (id == 0)
					{
						swarm[id].pos.push_back(Vec2f(0, 0));
					}
					else
					{
						swarm[id].pos.push_back(Vec2f((float)rand() / RAND_MAX*dense.x(), (float)rand() / RAND_MAX*dense.y()));
					}
				}
			}
		}
		swarm[id].fit = pso_obj_fun_t(swarm[id]);
		if (swarm[id].fit < solution.fit_b)
		{
			solution.fit_b = swarm[id].fit;
			solution.gbest = swarm[id].pos;
			solution.resualt = swarm[id].resualt;
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
	
	qDebug() << "end initialize swarm"<<"best fitness is "<<solution.fit_b.x()<< solution.fit_b.y();

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
	while (solution.fit_b.x() == 1e6)
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
			}

		}
		
		if (improved)
		{
			count = 0;
		}
		else
		{
			count++;
		}
		
		if (count > 200)
		{
			break;
		}

		qDebug() << step << solution.fit_b.x()<< solution.fit_b.y();
	}
	qDebug() << "end run solver and we have" << "points";
	return solution.resualt;
}

Vec2f PSO::pso_obj_fun_t(particle& bird)
{
	bird.resualt.clear();
	std::vector<Vec2f> que;
	for (auto iter=bird.pos.rbegin();iter!=bird.pos.rend();iter++)
	{
		que.push_back(*iter);
	}
	float max_rate = 0;
	for (int i = 0; i < component_regions_mesh.size(); i++)
	{
		std::map<int, std::set<Vec3f>> temp_int_set;
		for (int j = 0; j < component_regions_mesh[i].size(); j++)
		{
			Vec2f dense = SupportLib::get_dense(j);
			dense = SupportLib::get_dense(j);
			for (int k = 0; k < component_regions_mesh[i][j].size(); k++)
			{
				float rate= SupportLib::single_area_sampling(component_regions_mesh[i][j][k], dense, temp_int_set, j,que.back());
				que.pop_back();
				max_rate = max_rate > rate ? max_rate : rate;
			}
		}	
		Paths clip;
		dense = SupportLib::get_dense(4);
		for (int ii = 0; ii < COUNTOFANGLE; ii++)
		{
			for (auto iter = temp_int_set[ii].begin(); iter != temp_int_set[ii].end(); iter++)
			{
				bird.resualt[ii].insert(*iter);
				Path rectangle;
				rectangle << IntPoint(((*iter).x() - dense.x() / 2) * 1000, ((*iter).y() - dense.y() / 2) * 1000)
					<< IntPoint(((*iter).x() + dense.x() / 2) * 1000, ((*iter).y() - dense.y() / 2) * 1000)
					<< IntPoint(((*iter).x() + dense.x() / 2) * 1000, ((*iter).y() + dense.y() / 2) * 1000)
					<< IntPoint(((*iter).x() - dense.x() / 2) * 1000, ((*iter).y() + dense.y() / 2) * 1000);
				clip << rectangle;
			}
		}
		Paths subject;
		auto belist = component[i]->GetBLoop();
		for (int ii = 0; ii < belist.size(); ii++)
		{
			Path loop;
			for (int jj = 0; jj < belist[ii].size(); jj++)
			{
				loop << IntPoint(belist[ii][jj]->pvert_->position().x() * 1000, belist[ii][jj]->pvert_->position().y() * 1000);
			}
			subject << loop;
		}
		Clipper sov;
		sov.AddPaths(clip, ptClip, true);
		sov.AddPaths(subject, ptSubject, true);
		sov.Execute(ctDifference, subject, pftNonZero, pftNonZero);
		for (int ii = 0; ii < subject.size(); ii++)
		{
			float rate_ = Area(subject[ii]) / (1e6*dense.x()*dense.y());
			max_rate = max_rate > rate_ ? max_rate : rate_;
		}
	}

	int num = 0;
	for (int i = 0; i < COUNTOFANGLE; i++)
	{
		num += bird.resualt[i].size();
	}
	if (max_rate < 0.5)
	{
		return Vec2f(num, max_rate);
	}
	else
		return Vec2f(1e6, max_rate);
	
}

//==============================================================
// calculate swarm size based on dimensionality
int PSO::pso_calc_swarm_size(int dim) {
	int size = 10. + 2. * sqrt(dim);
	return (size > PSO_MAX_SIZE ? PSO_MAX_SIZE : size);
}
