#include "Bunny_Animation.h"
#include <Windows.h>

float x, y, z;
double t = 0.0, dist = 0.0, v0;

Bunny_Animation::Bunny_Animation()
{
	this->m_model_mat = glm::mat4(1.0f);
}


Bunny_Animation::~Bunny_Animation()
{
}

void Bunny_Animation::init()
{
	reset();
}

void Bunny_Animation::init(Curve* animation_curve)
{
	m_animation_curve = animation_curve;
	reset();
}

void Bunny_Animation::update(float delta_time, bool checkbox)
{
	if (t <= tmax)
	{
		int index = getIndex(getDistance(t));
		if (checkbox == 1) {
			moveBunny(getIndex(getDistance(t)));
		}
		else
		{
			if (index != 0)
			{
				m_model_mat[3][0] = m_animation_curve->point_length[index - 1][0];
				m_model_mat[3][1] = m_animation_curve->point_length[index - 1][1];
				m_model_mat[3][2] = m_animation_curve->point_length[index - 1][2];
			}

			x = (m_animation_curve->point_length[index][0] - m_model_mat[3][0]);
			y = (m_animation_curve->point_length[index][1] - m_model_mat[3][1]);
			z = (m_animation_curve->point_length[index][2] - m_model_mat[3][2]);
			m_model_mat = glm::translate(m_model_mat, glm::vec3(x, y, z));
		}

		t = t + delta_time;
	}
}

void Bunny_Animation::reset()
{
	m_model_mat = glm::mat4(1.0f);
	if (m_animation_curve != nullptr && m_animation_curve->control_points_pos.size() > 0)
	{
		m_model_mat = glm::translate(m_model_mat, m_animation_curve->control_points_pos[0]);

	}
	resetBunny();
	t = 0.0;
	dist = 0.0;
}


double Bunny_Animation::getVelocity()
{
	double vel;
	double ti;
	ti = t2 * tmax - (0.5*t1*tmax) + 0.5*(tmax - t2 * tmax);
	vel = m_animation_curve->max_distance / ti;
	v0 = vel;
	return v0;
}

double Bunny_Animation::getDistance(float time)
{
	double dist, dist1;
	double vel = getVelocity();

	if (time > 0 && time <= t1 * tmax)
	{
		dist = (0.5 * v0* time * time) / (t1*tmax);

	}

	else if (time > t1 * tmax && time <= t2 * tmax)
	{
		dist = (0.5*v0*t1*tmax) + (v0*(time - t1 * tmax));

	}

	else if (time > t2*tmax && time <= tmax)
	{
		double x = (time - t2 * tmax) / (2 * (tmax - t2 * tmax));
		dist = (0.5*v0*t1*tmax) + (v0*(t2 - t1)*tmax) + v0 * (time - (t2*tmax))*(1 - x);

	}
	return dist;
}

int Bunny_Animation::getIndex(float dist)
{
	int ind;
	for (int i = 0; i < m_animation_curve->get_points.size() - 1; i++)
	{
		if (m_animation_curve->point_length[i][3] <= dist)
		{
			ind = i;
		}
	}
	return ind;
}

void Bunny_Animation::moveBunny(int index)
{

	/*---------------TRANSLATION OF THE BUNNY------------------------ */
	x = (m_animation_curve->point_length[index][0] - m_model_mat[3][0]);
	y = (m_animation_curve->point_length[index][1] - m_model_mat[3][1]);
	z = (m_animation_curve->point_length[index][2] - m_model_mat[3][2]);
	m_model_mat = glm::translate(m_model_mat, glm::vec3(x, y, z));

	/*---------------ROTATION OF THE BUNNY------------------------ */
	glm::mat4 rotationBunny = m_animation_curve->quat_to_rotation_mat(m_animation_curve->get_quaternion[index]);
	glm::mat4 result;

	for (int a = 0; a < 3; a++)
	{
		for (int b = 0; b < 3; b++)
		{
			m_model_mat[a][b] = rotationBunny[a][b];
		}
	}
	m_model_mat[3][0] = m_animation_curve->point_length[index][0];
	m_model_mat[3][1] = m_animation_curve->point_length[index][1];
	m_model_mat[3][2] = m_animation_curve->point_length[index][2];
}

void Bunny_Animation::resetBunny()
{
	/*---------------RESETING THE POSITION OF THE BUNNY------------------------ */

	glm::mat4 result_mat, rotationMat;

	rotationMat = m_animation_curve->quat_to_rotation_mat(m_animation_curve->control_points_quaternion[0]);
	for (int a = 0; a < 4; a++)
	{
		for (int b = 0; b < 4; b++)
		{
			result_mat[a][b] = 0;
			for (int c = 0; c < 4; c++)
			{
				result_mat[a][b] += rotationMat[a][c] * m_model_mat[c][b];
			}
		}
	}

	for (int a = 0; a < 4; a++)
	{
		for (int b = 0; b < 4; b++)
		{
			m_model_mat[a][b] = result_mat[a][b];
		}
	}
}
