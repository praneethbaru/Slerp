#include "Curve.h"
#include <iostream>

int quat_index = 0;
double tot_points = 200;
Curve::Curve()
{
}

Curve::~Curve()
{
}

void Curve::init()
{
	this->control_points_pos =
	{
		{ 0.0, 8.5, -2.0 },
		{ -3.0, 11.0, 2.3 },
		{ -6.0, 8.5, -2.5 },
		{ -4.0, 5.5, 2.8 },
		{ 1.0, 2.0, -4.0 },
		{ 4.0, 2.0, 3.0 },
		{ 7.0, 8.0, -2.0 },
		{ 3.0, 10.0, 3.7 }
	};
	this->control_points_quaternion =
	{
		{0.991015,0.00883871,-0.102529,-0.085432},
		{0.998588,-0.000207206,-0.052976,0.0039058},
		{0.742373,0.186137,0.229859,0.601165},
		{-0.109644,0.356721,0.926794,-0.0422019},
		{0.0248536,-0.036511,0.999024,-0.000908318},
		{0.136048,-0.366425,0.918847,-0.0542544},
		{0.767665,-0.192912,0.270156,-0.548171},
		{0.999426,0.000371592,-0.0116842,-0.0317848}
	};

	calculate_curve();
}

void Curve::calculate_curve()
{
	this->curve_points_pos =
	{
		{ 0.0, 8.5, -2.0 }, //p0
		{ -3.0, 11.0, 2.3 }, //p1
		{ -6.0, 8.5, -2.5 }, //p2
		{ -4.0, 5.5, 2.8 },  //p3
		{ 1.0, 2.0, -4.0 },  //p4
		{ 4.0, 2.0, 3.0 },   //p5
		{ 7.0, 8.0, -2.0 },   //p6
		{ 3.0, 10.0, 3.7 },   //p7
	};
	this->get_points;
	this->point_length;
	this->get_quaternion;
	glm::mat4 coef_matrix =
	{
		{ -0.5,1.5,-1.5,0.5} ,
		{1,-2.5,2,-0.5},
		{-0.5,0,0.5,0},
		{0,1,0,0}
	};

	int i, j;
	glm::vec3 point1, point2, point3, point4;
	glm::mat4x3 point_mat;
	glm::vec3 p1 = { 0.0,8.5,-2.0 };
	get_points.push_back(p1);
	glm::quat getSlerp;

	for (i = 0; i < curve_points_pos.size(); i++)
	{
		if (i != 0 && i != 7 && i != 6)
		{
			point1 = curve_points_pos[i - 1];
			point2 = curve_points_pos[i];
			point3 = curve_points_pos[i + 1];
			point4 = curve_points_pos[i + 2];
		}

		else if (i == 0)
		{
			point1 = curve_points_pos[7];
			point2 = curve_points_pos[i];
			point3 = curve_points_pos[i + 1];
			point4 = curve_points_pos[i + 2];

		}

		else if (i == 6)
		{
			point1 = curve_points_pos[i - 1];
			point2 = curve_points_pos[i];
			point3 = curve_points_pos[i + 1];
			point4 = curve_points_pos[0];

		}

		else if (i == 7)
		{
			point1 = curve_points_pos[i - 1];
			point2 = curve_points_pos[i];
			point3 = curve_points_pos[0];
			point4 = curve_points_pos[1];

		}

		point_mat =
		{
			{point1[0],point1[1],point1[2] },
			{point2[0],point2[1],point2[2] },
			{point3[0],point3[1],point3[2] },
			{point4[0],point4[1],point4[2] }
		};

		for (int v = 1; v <= tot_points; v++)
		{

			get_points.push_back(calculate_point(coef_matrix, point_mat, v));
			getSlerp = interpolate_quat(i, v);

			u_quat[quat_index][0] = getSlerp.w;
			u_quat[quat_index][1] = getSlerp.x;
			u_quat[quat_index][2] = getSlerp.y;
			u_quat[quat_index][3] = getSlerp.z;

			get_quaternion.push_back(getSlerp);
			quat_index++;


		}
	}

	float distance = 0.0;

	for (int i = 0; i < get_points.size() - 1; i++)
	{
		if (i == 0)
		{
			distance = distance + glm::distance(get_points[i], get_points[i]);
		}
		else
		{
			distance = distance + glm::distance(get_points[i], get_points[i + 1]);
		}

		point_length[i][0] = get_points[i].x;
		point_length[i][1] = get_points[i].y;
		point_length[i][2] = get_points[i].z;
		point_length[i][3] = distance;

		max_distance = distance;
	}
}

glm::vec3 Curve::calculate_point(glm::mat4 co, glm::mat4x3 po, double v)
{
	glm::mat4 coef_matrix = co;
	glm::mat4x3 point = po;
	int i, j, k;
	double new_v = v / tot_points;
	glm::mat4x3 result_matrix_1;
	double result_matrix_2[] = { 0.0, 0.0, 0.0 };
	double u[] = { (new_v)*(new_v)*(new_v), (new_v)*(new_v), (new_v), 1 };

	/*---------------------co-eff_mat x point_mat-------------------*/
	for (i = 0; i <= 3; i++)
	{
		for (j = 0; j <= 2; j++)
		{
			result_matrix_1[i][j] = 0;
			for (int k = 0; k <= 3; k++)
			{
				result_matrix_1[i][j] += coef_matrix[i][k] * point[k][j];
			}
		}
	}

	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 4; j++)
		{
			result_matrix_2[i] += u[j] * result_matrix_1[j][i];
		}
	}
	return glm::vec3(result_matrix_2[0], result_matrix_2[1], result_matrix_2[2]);
}

glm::mat4 Curve::quat_to_rotation_mat(const glm::quat m_quat)
{
	int i, j;
	glm::mat4 rotation_mat =
	{
		{1 - (2 * m_quat.y*m_quat.y) - (2 * m_quat.z*m_quat.z), (2 * m_quat.x*m_quat.y) + (2 * m_quat.w*m_quat.z), (2 * m_quat.x *m_quat.z) - (2 * m_quat.w*m_quat.y),0},
		{(2 * m_quat.x*m_quat.y) - (2 * m_quat.w*m_quat.z), 1 - (2 * m_quat.x*m_quat.x) - (2 * m_quat.z*m_quat.z), (2 * m_quat.y*m_quat.z) + (2 * m_quat.w*m_quat.x),0},
		{(2 * m_quat.x*m_quat.z) + (2 * m_quat.w*m_quat.y), (2 * m_quat.y*m_quat.z) - (2 * m_quat.w*m_quat.x), 1 - (2 * m_quat.x*m_quat.x) - (2 * m_quat.y*m_quat.y),0},
		{0,0,0,1}
	};
	return rotation_mat;
}

glm::quat Curve::interpolate_quat(int i, double u)
{

	glm::quat slerp;
	float v = u / tot_points;
	if (i >= 0 && i < 7)
	{
		slerp = glm::slerp(control_points_quaternion[i], control_points_quaternion[i + 1], v);
	}
	else if (i == 7)
	{
		slerp = glm::slerp(control_points_quaternion[i], control_points_quaternion[0], v);
	}
	return slerp;
}