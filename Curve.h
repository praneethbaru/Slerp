#pragma once
#include <vector>
#include <iostream>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>

class Curve
{
public:

	Curve();
	~Curve();
	
	void init();
	void calculate_curve();
	glm::vec3 calculate_point(glm::mat4 co, glm::mat4x3 po, double v);

	static glm::mat4 quat_to_rotation_mat(const glm::quat m_quat);

	glm::quat interpolate_quat(int i, double u);

	

	
	
public:
	float tau = 0.5; // Coefficient for catmull-rom spline
	int num_points_per_segment = 200;
	double max_distance;

	std::vector<glm::vec3> control_points_pos;
	std::vector<glm::vec3> get_points;
	std::vector<glm::vec3> curve_points_pos;
	std::vector<glm::quat> control_points_quaternion;
	std::vector<glm::quat> get_quaternion;
	float point_length[1608][4];
	float u_quat[2000][6];

};