#pragma once

#include <vector>
#include <iostream>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Curve.h"

class Bunny_Animation
{

public:
	float t1 = 0.2f;
	float t2 = 0.8f;
	float tmax = 10;


private:
	glm::mat4 m_model_mat;
	Curve* m_animation_curve = nullptr;

public:
	Bunny_Animation();
	~Bunny_Animation();

	void init();
	void init(Curve* animation_curve);
	void update(float delta_time,bool checkbox);
	void reset();
	double getVelocity();
	double getDistance(float time);
	int getIndex(float dist);
	void moveBunny(int index);	
	void resetBunny();
	glm::mat4 get_model_mat() { return m_model_mat; };
};


