#include "particleBouyancy.h"
#include <iostream>

using namespace cyclone;

ParticleBouyancy::ParticleBouyancy(real waterLevel, real liquidDensity, real objVolume, real maxDepth) :
	waterLevel(waterLevel), liquidDensity(liquidDensity), objVolume(objVolume), maxDepth(maxDepth)
{ }

void ParticleBouyancy::updateForce(std::weak_ptr<Particle> particle, real dt)
{
	auto p = particle.lock();

	real depth = p->getPos().y;
	real d = (waterLevel + maxDepth - depth) / (2.f * maxDepth);

	// Add no force (out of water)
	if (d <= 0)
		return;
	// Clamp d to 1
	if (d >= 1) d = 1;

	// add force based on 'd'
	real magnitude = d * -liquidDensity * objVolume * -9.81f;

	p->addForce(Vector3::UP * magnitude);
}