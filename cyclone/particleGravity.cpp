#include "particleGravity.h"

using namespace cyclone;

ParticleGravity::ParticleGravity(const Vector3& gravity) : mGravity(gravity)
{ }

void ParticleGravity::updateForce(std::weak_ptr<Particle> particle, real dt)
{
	auto p = particle.lock();
	// dont need to add anything to a particle that cant move :/
	if (!p->hasFiniteMass()) return;

	p->addForce(mGravity * p->getMass());
}