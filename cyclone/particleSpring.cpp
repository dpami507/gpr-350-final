#include "particleSpring.h"
#include <iostream>

using namespace cyclone;

ParticleSpring::ParticleSpring(const Vector3& anchor, const real& restLength,
	const real& springConstant, const real& dampingConstant)
	: anchor(anchor), springConstant(springConstant),
	dampingConstant(dampingConstant), restLength(restLength)
{
}
void ParticleSpring::updateForce(std::weak_ptr<Particle> particle, real dt)
{
	auto p = particle.lock();

	Vector3 force;
	Vector3 offset = p->getPos() - anchor;

	Vector3 vel = p->getVel();
	Vector3 dir = offset.unit();
	real magnitude = offset.magnitude();

	// F = -kx - cv
	Vector3 x = dir * (magnitude - restLength);
	force = x * -springConstant - vel * dampingConstant;

	p->addForce(force);
}