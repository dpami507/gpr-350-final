#include "particleDrag.h"

using namespace cyclone;

ParticleDrag::ParticleDrag(const real& k1, const real& k2) : k1(k1), k2(k2)
{
}
void ParticleDrag::updateForce(std::weak_ptr<Particle> particle, real dt)
{
	auto p = particle.lock();

	// Get the velocity and drag coeff
	Vector3 force = p->getVel();
	real dragCoeff = force.magnitude();

	// set the drag coeff using the equation
	dragCoeff = (k1 * dragCoeff + k2 * dragCoeff * dragCoeff);
	force.normalise();

	// multiply the direction of the velocity by the negative of the dragcoeff
	force *= -dragCoeff;

	p->addForce(force);
}