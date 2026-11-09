#include <particle.h>

using namespace cyclone;

Particle::Particle(Vector3 position, Vector3 velocity, Vector3 acceleration, real damping, real mass)
	: mPosition(position), mVelocity(velocity), mAcceleration(acceleration), mDamping(damping)
{
	setMass(mass);
}

void Particle::addForce(const Vector3& force)
{
	mForceAccum += force;
}
void Particle::setMass(const real& mass)
{
	// If mass is in the 'no-no zone' then set it to zero
	mInverseMass = (mass <= 0) ? 0 : 1.0f / mass;
}
void Particle::setInverseMass(const real& inverseMass)
{
	mInverseMass = (inverseMass < 0) ? 0 : inverseMass;
}
bool Particle::hasFiniteMass()
{
	return mInverseMass != 0;
}

void Particle::integrate(real dt)
{
	// Skip integrating things with infinite mass
	if (mInverseMass <= 0.0f) return;

	// Calculate acceleration value
	Vector3 finalAccel = mAcceleration;
	finalAccel += mForceAccum * mInverseMass;

	// Increase velocity
	mVelocity += finalAccel * dt;
	// Apply damping
	mVelocity *= pow(mDamping, dt);

	// Adjust Position
	mPosition += mVelocity * dt;

	clearAccumlator();
}