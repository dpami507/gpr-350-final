#include "particleForceRegistry.h"

using namespace cyclone;

ParticleForceRegistry::ParticleForceRegistry()
{
}

ParticleForceRegistry::~ParticleForceRegistry()
{
	cleanup();
}
void ParticleForceRegistry::add(std::shared_ptr<Particle> particle, std::shared_ptr<ParticleForceGenerator> fg)
{
	mRegistry.push_back({ particle, fg });
}
void ParticleForceRegistry::clear()
{
	mRegistry.clear();
}
void ParticleForceRegistry::cleanup()
{
	clear();
}
void ParticleForceRegistry::updateForces(real dt)
{
	for (auto r : mRegistry)
	{
		r.fg->updateForce(r.particle, dt);
	}
}