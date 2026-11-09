#pragma once
#include "particle.h"
#include <memory>
#include <iostream>

namespace cyclone
{
	class ParticleForceGenerator
	{
	public:
		ParticleForceGenerator() {}
		~ParticleForceGenerator() {}

		virtual void updateForce(std::weak_ptr<Particle> particle, real dt) = 0;
	private:

	};
}