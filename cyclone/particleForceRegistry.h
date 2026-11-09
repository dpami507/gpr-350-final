#pragma once
#include "particle.h"
#include "particleForceGen.h"
#include <vector>
#include <iostream>

namespace cyclone
{
	class ParticleForceRegistry
	{
	protected:
		struct ParticleForceRegistration
		{
			std::shared_ptr<Particle> particle;
			std::shared_ptr<ParticleForceGenerator> fg;
		};

		std::vector<ParticleForceRegistration> mRegistry;

	public:
		ParticleForceRegistry();
		~ParticleForceRegistry();

		void add(std::shared_ptr<Particle> particle, std::shared_ptr<ParticleForceGenerator> fg);
		void clear();
		void cleanup();

		void updateForces(real dt);
	};
}