#pragma once
#include "particleForceGen.h"

namespace cyclone
{
	class ParticleDrag : public ParticleForceGenerator
	{
	public:
		ParticleDrag(const real& k1, const real& k2);
		~ParticleDrag() = default;

		void updateForce(std::weak_ptr<Particle> particle, real dt) override;
		void updateConstants(real k1, real k2) { this->k1 = k1; this->k2 = k2; }
	private:
		real k1;
		real k2;
	};
}