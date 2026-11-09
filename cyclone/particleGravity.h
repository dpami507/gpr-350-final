#pragma once
#include "particleForceGen.h"

namespace cyclone
{
	class ParticleGravity : public ParticleForceGenerator
	{
	public:
		ParticleGravity(const Vector3& gravity);
		~ParticleGravity() = default;

		inline void setGravity(const Vector3& gravity) { mGravity = gravity; }
		void updateForce(std::weak_ptr<Particle> particle, real dt) override;
	private:
		Vector3 mGravity;
	};
}