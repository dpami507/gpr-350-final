#pragma once
#include "particleForceGen.h"

namespace cyclone
{
	class ParticleSpring : public ParticleForceGenerator
	{
	public:
		ParticleSpring(const Vector3& anchor, const real& restLength, 
			const real& springConstant, const real& dampingConstant);
		~ParticleSpring() = default;

		void updateForce(std::weak_ptr<Particle> particle, real dt) override;

		inline void setAnchor(const Vector3& anchor) { this->anchor = anchor; }
		inline void setSpringConstant(const real& constant) { springConstant = constant; }
		inline void setDampingConstant(const real& constant) { dampingConstant = constant; }
		inline void setRestLength(const real& length) { restLength = length; }
	private:
		Vector3 anchor;
		real springConstant;
		real dampingConstant;
		real restLength;
	};
}