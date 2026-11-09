#pragma once
#include "particleForceGen.h"

namespace cyclone
{
	class ParticleBouyancy : public ParticleForceGenerator
	{
	public:
		ParticleBouyancy(real waterLevel, real liquidDensity, real objVolume, real maxDepth);
		~ParticleBouyancy() = default;

		void updateForce(std::weak_ptr<Particle> particle, real dt) override;

		void setWaterLevel(real level) { waterLevel = level; }
		void setLiquidDensity(real density) { liquidDensity = density; }
		void setObjVolume(real vol) { objVolume = vol; }
		void setMaxDepth(real depth) { maxDepth = depth; }
	private:
		real waterLevel;
		real liquidDensity;
		real objVolume;
		real maxDepth;
	};
}