#pragma once
#include "verletParticle.h"
#include "rigidbody.h"
#include "collider.h"

namespace cyclone
{
	class Rope
	{
	public:
		Rope(int segmentCount, real segmentLength, real thickness, real damping, real mass, Vector3 position, Vector3 accel);
		~Rope();
		void integrate(int constraintIterations, cyclone::real dt);
		void setPosition(const Vector3& pos);
		void checkCollision(Collider* collider, real frictionCoef);
		inline real getThickness() const { return mThickness; }
		void setDamping(real damping);
		inline std::vector<VerletParticle>& getParticles() { return mParticles; }

		void handleCollision(Collider* collider, VerletParticle& particle, real penetration, const Vector3& normal, real frictionCoef);

	private:
		std::vector<VerletParticle> mParticles;
		cyclone::real mThickness;
		cyclone::real mSegmentLength;
	};
}