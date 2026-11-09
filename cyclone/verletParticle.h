#pragma once
#include "core.h"
#include <vector>

namespace cyclone
{
	struct VerletParticle
	{
		cyclone::Vector3 mPosition;
		cyclone::Vector3 mPrevPosition;
		cyclone::real mInverseMass;
		cyclone::real mDamping;
		cyclone::Vector3 mAcceleration;

		void integrate(cyclone::real dt)
		{
			cyclone::Vector3 vel = (mPosition - mPrevPosition) * pow(mDamping, dt);
			cyclone::Vector3 nextPos = mPosition + vel + mAcceleration * dt * dt;
			mPrevPosition = mPosition;
			mPosition = nextPos;
		}
	};

	class Constraints
	{
	public:
		Constraints() = default;
		~Constraints() = default;

		static void distanceConstraint(VerletParticle& p1, VerletParticle& p2, real restLength);
	private:
	};
}