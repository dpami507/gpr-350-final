#pragma once
#include "core.h"

namespace cyclone
{
	class Particle
	{
	public:
		Particle(Vector3 position = Vector3::ZERO, Vector3 velocity = Vector3::ZERO, Vector3 acceleration = Vector3::ZERO, 
				real damping = 0.995f, real mass = 1.0f);
		~Particle() = default;

		void integrate(real dt);

		// setters
		void addForce(const Vector3& force);
		inline void setAccel(const Vector3& acceleration) { mAcceleration = acceleration; }
		inline void setVel(const Vector3& velocity) { mVelocity = velocity; }
		inline void setPos(const Vector3& position) { mPosition = position; }

		// mass
		inline real getInverseMass() { return mInverseMass; }
		void setInverseMass(const real& inverseMass);
		inline real getMass() { return 1.0f / mInverseMass; }
		void setMass(const real& mass);
		bool hasFiniteMass();

		// getters
		inline Vector3 getAccel() const { return mAcceleration; }
		inline Vector3 getVel() const { return mVelocity; }
		inline Vector3 getPos() const { return mPosition; }
	private:
		Vector3 mPosition = Vector3::ZERO;
		Vector3 mVelocity = Vector3::ZERO;
		Vector3 mAcceleration = Vector3::ZERO;
		real mDamping = 0.995f;

		Vector3 mForceAccum = Vector3::ZERO;
		real mInverseMass;

		void clearAccumlator() { mForceAccum.clear(); }
	};
}