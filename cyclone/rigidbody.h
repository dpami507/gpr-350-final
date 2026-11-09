#pragma once
#include "core.h"

namespace cyclone
{
	class Rigidbody
	{
	public:
		Rigidbody(Vector3 pos = Vector3(), Vector3 vel = Vector3(), Vector3 accel = Vector3(), real mass = 1.f, real linearDaming = 0.995f, real angularDamping = 0.995f);
		~Rigidbody() = default;

		void integrate(real dt);
		void addForce(const Vector3& force);
		void addTorque(const Vector3& torque);
		void addForceAtPoint(const Vector3& force, const Vector3& point);
		Matrix4 getTransformMatrix();

		void setMass(real mass);
		void setInertiaTensor(const Matrix3& inertiaTensor);
		inline void setAngularDaming(real daming) { mAngularDamping = daming; };
		inline void setLinearDaming(real daming) { mLinearDamping = daming; };
		inline void setPosition(const Vector3& pos) { mPosition = pos; };
		inline void setLinearVelocity(const Vector3& vel) { mVelocity = vel; };
		inline void setAngularVelocity(const Vector3& vel) { mAngularVelocity = vel; };
		inline void setAcceleration(const Vector3& accel) { mAcceleration = accel; };
		inline void setOrientation(const Quaternion& orient) { mOrientation = orient; }

		Vector3 orientationToAngleAxis();

		inline Vector3 getPos() const { return mPosition; };
		inline real getMass() const { return 1.f / mInverseMass; };
		inline real getInverseMass() const { return mInverseMass; };
		inline Quaternion getOrientation() const { return mOrientation; };
		inline Vector3 getLinearVelocity() const { return mVelocity; };

	private:
		Vector3 mPosition = Vector3();
		Vector3 mVelocity = Vector3();
		Vector3 mAcceleration = Vector3();
		real mLinearDamping = 0.995;
		real mInverseMass = 1;
		Vector3 mForceAccum = Vector3();

		Quaternion mOrientation = Quaternion();
		Vector3 mAngularVelocity = Vector3();
		Vector3 mTorqueAccum = Vector3();
		Matrix3 mInverseInertiaTensor = Matrix3(1, 0, 0, 0, 1, 0, 0, 0, 1);
		real mAngularDamping = 0.995;
		Matrix3 mInverseInertiaTensorWorld = Matrix3(1, 0, 0, 0, 1, 0, 0, 0, 1);
		Matrix4 mTransformMatrix = Matrix4();

		void clearAccumulators();
		void calculateDerivedData();
		void calculateInertiaTensor();
	};
}