#include "rigidbody.h"

using namespace cyclone;

Rigidbody::Rigidbody(Vector3 pos, Vector3 vel, Vector3 accel, real mass, real linearDaming, real angularDamping)
	: mPosition(pos), mVelocity(vel), mAcceleration(accel), mLinearDamping(linearDaming), mAngularDamping(angularDamping)
{
	setMass(mass);
}

Vector3 Rigidbody::orientationToAngleAxis()
{
    real theta = 2 * acos(mOrientation.r);
    Vector3 x(mOrientation.i, mOrientation.j, mOrientation.k);
    Vector3 vec = x * (1.0 / sin(theta * 0.5f));
    return vec;
}

void Rigidbody::integrate(real dt)
{
	// Skip integrating things with infinite mass
	if (mInverseMass > 0.0f)
	{
		Vector3 accel = mAcceleration;
		accel += mForceAccum * mInverseMass;
		mVelocity += accel * dt;
		mVelocity *= pow(mLinearDamping, dt);
		mPosition += mVelocity * dt;

		Vector3 angularAccel = mInverseInertiaTensorWorld.transform(mTorqueAccum);
		mAngularVelocity += angularAccel * dt;
		mAngularVelocity *= pow(mAngularDamping, dt);
		mOrientation.addScaledVector(mAngularVelocity, dt);

		calculateDerivedData();
	}

	clearAccumulators();
}
void Rigidbody::addForce(const Vector3& force)
{
	mForceAccum += force;
}
void Rigidbody::addTorque(const Vector3& torque)
{
	mTorqueAccum += torque;
}
void Rigidbody::addForceAtPoint(const Vector3& force, const Vector3& point)
{
	Vector3 offset = point - mPosition;
	mForceAccum += force;
	mTorqueAccum += offset % force;
}
void Rigidbody::setMass(real mass)
{
	// If mass is in the 'no-no zone' then set it to zero
	mInverseMass = (mass <= 0) ? 0 : 1.0f / mass;
}
void Rigidbody::setInertiaTensor(const Matrix3& inertiaTensor)
{
	mInverseInertiaTensor = inertiaTensor.inverse();
}
Matrix4 Rigidbody::getTransformMatrix()
{
	return mTransformMatrix;
}

void Rigidbody::clearAccumulators()
{
	mForceAccum.clear();
	mTorqueAccum.clear();
}
void Rigidbody::calculateDerivedData() 
{
	mOrientation.normalise();

	mTransformMatrix.setOrientationAndPos(mOrientation, mPosition);

	calculateInertiaTensor();
}

void Rigidbody::calculateInertiaTensor()
{
	real t4 = mTransformMatrix.data[0] * mInverseInertiaTensor.data[0] + mTransformMatrix.data[1] * mInverseInertiaTensor.data[3] + mTransformMatrix.data[2] * mInverseInertiaTensor.data[6];
	real t9 = mTransformMatrix.data[0] * mInverseInertiaTensor.data[1] + mTransformMatrix.data[1] * mInverseInertiaTensor.data[4] + mTransformMatrix.data[2] * mInverseInertiaTensor.data[7];
	real t14 = mTransformMatrix.data[0] * mInverseInertiaTensor.data[2] + mTransformMatrix.data[1] * mInverseInertiaTensor.data[5] + mTransformMatrix.data[2] * mInverseInertiaTensor.data[8];
	real t28 = mTransformMatrix.data[4] * mInverseInertiaTensor.data[0] + mTransformMatrix.data[5] * mInverseInertiaTensor.data[3] + mTransformMatrix.data[6] * mInverseInertiaTensor.data[6];
	real t33 = mTransformMatrix.data[4] * mInverseInertiaTensor.data[1] + mTransformMatrix.data[5] * mInverseInertiaTensor.data[4] + mTransformMatrix.data[6] * mInverseInertiaTensor.data[7];
	real t38 = mTransformMatrix.data[4] * mInverseInertiaTensor.data[2] + mTransformMatrix.data[5] * mInverseInertiaTensor.data[5] + mTransformMatrix.data[6] * mInverseInertiaTensor.data[8];
	real t52 = mTransformMatrix.data[8] * mInverseInertiaTensor.data[0] + mTransformMatrix.data[9] * mInverseInertiaTensor.data[3] + mTransformMatrix.data[10] * mInverseInertiaTensor.data[6];
	real t57 = mTransformMatrix.data[8] * mInverseInertiaTensor.data[1] + mTransformMatrix.data[9] * mInverseInertiaTensor.data[4] + mTransformMatrix.data[10] * mInverseInertiaTensor.data[7];
	real t62 = mTransformMatrix.data[8] * mInverseInertiaTensor.data[2] + mTransformMatrix.data[9] * mInverseInertiaTensor.data[5] + mTransformMatrix.data[10] * mInverseInertiaTensor.data[8];

	mInverseInertiaTensorWorld.data[0] = t4 * mTransformMatrix.data[0] + t9 * mTransformMatrix.data[1] + t14 * mTransformMatrix.data[2];
	mInverseInertiaTensorWorld.data[1] = t4 * mTransformMatrix.data[4] + t9 * mTransformMatrix.data[5] + t14 * mTransformMatrix.data[6];
	mInverseInertiaTensorWorld.data[2] = t4 * mTransformMatrix.data[8] + t9 * mTransformMatrix.data[9] + t14 * mTransformMatrix.data[10];
	mInverseInertiaTensorWorld.data[3] = t28 * mTransformMatrix.data[0] + t33 * mTransformMatrix.data[1] + t38 * mTransformMatrix.data[2];
	mInverseInertiaTensorWorld.data[4] = t28 * mTransformMatrix.data[4] + t33 * mTransformMatrix.data[5] + t38 * mTransformMatrix.data[6];
	mInverseInertiaTensorWorld.data[5] = t28 * mTransformMatrix.data[8] + t33 * mTransformMatrix.data[9] + t38 * mTransformMatrix.data[10];
	mInverseInertiaTensorWorld.data[6] = t52 * mTransformMatrix.data[0] + t57 * mTransformMatrix.data[1] + t62 * mTransformMatrix.data[2];
	mInverseInertiaTensorWorld.data[7] = t52 * mTransformMatrix.data[4] + t57 * mTransformMatrix.data[5] + t62 * mTransformMatrix.data[6];
	mInverseInertiaTensorWorld.data[8] = t52 * mTransformMatrix.data[8] + t57 * mTransformMatrix.data[9] + t62 * mTransformMatrix.data[10];
}