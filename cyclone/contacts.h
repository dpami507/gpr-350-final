#pragma once
#include "rigidbody.h"
#include "core.h"
#include "collider.h"

#include <map>
#include <functional>

namespace cyclone
{
	class Contact;
	class CollisionData;
	class CollisionTests;


	class Contact
	{
		friend class ContactResolver;
	public:
		Rigidbody* mBodies[2];
		Vector3 mPoint = { 0,0,0 };
		Vector3 mNormal = { 0,0,0 };
		real mPenetration = 0;
		// needed here for math
		real mCoeffRest;

	protected:
		// swap the two rigidbodies when it is called.
		void swapBodies();

		// calculate the internals of the contact from the data in here already. should be called before any resolution happens. never called manually.
		void calculateInternals(real deltaTime);
		// calculates and sets the internal value for the desired delta velocity
		void calculateDesiredDeltaVelocity(real deltaTime);
		// calculates and returns the velocity of the contact point on the given body
		Vector3 calculateLocalVelocity(unsigned bodyIndex, real deltaTime);
		// calculates the orthonormal basis of the contact point based on the primary friction direction or random orientation
		void calculateContactBasis();
		// calculates the impulse neede to resolve the contact given there is no friction. to save time, we pass in an array pointer of both inverse inertia tensors
		Vector3 calculateFrictionlessImpulse(Matrix3* inverseInertiaTensors);

		// performs an inertia weighted impulse based resolution of this contact by itself
		void applyVelocityChange(Vector3 velChange[2], Vector3 rotChange[2]);
		// performs an inertia weighted penetration resolution of this contact by itself
		void applyPositionChange(Vector3 linearChange[2], Vector3 angularChange[2], real penetration);

	//protected:
		// A transform matrix that converts coords in the contact's frame of refrence to world coords. the columns of this matrix form an orthonormal set of vectors.
		Matrix3 contactToWorld;

		// holds the closing velocity at the point of contact, set when calculateInternals is ran.
		Vector3 contactVelocity;

		// holds the required change in velocity for this contact to be resolved
		real desiredDeltaVel;

		// relative contact positions are set when the calculateInternals function is ran.
		Vector3 relativeContactPositions[2];
	}; 
	class CollisionData {
	public:
		void init(unsigned int maxContacts);
		void cleanup();
		inline void setToStart() { mContactCount = 0; };
		inline void setCoeffRestitution(real newCoeff) { mCoeffRestitution = newCoeff; };

		inline real getCoeffRestitution() const { return mCoeffRestitution; };
		inline unsigned int getCount() const { return mContactCount; };
		inline Contact* getArray() const { return mContactArray; };
		inline Contact* getNextContact() const { return mContactArray + mContactCount; };
		void incrementToNext();
	private:
		Contact* mContactArray;	//Start of contact array
		int mContactCount;		//Amount of contacts found so far
		int mMaxContacts;		//Amount of contacts remaining
		bool isInit = false;
		// for giving the contacts the coeff of restitution
		real mCoeffRestitution;
	};
	class CollisionTests
	{
	public:
		static bool checkCollide(Collider* c1, Collider* c2, CollisionData* data);
		static int SphereSphereCollide(const SphereCollider& a, const SphereCollider& b, CollisionData* data);
		static int SpherePlaneCollide(const SphereCollider& a, const PlaneCollider& b, CollisionData* data);
	private:
		CollisionTests() = default;
		~CollisionTests() = default;

		static std::map<std::pair<cyclone::ColliderType, cyclone::ColliderType>, std::function<bool(Collider*, Collider*, CollisionData*)>> mMap;
	};
}