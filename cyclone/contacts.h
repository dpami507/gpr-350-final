#pragma once
#include "rigidbody.h"
#include "core.h"
#include "collider.h"

#include <map>
#include <functional>

namespace cyclone
{
	struct Contact
	{
		Rigidbody* mBodies[2];
		Vector3 mPoint = { 0,0,0 };
		Vector3 mNormal = { 0,0,0 };
		real mPenetration = 0;
	};
	class CollisionData {
	public:
		void init(unsigned int maxContacts);
		void cleanup();
		inline void setToStart() { mContactCount = 0; };

		inline unsigned int getCount() const { return mContactCount; };
		inline Contact* getArray() const { return mContactArray; };
		inline Contact* getNextContact() const { return mContactArray + mContactCount; };
		void incrementToNext();
	private:
		Contact* mContactArray;	//Start of contact array
		int mContactCount;		//Amount of contacts found so far
		int mMaxContacts;		//Amount of contacts remaining
		bool isInit = false;
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