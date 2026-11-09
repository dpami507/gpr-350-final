#include "contacts.h"

using namespace cyclone;

// Functions for each type of collision
std::map<std::pair<cyclone::ColliderType, cyclone::ColliderType>, std::function<bool(Collider*, Collider*, CollisionData*)>> CollisionTests::mMap =
{
	{	std::make_pair(ColliderType::Sphere, ColliderType::Sphere), [](Collider* a, Collider* b, CollisionData* data) { return SphereSphereCollide(*static_cast<SphereCollider*>(a), *static_cast<SphereCollider*>(b), data); }},
	{	std::make_pair(ColliderType::Sphere, ColliderType::Plane), [](Collider* a, Collider* b, CollisionData* data) { return SpherePlaneCollide(*static_cast<SphereCollider*>(a), *static_cast<PlaneCollider*>(b), data); }},
	{	std::make_pair(ColliderType::Plane, ColliderType::Plane), [](Collider* a, Collider* b, CollisionData* data) { return false; } },
};

bool CollisionTests::checkCollide(Collider* c1, Collider* c2, CollisionData* data)
{
	// get both types
	auto c1Type = c1->getType();
	auto c2Type = c2->getType();

	// try and find if its in the first type
	auto it1 = mMap.find({ c1Type, c2Type });
	if (it1 != mMap.end())
		return it1->second(c1, c2, data);
	// if it doesn't swap them and check the opposite
	else {
		auto it2 = mMap.find({ c2Type, c1Type });
		if (it2 != mMap.end())
			return it2->second(c2, c1, data);
		// if it still doesn't output and error 
		else {
			std::cout << "Error: Collision between types is not set up" << '\n';
			return false;
		}
	}
}
int CollisionTests::SphereSphereCollide(const SphereCollider& a, const SphereCollider& b, CollisionData* data)
{
	// get off set and normal
	Vector3 offset = a.getAxis(3) - b.getAxis(3);
	real dist = offset.magnitude();
	real radiiSum = a.getRadius() + b.getRadius();

	// early out if there is no penetration
	if (dist >= radiiSum) return 0;

	// get penetration
	real penetration = radiiSum - dist;

	Vector3 normal = offset.unit();

	// get intersection point
	Vector3 point = a.getAxis(3) + offset * 0.5f;

	// generate contact
	Contact* contact = data->getNextContact();

	contact->mPoint = point;
	contact->mNormal = normal;
	contact->mPenetration = penetration;
	contact->mBodies[0] = a.getRigidbody();
	contact->mBodies[1] = b.getRigidbody();

	data->incrementToNext();
	return 1;
}
int CollisionTests::SpherePlaneCollide(const SphereCollider& a, const PlaneCollider& b, CollisionData* data)
{
	real distance = b.getNormal() * a.getAxis(3) - a.getRadius() - b.getOffset();

	// exit if we are far away in a galaxy far away or nearby ig
	if (distance >= 0) return 0;

	// Get Penetration and intersection point
	real penetration = -distance;
	Vector3 point = a.getAxis(3) - (b.getNormal() * distance);

	// generate contact
	Contact* contact = data->getNextContact();

	contact->mPoint = point;
	contact->mNormal = b.getNormal();
	contact->mPenetration = penetration;
	contact->mBodies[0] = a.getRigidbody();
	contact->mBodies[1] = b.getRigidbody();

	data->incrementToNext();
	return 1;
}

void CollisionData::init(unsigned int maxContacts)
{
	if (isInit) cleanup();

	mMaxContacts = maxContacts;
	mContactArray = new Contact[maxContacts];

	mContactCount = 0;
	isInit = true;
}
void CollisionData::cleanup()
{
	if (!isInit) return;
	free(mContactArray);
}
void CollisionData::incrementToNext()
{
	if (mContactCount > mMaxContacts)
	{
		std::cerr << "ERR// no more contacts left, contacts will be overridded\n";
		return;
	}
	mContactCount++;
}