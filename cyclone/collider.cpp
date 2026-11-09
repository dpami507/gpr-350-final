#include "collider.h"
#include <iostream>

using namespace cyclone;

// Functions for each type of collision
std::map<std::pair<cyclone::ColliderType, cyclone::ColliderType>, std::function<bool(Collider*, Collider*)>> Collider::mMap =
{
	{	std::make_pair(ColliderType::Sphere, ColliderType::Sphere), [](Collider* a, Collider* b) { return SphereSphereOverlap(*static_cast<SphereCollider*>(a), *static_cast<SphereCollider*>(b)); }},
	{	std::make_pair(ColliderType::Sphere, ColliderType::Plane), [](Collider* a, Collider* b) { return SpherePlaneOverlap(*static_cast<SphereCollider*>(a), *static_cast<PlaneCollider*>(b)); }},
	{	std::make_pair(ColliderType::Box, ColliderType::Sphere), [](Collider* a, Collider* b) { return BoxSphereOverlap(*static_cast<BoxCollider*>(a), *static_cast<SphereCollider*>(b)); }},
	{	std::make_pair(ColliderType::Box, ColliderType::Plane), [](Collider* a, Collider* b) { return BoxPlaneOverlap(*static_cast<BoxCollider*>(a), *static_cast<PlaneCollider*>(b)); }},
	{	std::make_pair(ColliderType::Box, ColliderType::Box), [](Collider* a, Collider* b) { return BoxBoxOverlap(*static_cast<BoxCollider*>(a), *static_cast<BoxCollider*>(b)); }},
	{	std::make_pair(ColliderType::Plane, ColliderType::Plane), [](Collider* a, Collider* b) { return false; }},
};

void Collider::updateInternals()
{
	mOverLapCount = 0;

	// translate the transform matrix by the offset to get the final pos
	if (mBody != nullptr)
		mTransform = mBody->getTransformMatrix() * mOffset;
	else
		mTransform = mOffset;
}
Vector3 Collider::getAxis(const uint8_t index) const
{
	// return the axis(index) of the transform matrix 
	return mTransform.getAxisVector(index);
}
void Collider::setOffset(const Vector3& position)
{
	mOffset.data[3] = position.x;
	mOffset.data[7] = position.y;
	mOffset.data[11] = position.z;
}
void Collider::setOrientation(const Quaternion& orientation)
{
	mOffset.setOrientation(orientation);
}

// clamping functions
float Collider::clamp(float min, float max, float value)
{
	return std::min(max, std::max(min, value));
}
Vector3 Collider::clampVec(const Vector3& min, const Vector3& max, const Vector3& point)
{
	return { clamp(min.x, max.x, point.x),
			 clamp(min.y, max.y, point.y),
			 clamp(min.z, max.z, point.z) };
}

bool Collider::checkOverlap(Collider* c1, Collider* c2)
{
	// get both types
	auto c1Type = c1->getType();
	auto c2Type = c2->getType();

	// try and find if its in the first type
	auto it1 = mMap.find({ c1Type, c2Type });
	if (it1 != mMap.end())
		return it1->second(c1, c2);
	// if it doesn't swap them and check the opposite
	else {
		auto it2 = mMap.find({ c2Type, c1Type });
		if (it2 != mMap.end())
			return it2->second(c2, c1);
		// if it still doesn't output and error 
		else {
			std::cerr << "ERR// Collision between types is not set up" << '\n';
			return false;
		}
	}
}
real BoxCollider::getSizeAlongAxis(const BoxCollider& c1, const Vector3& axis)
{
	Vector3 halfSize = c1.getHalfSize();
	real dx = halfSize.x * real_abs(axis * c1.getTransform().getAxisVector(0));
	real dy = halfSize.y * real_abs(axis * c1.getTransform().getAxisVector(1));
	real dz = halfSize.z * real_abs(axis * c1.getTransform().getAxisVector(2));
	return dx + dy + dz;
}
bool BoxCollider::overlapOnAxis(const BoxCollider& c1, const BoxCollider& c2, const Vector3& axis, const Vector3& toCenter)
{
	if (axis.squareMagnitude() < 0.01) return true;
	
	const real oneProject = getSizeAlongAxis(c1, axis);
	const real twoProject = getSizeAlongAxis(c2, axis);
	const real distance = real_abs(toCenter * axis);
	return distance <= oneProject + twoProject;
}
bool Collider::SphereSphereOverlap(const SphereCollider& s1, const SphereCollider& s2)
{
	real dist = (s1.mTransform.getAxisVector(3) - s2.mTransform.getAxisVector(3)).squareMagnitude();
	real radiiSum = s1.getRadius() + s2.getRadius();
	return dist < pow(radiiSum, 2);
}
bool Collider::SpherePlaneOverlap(const SphereCollider& s1, const PlaneCollider& p1)
{
	return s1.getRadius() > p1.getNormal() * s1.getTransform().getAxisVector(3) - p1.getOffset();
}
bool Collider::BoxSphereOverlap(const BoxCollider& c1, const SphereCollider& s1)
{
	Vector3 sphereCenter = s1.getTransform().getAxisVector(3);
	Vector3 p = c1.getTransform().transformInverse(sphereCenter);
	Vector3 nearest = clampVec(c1.getHalfSize() * -1., c1.getHalfSize(), p);
	real distance = (nearest - p).magnitude();
	return distance < s1.getRadius();
}
bool Collider::BoxPlaneOverlap(const BoxCollider& c1, const PlaneCollider& p1)
{
	return BoxCollider::getSizeAlongAxis(c1, p1.getNormal()) > p1.getNormal() * c1.getTransform().getAxisVector(3) - p1.getOffset();
}
bool Collider::BoxBoxOverlap(const BoxCollider& c1, const BoxCollider& c2)
{
	Vector3 toCenter = c1.getTransform().getAxisVector(3) - c2.getTransform().getAxisVector(3);
	#define TEST_OVERLAP(axis) BoxCollider::overlapOnAxis(c1, c2, (axis), toCenter)

	return
	(
		TEST_OVERLAP(c1.getTransform().getAxisVector(0)) &&
		TEST_OVERLAP(c1.getTransform().getAxisVector(1)) &&
		TEST_OVERLAP(c1.getTransform().getAxisVector(2)) &&
		TEST_OVERLAP(c2.getTransform().getAxisVector(0)) &&
		TEST_OVERLAP(c2.getTransform().getAxisVector(1)) &&
		TEST_OVERLAP(c2.getTransform().getAxisVector(2)) &&
		TEST_OVERLAP(c1.getTransform().getAxisVector(0) % c2.getTransform().getAxisVector(0)) &&
		TEST_OVERLAP(c1.getTransform().getAxisVector(0) % c2.getTransform().getAxisVector(1)) &&
		TEST_OVERLAP(c1.getTransform().getAxisVector(0) % c2.getTransform().getAxisVector(2)) &&
		TEST_OVERLAP(c1.getTransform().getAxisVector(1) % c2.getTransform().getAxisVector(0)) &&
		TEST_OVERLAP(c1.getTransform().getAxisVector(1) % c2.getTransform().getAxisVector(1)) &&
		TEST_OVERLAP(c1.getTransform().getAxisVector(1) % c2.getTransform().getAxisVector(2)) &&
		TEST_OVERLAP(c1.getTransform().getAxisVector(2) % c2.getTransform().getAxisVector(0)) &&
		TEST_OVERLAP(c1.getTransform().getAxisVector(2) % c2.getTransform().getAxisVector(1)) &&
		TEST_OVERLAP(c1.getTransform().getAxisVector(2) % c2.getTransform().getAxisVector(2))
	);
}