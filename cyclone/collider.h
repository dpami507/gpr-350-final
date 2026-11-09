#pragma once
#include "rigidbody.h"
#include "core.h"
#include <map>
#include <functional>

namespace cyclone
{
	class Collider;
	class SphereCollider;
	class BoxCollider;
	class PlaneCollider;

	enum class ColliderType
	{
		None = -1,
		Sphere = 0,
		Plane = 1,
		Box = 2
	};
	class Collider
	{
	public:
		inline Collider(const ColliderType& type, Rigidbody* body, const Matrix4& offset = Matrix4()) :
			mType(type), mBody(body), mOffset(offset), mOverLapCount(0) { }
		~Collider() = default;

		void updateInternals();

		// getters
		Vector3 getAxis(const uint8_t index) const;
		inline Matrix4 getTransform() const { return mTransform; };
		inline Matrix4 getOffset() const { return mOffset; };
		inline ColliderType getType() const { return mType; };
		int getOverlapCount() const { return mOverLapCount; };

		Rigidbody* getRigidbody() const { return mBody; };
		bool hasRigidbody() const { return mBody != nullptr; };
		// setters
		void setOffset(const Vector3& position);
		void setOrientation(const Quaternion& orientation);

		inline void incrementOverlapCount() { mOverLapCount++; };
		virtual real getBoundingLength() = 0;
		static bool checkOverlap(Collider* c1, Collider* c2);

		static bool SphereSphereOverlap(const SphereCollider& a, const SphereCollider& b);
		static bool SpherePlaneOverlap(const SphereCollider& a, const PlaneCollider& b);
		static bool BoxSphereOverlap(const BoxCollider& a, const SphereCollider& b);
		static bool BoxPlaneOverlap(const BoxCollider& a, const PlaneCollider& b);
		static bool BoxBoxOverlap(const BoxCollider& c1, const BoxCollider& c2);

	protected:
		Rigidbody* mBody;
		Matrix4 mOffset;
		Matrix4 mTransform;

		ColliderType mType;
		int mOverLapCount;

		static float clamp(float min, float max, float value);
		static Vector3 clampVec(const Vector3& min, const Vector3& max, const Vector3& point);
	private:
		static std::map<std::pair<cyclone::ColliderType, cyclone::ColliderType>, std::function<bool(Collider*, Collider*)>> mMap;
	};

	class SphereCollider : public Collider
	{
	public:
		inline SphereCollider(Rigidbody* body, const Matrix4& offset, real radius)
			: mRadius(radius), Collider(ColliderType::Sphere, body, offset) { }
		~SphereCollider() = default;

		// getter
		inline real getRadius() const { return mRadius; };
		// setter
		inline void setRadius(real radius) { mRadius = radius; };

		inline real getBoundingLength() override {
			return mRadius;
		}

	private:
		real mRadius;
	};
	class BoxCollider : public Collider
	{
	public:
		inline BoxCollider(Rigidbody* body, const Matrix4& offset, const Vector3& size)
			: mHalfSize(size * 0.5f), Collider(ColliderType::Box, body, offset) { }
		~BoxCollider() = default;

		// getters
		inline Vector3 getSize() const { return mHalfSize * 2.f; };
		inline Vector3 getHalfSize() const { return mHalfSize; };
		//setter
		inline void setSize(const Vector3& size) { mHalfSize = size * 0.5; };

		inline real getBoundingLength() override {
			return sqrt(mHalfSize.x * mHalfSize.x + mHalfSize.y * mHalfSize.y + mHalfSize.z * mHalfSize.z);
		}

	private:
		Vector3 mHalfSize;

		static real getSizeAlongAxis(const BoxCollider& c1, const Vector3& axis);
		static bool overlapOnAxis(const BoxCollider& c1, const BoxCollider& c2, const Vector3& axis, const Vector3& toCenter);

		friend Collider;
	};
	class PlaneCollider : public Collider
	{
	public:
		inline PlaneCollider(Rigidbody* body, const real& offset, const Vector3& normal)
			: Collider(ColliderType::Plane, body)
		{
			setNormal(normal);
			setOffset(offset);
			setOrientation(cyclone::normalToQuaternion(normal));
		}
		~PlaneCollider() = default;

		inline real getBoundingLength() override {
			return 9999999; // the plane is infinite
		}

		// getters
		inline Vector3 getNormal() const { return mNormal; };
		inline real getOffset() const { return mOffset; };
		// setters
		inline void setOffset(const real& offset) 
		{
			mOffset = offset;
			Collider::setOffset(mNormal * offset);
		};
		inline void setNormal(const Vector3& normal) 
		{ 
			mNormal = normal.unit(); 
		};

	private:
		Vector3 mNormal;
		real mOffset;
	};
}