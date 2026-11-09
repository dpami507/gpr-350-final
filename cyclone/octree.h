#include "core.h"

#include "collider.h"
#include <vector>

namespace cyclone
{
	class OctreeElement
	{
	public:
		OctreeElement() = default;
		~OctreeElement() = default;

		virtual void insert(Collider*) = 0;
		virtual void clear() = 0;
		virtual void queryPossiblePairs(std::vector<std::pair<Collider*, Collider*>>& pairs) = 0;

		inline void setCenter(const Vector3& center) { mCenter = center; };
		inline void setHalfSize(real size) { mHalfSize = size; };

		static OctreeElement* buildOctree(const Vector3& center, real halfWidth, int stopDepth);
	protected:
		static int dirToIndex(const Vector3& dir);

		Vector3 mCenter = { 0,0,0 };
		real mHalfSize = 0;
	};

	class OctreeNode : public OctreeElement
	{
	public:
		OctreeNode() = default;
		~OctreeNode() = default;

		void insert(Collider* collider) override;
		void clear() override;
		void queryPossiblePairs(std::vector<std::pair<Collider*, Collider*>>& pairs) override;
		void addNode(OctreeElement* node, int index);

	private:
		OctreeElement* pChildren[8];
	};

	class OctreeLeaf : public OctreeElement
	{
	public:
		OctreeLeaf() = default;
		~OctreeLeaf() = default;

		void insert(Collider* collider) override;
		void clear() override;
		void queryPossiblePairs(std::vector<std::pair<Collider*, Collider*>>& pairs) override;

	private:
		std::vector<Collider*> mColliders;
	};
}