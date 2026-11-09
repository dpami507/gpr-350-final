#include "octree.h"

#include "contacts.h"

namespace cyclone
{
	OctreeElement* OctreeElement::buildOctree(const Vector3& center, real halfWidth, int stopDepth)
	{
		OctreeNode* newTree = new OctreeNode();
		newTree->setCenter(center);
		newTree->setHalfSize(halfWidth);

		if (stopDepth <= 0)
		{
			for (int i = 0; i < 8; i++)
				newTree->addNode(new OctreeLeaf(), i);
		}
		else
		{
			real newHalfWidth = halfWidth / 2;
			int newStop = stopDepth - 1;

			newTree->addNode(buildOctree(center + Vector3(newHalfWidth, newHalfWidth, newHalfWidth), newHalfWidth, newStop), 0);
			newTree->addNode(buildOctree(center + Vector3(-newHalfWidth, newHalfWidth, newHalfWidth), newHalfWidth, newStop), 1);
			newTree->addNode(buildOctree(center + Vector3(newHalfWidth, -newHalfWidth, newHalfWidth), newHalfWidth, newStop), 2);
			newTree->addNode(buildOctree(center + Vector3(-newHalfWidth, -newHalfWidth, newHalfWidth), newHalfWidth, newStop), 3);
			newTree->addNode(buildOctree(center + Vector3(newHalfWidth, newHalfWidth, -newHalfWidth), newHalfWidth, newStop), 4);
			newTree->addNode(buildOctree(center + Vector3(-newHalfWidth, newHalfWidth, -newHalfWidth), newHalfWidth, newStop), 5);
			newTree->addNode(buildOctree(center + Vector3(newHalfWidth, -newHalfWidth, -newHalfWidth), newHalfWidth, newStop), 6);
			newTree->addNode(buildOctree(center + Vector3(-newHalfWidth, -newHalfWidth, -newHalfWidth), newHalfWidth, newStop), 7);
		}

		return newTree;
	}
	int OctreeElement::dirToIndex(const Vector3& dir)
	{
		int x = (int)(dir.x >= 0);
		int y = (int)(dir.y >= 0);
		int z = (int)(dir.z >= 0);
		uint8_t idx = x | (y << 1) | (z << 2);

		switch (idx)
		{
		case (7): return 0;
		case (6): return 1;
		case (5): return 2;
		case (4): return 3;
		case (3): return 4;
		case (2): return 5;
		case (1): return 6;
		case (0): return 7;
		default: return 0; break;
		}
	}

	void OctreeNode::insert(Collider* collider)
	{
		Vector3 colliderPos = collider->getAxis(3);

		int index = OctreeElement::dirToIndex(colliderPos - mCenter);
		pChildren[index]->insert(collider);

		// Check for overlap between neighboring grids
		Vector3 axis[3] = {
			{1,0,0}, {0,1,0}, {0,0,1}
		};
		Vector3 tempColliderPos;
		int tempIndex;
		for (const Vector3& a : axis)
		{
			// check negative axis
			tempColliderPos = colliderPos + (a * -collider->getBoundingLength());
			tempIndex = OctreeElement::dirToIndex(tempColliderPos - mCenter);
			if (index != tempIndex)
				pChildren[tempIndex]->insert(collider);

			// check positive axis
			tempColliderPos = colliderPos + (a * collider->getBoundingLength());
			tempIndex = OctreeElement::dirToIndex(tempColliderPos - mCenter);
			if (index != tempIndex)
				pChildren[tempIndex]->insert(collider);
		}
	}
	void OctreeNode::clear()
	{
		for (OctreeElement* c : pChildren)
		{
			c->clear();
		}
	}
	void OctreeNode::queryPossiblePairs(std::vector<std::pair<Collider*, Collider*>>& pairs)
	{
		for (OctreeElement* c : pChildren)
		{
			c->queryPossiblePairs(pairs);
		}
	}
	void OctreeNode::addNode(OctreeElement* node, int index)
	{
		if (index < 0 || index > 7)
			return;

		pChildren[index] = node;
	}

	void OctreeLeaf::insert(Collider* collider)
	{
		mColliders.push_back(collider);
	}
	void OctreeLeaf::clear()
	{
		mColliders.clear();
	}
	void OctreeLeaf::queryPossiblePairs(std::vector<std::pair<Collider*, Collider*>>& pairs)
	{
		for (int i = 0; i < mColliders.size(); i++)
		{
			for (int j = i + 1; j < mColliders.size(); j++)
			{
				real rs = mColliders[i]->getBoundingLength() + mColliders[j]->getBoundingLength();
				if (rs * rs > (mColliders[i]->getAxis(3) - mColliders[j]->getAxis(3)).squareMagnitude())
				{
					pairs.push_back({ mColliders[i], mColliders[j] });
				}
			}
		}
	}
}