#pragma once
#include <iostream>
#include <sstream>
#include <string>

#include "../cyclone/cyclone.h"

enum SerializeTypes
{
	COLLIDER,
	RIGIDBODY,
};

class Serializer
{
public:
	Serializer();
	~Serializer();

	static std::string serializeRigidbody(const cyclone::Rigidbody* rb)
	{
		std::stringstream output;
		output << RIGIDBODY + ' ';

		output << rb->getMass() << ' ';
		output << serializeVec3(rb->getPos()) << ' ';
		output << serializeVec3(rb->getLinearVelocity()) << ' ';
		output << serializeQuaternion(rb->getOrientation()) << ' ';

		return output.str();
	}
	static cyclone::Rigidbody& deserializeRigidbody(const std::string& str)
	{
		std::stringstream ss(str);
		std::string token;
		int indx = 0;

		cyclone::Rigidbody newRb;

		newRb.setMass(std::getline(ss, ' '))
		while (std::getline(ss, token, ' '))
		{
			newMat4.data[indx] = std::stof(token);
			indx++;
		}
	}
	static std::string serializeCollider(const cyclone::Collider& col)
	{
		std::string output;
		output += COLLIDER + ' ';
		
		output += (int)col.getType() + ' ';
		output += serializeMat4(col.getTransform()) + ' ';
		output += serializeMat4(col.getOffset()) + ' ';
		return output;
	}
	static cyclone::Collider* deserializeCollider(const std::string& str)
	{
		std::stringstream ss(str);
		std::string token;

		// read colldier type
		std::getline(ss, token, ' ');
		int type = std::stoi(token);
		switch (type)
		{
		case 0: // Sphere
			cyclone::SphereCollider* newSphere = new cyclone::SphereCollider()
			break;
		case 1: // Plane
			break;
		case 2: // Box
		default:
			break;
		}

		cyclone::Collider newCollider;
		while (std::getline(ss, token, ' '))
		{
			newMat4.data[indx] = std::stof(token);
			indx++;
		}
	}
private:
	static std::string serializeVec3(const cyclone::Vector3& vec3)
	{
		std::stringstream output;

		for (auto d : vec3.data)
			output << d + ' ';

		return output.str();
	}
	static cyclone::Vector3& deserializeVec3(const std::string& str)
	{
		std::stringstream ss(str);
		std::string token;
		int indx = 0;

		cyclone::Vector3 newVec3;
		while (std::getline(ss, token, ' '))
		{
			newVec3.data[indx] = std::stof(token);
			indx++;
		}
	}
	static std::string serializeQuaternion(const cyclone::Quaternion& quat)
	{
		std::stringstream output;

		for (auto d : quat.data)
			output << d + ' ';

		return output.str();
	}
	static cyclone::Quaternion& deserializeQuaternion(const std::string& str)
	{
		std::stringstream ss(str);
		std::string token;
		int indx = 0;

		cyclone::Quaternion newQuat;
		while (std::getline(ss, token, ' '))
		{
			newQuat.data[indx] = std::stof(token);
			indx++;
		}
	}
	static std::string serializeMat4(const cyclone::Matrix4& mat4)
	{
		std::stringstream output;

		for (auto d : mat4.data)
			output << d + ' ';

		return output.str();
	}
	static cyclone::Matrix4& deserializeMat4(const std::string& str)
	{
		std::stringstream ss(str);
		std::string token;
		int indx = 0;

		cyclone::Matrix4 newMat4;
		while (std::getline(ss, token, ' '))
		{
			newMat4.data[indx] = std::stof(token);
			indx++;
		}
	}
};

Serializer::Serializer()
{
}

Serializer::~Serializer()
{
}