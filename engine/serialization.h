#pragma once
#include <iostream>
#include <sstream>
#include <string>
#include <bit>

#include "../cyclone/cyclone.h"

enum SerializeTypes
{
	COLLIDER,
	RIGIDBODY,
};

class BitWriter
{
public:
	BitWriter() = default;
	~BitWriter() = default;

	void writeBits(uint32_t value, int bits)
	{
		if (bits < 1 || bits > 32)
			return;

		//Shift masked value and or it into m_scrath
		mScratch |= static_cast<uint64_t>(value) << mScratchBits;

		//Add 'bits' to 'mScratchBits'
		mScratchBits += bits;

		while (mScratchBits >= 8)
		{
			//Bit mask to 8 bits
			mBuffer.push_back(static_cast<uint8_t>(mScratch & 0xFF));

			//Move over to the next 8 bits
			mScratch >>= 8;
			mScratchBits -= 8;
		}

		//Add bits to total bits
		mTotalBits += bits;
	}
	void writeBool(bool value)
	{
		writeBits(value ? 1 : 0, 1);
	}
	void flush() {
		while (mScratchBits > 0) {
			mBuffer.push_back(static_cast<uint8_t>(mScratch & 0xFF));
			mScratch >>= 8;
			mScratchBits = (mScratchBits > 8) ? (mScratchBits - 8) : 0;
		}
	}
	void writeBytes(const uint8_t* data, size_t length)
	{
		flush(); // ensure byte alignment
		mBuffer.insert(mBuffer.end(), data, data + length);
		mTotalBits += static_cast<int>(length * 8);
	}

	/// Pointer to the serialized byte buffer (valid after flush).
	const uint8_t* data() const { return mBuffer.data(); }

	/// Number of bytes written (valid after flush).
	size_t size() const { return mBuffer.size(); }

	/// Total number of bits written so far.
	size_t bitsWritten() const { return mTotalBits; }

private:
	std::vector<uint8_t> mBuffer;
	uint64_t mScratch = 0;
	int mScratchBits = 0;       
	int mTotalBits = 0;           
};
class BitReader
{
public:
    BitReader(const uint8_t* data, size_t size)
        : mData(data), mSize(size) {
    }

    uint32_t readBits(int bits) {
        if (bits < 1 || bits > 32)
            return;

        while (mScratchBits < bits && mByteOffset < mSize)
        {
            //Read the next byte from `mData[mByteOffset]`.
            //OR it into `mScratch`, shifted LEFT by `mScratchBits`.
			mScratch |= static_cast<uint64_t>(mData[mByteOffset++]) << mScratchBits;

            //Increment `mByteOffset` and add 8 to `mScratchBits`.
			mScratchBits += 8;
        }

        //Mask the lowest `bits` bits of `mScratch` — that's the result.
        uint32_t result = mScratch & ((1ULL << bits) - 1);

        //Right-shift `mScratch` by `bits` and subtract from `mScratchBits`.
		mScratch >>= bits;
		mScratchBits -= bits;

        //Add `bits` to `mTotalBitsRead`.
		mTotalBitsRead += bits;

        //Return the result.
        return result;
    }

    /**
     * Read a single boolean (1 bit).
     */
    bool readBool() {
        return readBits(1) != 0;
    }

    /**
     * Read raw bytes from the buffer (for string data, etc.).
     * Discards any remaining bits in scratch to re-align to byte boundary.
     */
    void readBytes(uint8_t* out, size_t len) {
        // Discard remaining scratch bits to align to byte boundary
		mScratch = 0;
		mScratchBits = 0;
        for (size_t i = 0; i < len && mByteOffset < mSize; ++i) {
            out[i] = mData[mByteOffset++];
        }
		mTotalBitsRead += static_cast<int>(len * 8);
    }

    /// Total bits consumed so far.
    size_t bitsRead() const { return mTotalBitsRead; }

private:
    const uint8_t* mData;       ///< source byte buffer
    size_t mSize;               ///< total bytes in buffer
    size_t mByteOffset = 0;    ///< current byte read position
    uint64_t mScratch = 0;      ///< scratch register
    int mScratchBits = 0;      ///< bits available in scratch
    int mTotalBitsRead = 0;   ///< running count of all bits read
};

class Serializer
{
public:
	Serializer() = default;
	~Serializer() = default;

	void serializeRigidbody(BitWriter& writer, cyclone::Rigidbody& rigidbody)
	{
		serializeVec3(writer, rigidbody.getPos());
		serializeVec3(writer, rigidbody.getLinearVelocity());
		serializeVec3(writer, rigidbody.getAccel());

		writer.writeBits(rigidbody.getLinearDamping(), 32);
		writer.writeBits(rigidbody.getMass(), 32);

		serializeQuaternion(writer, rigidbody.getOrientation());
		serializeVec3(writer, rigidbody.getAngularVelocity());
		serializeMat3(writer, rigidbody.getInertiaTensor());

		writer.writeBits(rigidbody.getAngularDamping(), 32);

		writer.flush();
	}
	void deserializeRigidbody(BitReader& reader, cyclone::Rigidbody& rigidbody)
	{
		cyclone::Vector3 pos;
		deserializeVec3(reader, pos);
		rigidbody.setPosition(pos);

		cyclone::Vector3 linVel;
		deserializeVec3(reader, linVel);
		rigidbody.setLinearVelocity(linVel);

		cyclone::Vector3 accel;
		deserializeVec3(reader, accel);
		rigidbody.setAcceleration(accel);

		rigidbody.setLinearDaming(reader.readBits(32));
		rigidbody.setMass(reader.readBits(32));

		cyclone::Quaternion ori;
		deserializeQuaternion(reader, ori);
		rigidbody.setOrientation(ori);

		cyclone::Vector3 angVel;
		deserializeVec3(reader, angVel);
		rigidbody.setAngularVelocity(angVel);

		cyclone::Matrix3 iit;
		deserializeMat3(reader, iit);
		rigidbody.setInertiaTensor(iit);

		rigidbody.setAngularDaming(reader.readBits(32));
	}

private:
	void serializeMat4(BitWriter& writer, const cyclone::Matrix4& mat4)
	{
		for (cyclone::real n : mat4.data)
		{
			writer.writeBits(n, 32);
		}
	}
	void deserializeMat4(BitReader& reader, cyclone::Matrix4& mat4)
	{
		for (cyclone::real n : mat4.data)
		{
			n = reader.readBits(32);
		}
	}
	void serializeMat3(BitWriter& writer, const cyclone::Matrix3& mat3)
	{
		for (cyclone::real n : mat3.data)
		{
			writer.writeBits(n, 32);
		}
	}
	void deserializeMat3(BitReader& reader, cyclone::Matrix3& mat3)
	{
		for (cyclone::real n : mat3.data)
		{
			n = reader.readBits(32);
		}
	}
	void serializeVec3(BitWriter& writer, const cyclone::Vector3& vec3)
	{
		writer.writeBits(vec3.x, 32);
		writer.writeBits(vec3.y, 32);
		writer.writeBits(vec3.z, 32);
	}
	void deserializeVec3(BitReader& reader, cyclone::Vector3& vec3)
	{
		vec3.x = reader.readBits(32);
		vec3.y = reader.readBits(32);
		vec3.z = reader.readBits(32);
	}
	void serializeQuaternion(BitWriter& writer, const cyclone::Quaternion& quaternion)
	{
		writer.writeBits(quaternion.i, 32);
		writer.writeBits(quaternion.j, 32);
		writer.writeBits(quaternion.k, 32);
		writer.writeBits(quaternion.r, 32);
	}
	void deserializeQuaternion(BitReader& reader, cyclone::Quaternion& quaternion)
	{
		quaternion.i = reader.readBits(32);
		quaternion.j = reader.readBits(32);
		quaternion.k = reader.readBits(32);
		quaternion.r = reader.readBits(32);
	}
};