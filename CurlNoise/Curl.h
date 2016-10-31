#pragma once

#include "vmath.hpp" // PORTME: vmath can be replaced with IG's internal math equivalents
#define USING_UNITY 1

#if USING_UNITY
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT 
#endif

namespace CurlNoise
{
	using namespace Vectormath::Aos;

	// This class encapsulates different volume collider primitives for the purpose of distance field estimation
	// It does NOT handle irregular scaling for curved primitives (for ex, ellipsoids, ellipsoid cylinders..) at the moment.
	// PORTME: IG Core already defines different Volume types. It seems to have a distance field function GetSignedDistancePoint(..)
	//         It handles all the primitive shapes defined below, but approximates some of them 
	//		   (for ex, box distance is Manhattan distance to the corner point)
	//		   I'd guess that being tied to SceneObject, that would end up being shared with the vfx system.
	//		   This is just a place holder for proof of concept and testing.	
	class Volume
	{
		enum eShape
		{
			kSphere,
			kCylinder,
			kBox,

			kCount
		};

	public:
		// constructor overloads for various primitive types
		Volume(const Matrix4& worldToObjectNoScale, float r); // sphere
		Volume(const Matrix4& worldToObjectNoScale, const Vector3& extents); // box
		Volume(const Matrix4& worldToObjectNoScale, float r, float h); // cylinder

		float DistanceToSurface(const Vector3& wsPos) const;
		inline void SetWorldToObjectTransform(const Matrix4& m);
		inline Vector3 GetWorldPos() const;

	protected:
		eShape m_Shape;
		Vector3 m_Extents; // world space units; interpreted based on the shape
		Matrix4 m_WorldToObject; // inverse of the object to world matrix consisting of only translation and rotation
	};


	struct CurlSettings
	{
		bool			m_bCheapGradient = false;
		float			m_Frequency = 1.f;
		unsigned int	m_NumOctaves = 1;
		float			m_Lacunarity = 0.f;
		float			m_Persistence = 0.f;		
	};

#if USING_UNITY
	extern "C"
	{		
		struct float3
		{
			float val[3];		
		};
		
		DLLEXPORT float3 ComputeCurlBruteForce(Vector3 wsPos, Volume *pColliders, unsigned int length);

		DLLEXPORT float3 ComputeCurlNoBoundaries(Vector3 wsPos);

		DLLEXPORT float3 ComputeCurlNonBruteForce(Vector3 wsPos, Volume *pColliders, unsigned int length);

		DLLEXPORT void SetCurlSettings(const CurlSettings& settings);

	}
#else
	// API for visual effect system to use
	Vector3 ComputeCurl(Vector3 wsPos, const Volume *pColliders, unsigned int length);
	Vector3 ComputeCurlWithoutObstacles(Vector3 wsPos);
	void SetCurlSettings(const CurlSettings& settings);
#endif

};