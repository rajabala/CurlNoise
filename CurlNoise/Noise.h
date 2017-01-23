#pragma once
#include "vmath.hpp"

namespace CurlNoise
{
	using namespace Vectormath::Aos;

	//-----------------------------------------------------------------------------------------------
	// Return structure for noise sampling containing scalar float and 3D derivative
	struct NoiseSample
	{
	public:
		float value;
		Vector3 derivative;

		NoiseSample& operator+= (const NoiseSample& s)
		{
			value += s.value;
			derivative += s.derivative;
			return *this;
		}
	};

	inline NoiseSample operator+(NoiseSample a, float b) {
		a.value += b;
		return a;
	}

	inline NoiseSample operator + (float a, NoiseSample b) {
		b.value += a;
		return b;
	}

	inline NoiseSample operator + (NoiseSample a, NoiseSample b) {
		a.value += b.value;
		a.derivative += b.derivative;
		return a;
	}

	inline NoiseSample operator - (NoiseSample a, float b) {
		a.value -= b;
		return a;
	}

	inline NoiseSample operator - (float a, NoiseSample b) {
		b.value = a - b.value;
		b.derivative = -b.derivative;
		return b;
	}

	inline NoiseSample operator - (NoiseSample a, NoiseSample b) {
		a.value -= b.value;
		a.derivative -= b.derivative;
		return a;
	}

	inline NoiseSample operator * (NoiseSample a, float b) {
		a.value *= b;
		a.derivative *= b;
		return a;
	}

	inline NoiseSample operator * (float a, NoiseSample b) {
		b.value *= a;
		b.derivative *= a;
		return b;
	}

	inline NoiseSample operator * (NoiseSample a, NoiseSample b) {
		a.derivative = a.derivative * b.value + b.derivative * a.value;
		a.value *= b.value;
		return a;
	}

	
	//-----------------------------------------------------------------------------------------------
	struct PerlinNoise3
	{
	public:
		static NoiseSample Noise( const Vector3& point,
							 	              float frequency, 
								              int octaves, 
								              float lacunarity, 
								              float persistence);

    static NoiseSample EvaluateNoise(const Vector3& point, float frequency);

	protected:
		inline static float Dot(const Vector3& g, float x, float y, float z) {
			return g.getX() * x + g.getY() * y + g.getZ() * z;
		}

		inline static float Smooth(float t) {
			// 6t^5 - 15t^4 + 10t^3
			return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
		}

		inline static float SmoothDerivative(float t) {
			// 30t^4 - 60t^3 + 30t^2
			return 30.0f * t * t * (t * (t - 2.0f) + 1.0f);
		}
	};	

} // namespace CurlNoise