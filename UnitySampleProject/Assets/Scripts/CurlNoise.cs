using UnityEngine;
using System.Runtime.InteropServices;
using System;

// This script when attached to a game object containing a Particle System component,
// updates the velocities of each of the particles based on the curl of a potential (force)
// field and obstacles in the immediate vicinity of the particle.
// 
[RequireComponent(typeof(ParticleSystem))]
public class CurlNoise : MonoBehaviour {
    public Transform[] m_Spheres;
    public bool m_NoCurl;
    public bool m_CurlWithoutBoundaries;
    public bool m_BruteForce;
    public GameObject m_CurlVolume;

	[Range(0f, 1f)]
	public float strength = 1f;
	
	public float frequency = 5f;

    [Range(1, 8)]
    public int octaves = 1;

    [Range(1f, 4f)]
    public float lacunarity = 2f;

    [Range(0f, 1f)]
    public float persistence = 0.5f;

    private ParticleSystem system;
	private ParticleSystem.Particle[] particles;

    IntPtr pColliders;

    [Serializable]
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    struct Volume
    {
        public enum eShape { kSphere, kBox, kCylinder };

        [MarshalAs(UnmanagedType.U4, SizeConst = 4)]
        public eShape mShape;
        public Vector3 mExtent;
        public Matrix4x4 mWorldToObject;
    }


    //[Serializable]
    //[StructLayout(LayoutKind.Sequential, Pack = 1)]
    //struct Vector3Serial
    //{        
    //    float x, y, z;

    //    public Vector3Serial(Vector3 v)
    //    {
    //        x = v.x; y = v.y; z = v.z;
    //    }
    //}


    //[Serializable]
    //[StructLayout(LayoutKind.Sequential, Pack = 1)]
    //struct Vector4Serial
    //{
    //    float x, y, z, w;

    //    public Vector4Serial(Vector4 v)
    //    {
    //        x = v.x;
    //        y = v.y;
    //        z = v.z;
    //        w = v.w;
    //    }       

    //}

    //[Serializable]
    //[StructLayout(LayoutKind.Sequential, Pack = 1)]
    //struct Matrix4Serial
    //{
    //    Vector4Serial col0;
    //    Vector4Serial col1;
    //    Vector4Serial col2;
    //    Vector4Serial col3;

    //    public Matrix4Serial(Matrix4x4 m)
    //    {
    //        col0 = new Vector4Serial(m.GetColumn(0));
    //        col1 = new Vector4Serial(m.GetColumn(1));
    //        col2 = new Vector4Serial(m.GetColumn(2));
    //        col3 = new Vector4Serial(m.GetColumn(3));
    //    }
    //}

    void Awake()
    {
        // TODO extend to other primitives
        Int32 numColliders = m_Spheres.Length;
        pColliders = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(Volume)) * numColliders);
    }

    private void LateUpdate () {
		if (system == null) {
			system = GetComponent<ParticleSystem>();
            system.simulationSpace = ParticleSystemSimulationSpace.World;

        }
		if (particles == null || particles.Length < system.maxParticles) {
			particles = new ParticleSystem.Particle[system.maxParticles];
		}
		int particleCount = system.GetParticles(particles);

        if (!m_NoCurl)
        {
            Bounds b = system.GetComponent<ParticleSystemRenderer>().bounds;

            if (b.Intersects(m_CurlVolume.GetComponent<Renderer>().bounds))
            {
                NoisePluginWrapper.UpdateCurlSettings(false, frequency, octaves, lacunarity, persistence);
                UpdateParticleVelocity();
                system.SetParticles(particles, particleCount);
            }            
        }        		
    }


    private void UpdateParticleVelocity()
    {
        Int32 numColliders = m_Spheres.Length;
        IntPtr ptr = pColliders;

        foreach (Transform t in m_Spheres)
        {
            Volume obstacle = new Volume();
            
            obstacle.mShape = Volume.eShape.kSphere;
            obstacle.mExtent = t.lossyScale * 0.5f;
            obstacle.mWorldToObject = Matrix4x4.TRS(t.position, t.rotation, Vector3.one);

            Marshal.StructureToPtr(obstacle, ptr, false);
            //Debug.Log("Marshal.SizeOf(typeof(Volume) = " + Marshal.SizeOf(typeof(Volume)));
            ptr = (IntPtr) ((int)ptr + Marshal.SizeOf(typeof(Volume)));
        }


        for (int i = 0; i < particles.Length; i++)
        {
            Vector3 ppos = particles[i].position;

            Vector3 p_to_c = ppos - m_CurlVolume.transform.position;
            float r = m_CurlVolume.transform.lossyScale.x * 0.5f;
            if (p_to_c.sqrMagnitude <  r * r)
            {
                Vector3 v = new Vector3();

                if (m_CurlWithoutBoundaries)
                {
                    Profiler.BeginSample("CC-NoBoundaries");
                    Vector3 curl = NoisePluginWrapper.ComputeCurlA(ppos);
                    v = strength * curl;
                    Profiler.EndSample();
                }
                else
                {
                    Profiler.BeginSample("CC-YesBoundaries");
                    if (m_BruteForce)
                    {
                        Vector3 curl = NoisePluginWrapper.ComputeCurlB(ppos, pColliders, numColliders);
                        v = strength * curl;
                    }
                    else
                    {
                        Vector3 curl = NoisePluginWrapper.ComputeCurlC(ppos, pColliders, numColliders);
                        v = strength * curl;
                        //particles[i].velocity = new Vector3(0f, 0.5f, 0f);
                    }
                    Profiler.EndSample();
                }

                particles[i].velocity = new Vector3(0f, 0.04f, 0f) + v;
            }
        }
    }  

    void OnDrawGizmos()
    {
        Gizmos.DrawWireSphere(m_CurlVolume.transform.position, m_CurlVolume.transform.lossyScale.x * 0.5f);
    }
}