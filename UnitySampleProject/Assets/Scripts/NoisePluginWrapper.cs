using UnityEngine;
using System.Collections;
using System.Runtime.InteropServices;
using System; // IntPtr


// Wrapper around the native DLL that acquires function pointers to exported functions and wraps
// calls to them.
// Uses Native.cs to handle loading and unloading of the DLL manually, so we can update the DLL
// when the Unity Editor is open.
public class NoisePluginWrapper : MonoBehaviour
{
    #region Debug Hooks
    // The block of code below is a neat trick to allow for calling into the debug console from C++
    delegate void DebugLog(string log);
    delegate void LinkDebug([MarshalAs(UnmanagedType.FunctionPtr)]IntPtr debugCal);

    private static readonly DebugLog debugLog = DebugWrapper;
    private static readonly IntPtr functionPointer = Marshal.GetFunctionPointerForDelegate(debugLog);
    private static void DebugWrapper(string log) { Debug.Log(log); }
    private static void HookDebugWrapperWithNativeCode()
    {
        Delegate fpLinkDebug = Native.GetFuncPtr<LinkDebug>(nativeLibraryPtr);
        if (fpLinkDebug != null)
            Native.InvokeFast(fpLinkDebug, functionPointer); // Let's the native plugin write into Unity console
    }
    #endregion

    static string dllName = "CurlNoise";
    static IntPtr nativeLibraryPtr;
    static Delegate fpCComputeCurlNoBoundaries;
    static Delegate fpBruteForceCurl;
    static Delegate fpNonBruteForceCurl;
    static Delegate fpSetCurlSettings;

    delegate Vector3 ComputeCurlNoBoundaries(Vector3 wsPos);
    delegate Vector3 ComputeCurlBruteForce(Vector3 p, IntPtr colliders, Int32 length);
    delegate Vector3 ComputeCurlNonBruteForce(Vector3 p, IntPtr colliders, Int32 length);
    delegate void SetCurlSettings(bool bCheapGradient,
                                  float frequency,
                                  UInt32 numOctaves,
                                  float lacunarity,
                                  float persistence);

    void Awake()
    {
        if (nativeLibraryPtr != IntPtr.Zero) return;

        string dllPath = Application.dataPath + "/" + dllName;
        nativeLibraryPtr = Native.LoadLibrary(dllPath);
        if (nativeLibraryPtr == IntPtr.Zero)
        {
            Debug.LogError("Failed to load native library at " + dllPath);
        }

        HookDebugWrapperWithNativeCode();

        GetFunctionPointersToExportedNativeMethods();
    }


    private static void GetFunctionPointersToExportedNativeMethods()
    {
        fpCComputeCurlNoBoundaries = Native.GetFuncPtr<Vector3, ComputeCurlNoBoundaries>(nativeLibraryPtr);
        fpBruteForceCurl = Native.GetFuncPtr<Vector3, ComputeCurlBruteForce>(nativeLibraryPtr);
        fpNonBruteForceCurl = Native.GetFuncPtr<Vector3, ComputeCurlNonBruteForce>(nativeLibraryPtr);
        fpSetCurlSettings = Native.GetFuncPtr<SetCurlSettings>(nativeLibraryPtr);
    }

    // Curl evaluated over 3D noise. Obstacles not considered.
    public static Vector3 ComputeCurlA(Vector3 p)
    {
        if (fpCComputeCurlNoBoundaries != null)
            return Native.InvokeFast<Vector3>(fpCComputeCurlNoBoundaries, p);
        else
        {
            Debug.LogError("Couldnt get fn ptr");
            return Vector3.zero;
        }
    }


    // Curl evaluated over 3D noise and obstacles. The brute force approach calculates 
    // a numerical derivative of the potential field even if obstacles don't affect the
    // potential at the particle's position.
    public static Vector3 ComputeCurlB(Vector3 p, IntPtr colliders, Int32 length)
    {
        if (fpBruteForceCurl != null)
            return Native.InvokeFast<Vector3>(fpBruteForceCurl, p, colliders, length);
        else
        {
            Debug.LogError("Couldnt get fn ptr");
            return Vector3.zero;
        }
    }


    // Curl evaluated over 3D noise and obstacles. The non-brute force approach checks if there are 
    // any obstacles that affect the potential field at 'p'. If not, it computes the curl using
    // analytical derivatives of the field, which involves much lesser computation that numerical 
    // derivatives
    public static Vector3 ComputeCurlC(Vector3 p, IntPtr colliders, Int32 length)
    {
        if (fpBruteForceCurl != null)
            return Native.InvokeFast<Vector3>(fpNonBruteForceCurl, p, colliders, length);
        else
        {
            Debug.LogError("Couldnt get fn ptr");
            return Vector3.zero;
        }
    }


    public static void UpdateCurlSettings(bool bCheapGradient,
                                          float frequency,
                                          int numOctaves,
                                          float lacunarity,
                                          float persistence)
    {
        if (fpSetCurlSettings != null)
            Native.InvokeFast(fpSetCurlSettings, bCheapGradient, frequency, (UInt32) numOctaves, lacunarity, persistence);
        else
        {
            Debug.LogError("Couldnt get fn ptr");
        }
    }


    void OnApplicationQuit()
    {
        if (nativeLibraryPtr == IntPtr.Zero) return;

        Debug.Log(Native.FreeLibrary(nativeLibraryPtr)
                      ? "Native library successfully unloaded."
                      : "Native library could not be unloaded.");
    }
}
