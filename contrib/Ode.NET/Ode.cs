using System;
using System.Runtime.InteropServices;

namespace Ode.NET
{
#if dDOUBLE
	using dReal = System.Double;
#else
	using dReal = System.Single;
#endif

	public static class d
	{
		public static dReal Infinity = dReal.MaxValue;

		#region Flags and Enumerations

		[Flags]
		public enum ContactFlags : int
		{
			Mu2 = 0x001,
			FDir1 = 0x002,
			Bounce = 0x004,
			SoftERP = 0x008,
			SoftCFM = 0x010,
			Motion1 = 0x020,
			Motion2 = 0x040,
			Slip1 = 0x080,
			Slip2 = 0x100,
			Approx0 = 0x0000,
			Approx1_1 = 0x1000,
			Approx1_2 = 0x2000,
			Approx1 = 0x3000
		}

		public enum GeomClass : int
		{
			SphereClass,
			BoxClass,
			CapsuleClass,
			CylinderClass,
			PlaneClass,
			RayClass,
			ConvexClass,
			GeomTransformClass,
			TriMeshClass,
			HeightfieldClass,
			FirstSpaceClass,
			SimpleSpaceClass = FirstSpaceClass,
			HashSpaceClass,
			QuadTreeSpaceClass,
			LastSpaceClass = QuadTreeSpaceClass,
			FirstUserClass,
			LastUserClass = FirstUserClass + MaxUserClasses - 1,
			NumClasses,
			MaxUserClasses = 4
		}

		public enum JointType : int
		{
			None,
			Ball,
			Hinge,
			Slider,
			Contact,
			Universal,
			Hinge2,
			Fixed,
			Null,
			AMotor,
			LMotor,
			Plane2D
		}

		#endregion

		#region Type Definitions

		[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
		public delegate void NearCallback(IntPtr data, IntPtr geom1, IntPtr geom2);

		[StructLayout(LayoutKind.Sequential)]
		public struct Contact
		{
			public SurfaceParameters surface;
			public ContactGeom geom;
			public Vector3 fdir1;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct ContactGeom
		{
			public static readonly int SizeOf = Marshal.SizeOf(typeof(ContactGeom));

			public Vector3 pos;
			public Vector3 normal;
			public dReal depth;
			public IntPtr g1;
			public IntPtr g2;
			public int side1;
			public int side2;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct Mass
		{
			public dReal mass;
			public Vector4 c;
			public Matrix3 I;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct Matrix3
		{
			public Matrix3(dReal m00, dReal m10, dReal m20, dReal m01, dReal m11, dReal m21, dReal m02, dReal m12, dReal m22)
			{
				M00 = m00;
				M10 = m10;
				M20 = m20;
				M30 = 0.0f;
				M01 = m01;
				M11 = m11;
				M21 = m21;
				M31 = 0.0f;
				M02 = m02;
				M12 = m12;
				M22 = m22;
				M32 = 0.0f;
			}

			public dReal M00, M10, M20, M30;
			public dReal M01, M11, M21, M31;
			public dReal M02, M12, M22, M32;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct Quaternion
		{
			public dReal W, X, Y, Z;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct SurfaceParameters
		{
			public ContactFlags mode;
			public dReal mu;
			public dReal mu2;
			public dReal bounce;
			public dReal bounce_vel;
			public dReal soft_erp;
			public dReal soft_cfm;
			public dReal motion1;
			public dReal motion2;
			public dReal slip1;
			public dReal slip2;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct Vector3
		{
			public dReal X, Y, Z, W;

			public Vector3(dReal x, dReal y, dReal z)
			{
				X = x;
				Y = y;
				Z = z;
				W = 0.0f;
			}
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct Vector4
		{
			public dReal X, Y, Z, W;

			public Vector4(dReal x, dReal y, dReal z, dReal w)
			{
				X = x;
				Y = y;
				Z = z;
				W = w;
			}
		}

		#endregion

		[DllImport("ode", EntryPoint = "dAreConnectedExcluding")]
		public static extern bool AreConnectedExcluding(IntPtr body0, IntPtr body1, JointType jointType);

		[DllImport("ode", EntryPoint = "dBodyCopyPosition")]
		public static extern void BodyCopyPosition(IntPtr body, out Vector3 pos);

		[DllImport("ode", EntryPoint = "dBodyCopyRotation")]
		public static extern void BodyCopyRotation(IntPtr body, out Matrix3 R);

		[DllImport("ode", EntryPoint = "dBodyCopyQuaternion")]
		public static extern void BodyCopyQuaternion(IntPtr body, out Quaternion quat);

		[DllImport("ode", EntryPoint = "dBodyCreate")]
		public static extern IntPtr BodyCreate(IntPtr world);

		[DllImport("ode", EntryPoint = "dBodyDestroy")]
		public static extern void BodyDestroy(IntPtr body);

		[DllImport("ode", EntryPoint = "dBodySetMass")]
		public static extern void BodySetMass(IntPtr body, ref Mass mass);

		[DllImport("ode", EntryPoint = "dBodySetPosition")]
		public static extern void BodySetPosition(IntPtr body, dReal x, dReal y, dReal z);

		[DllImport("ode", EntryPoint = "dBodySetRotation")]
		public static extern void BodySetRotation(IntPtr body, ref Matrix3 R);

		[DllImport("ode", EntryPoint = "dCloseODE")]
		public static extern void CloseODE();

		[DllImport("ode", EntryPoint = "dCollide")]
		public static extern int Collide(IntPtr o1, IntPtr o2, int flags, [In, Out] ContactGeom[] contact, int skip);

		[DllImport("ode", EntryPoint = "dCreateBox")]
		public static extern IntPtr CreateBox(IntPtr space, dReal lx, dReal ly, dReal lz);

		[DllImport("ode", EntryPoint = "dCreateCapsule")]
		public static extern IntPtr CreateCapsule(IntPtr space, dReal radius, dReal length);

		[DllImport("ode", EntryPoint = "dCreateConvex")]
		public static extern IntPtr CreateConvex(IntPtr space, dReal[] planes, int planeCount, dReal[] points, int pointCount, int[] polygons);

		[DllImport("ode", EntryPoint = "dCreatePlane")]
		public static extern IntPtr CreatePlane(IntPtr space, dReal a, dReal b, dReal c, dReal d);

		[DllImport("ode", EntryPoint = "dGeomBoxGetLengths")]
		public static extern void GeomBoxGetLengths(IntPtr geom, out Vector3 len);

		[DllImport("ode", EntryPoint = "dGeomCapsuleGetParams")]
		public static extern void GeomCapsuleGetParams(IntPtr geom, out dReal radius, out dReal length);

		[DllImport("ode", EntryPoint = "dGeomCopyPosition")]
		public static extern void GeomCopyPosition(IntPtr geom, out Vector3 pos);

		[DllImport("ode", EntryPoint = "dGeomDestroy")]
		public static extern void GeomDestroy(IntPtr geom);

		[DllImport("ode", EntryPoint = "dGeomGetBody")]
		public static extern IntPtr GeomGetBody(IntPtr geom);

		[DllImport("ode", EntryPoint = "dGeomGetClass")]
		public static extern GeomClass GeomGetClass(IntPtr geom);

		[DllImport("ode", EntryPoint = "dGeomSetBody")]
		public static extern void GeomSetBody(IntPtr geom, IntPtr body);

		[DllImport("ode", EntryPoint = "dHashSpaceCreate")]
		public static extern IntPtr HashSpaceCreate(IntPtr space);

		[DllImport("ode", EntryPoint = "dJointAttach")]
		public static extern void JointAttach(IntPtr joint, IntPtr body1, IntPtr body2);

		[DllImport("ode", EntryPoint = "dJointCreateContact")]
		public static extern IntPtr JointCreateContact(IntPtr world, IntPtr group, ref Contact contact);

		[DllImport("ode", EntryPoint = "dJointGroupCreate")]
		public static extern IntPtr JointGroupCreate(int max_size);

		[DllImport("ode", EntryPoint = "dJointGroupDestroy")]
		public static extern void JointGroupDestroy(IntPtr group);

		[DllImport("ode", EntryPoint = "dJointGroupEmpty")]
		public static extern void JointGroupEmpty(IntPtr group);

		[DllImport("ode", EntryPoint = "dMassSetBox")]
		public static extern void MassSetBox(out Mass mass, dReal density, dReal lx, dReal ly, dReal lz);

		[DllImport("ode", EntryPoint = "dMassSetCapsule")]
		public static extern void MassSetCapsule(out Mass mass, dReal density, int direction, dReal radius, dReal length);

		[DllImport("ode", EntryPoint = "dRandReal")]
		public static extern dReal RandReal();

		[DllImport("ode", EntryPoint = "dRFromAxisAndAngle")]
		public static extern void RFromAxisAndAngle(out Matrix3 R, dReal x, dReal y, dReal z, dReal angle);

		[DllImport("ode", EntryPoint = "dSpaceCollide")]
		public static extern void SpaceCollide(IntPtr space, IntPtr data, NearCallback callback);

		[DllImport("ode", EntryPoint = "dSpaceDestroy")]
		public static extern void SpaceDestroy(IntPtr space);

		[DllImport("ode", EntryPoint = "dWorldCreate")]
		public static extern IntPtr WorldCreate();

		[DllImport("ode", EntryPoint = "dWorldDestroy")]
		public static extern void WorldDestroy(IntPtr world);

		[DllImport("ode", EntryPoint = "dWorldQuickStep")]
		public static extern void WorldQuickStep(IntPtr world, dReal stepsize);

		[DllImport("ode", EntryPoint = "dWorldSetAutoDisableFlag")]
		public static extern void WorldSetAutoDisableFlag(IntPtr world, bool do_auto_disable);

		[DllImport("ode", EntryPoint = "dWorldSetCFM")]
		public static extern void WorldSetCFM(IntPtr world, dReal cfm);

		[DllImport("ode", EntryPoint = "dWorldSetContactMaxCorrectingVel")]
		public static extern void WorldSetContactMaxCorrectingVel(IntPtr world, dReal vel);

		[DllImport("ode", EntryPoint = "dWorldSetContactSurfaceLayer")]
		public static extern void WorldSetContactSurfaceLayer(IntPtr world, dReal depth);

		[DllImport("ode", EntryPoint = "dWorldSetGravity")]
		public static extern void WorldSetGravity(IntPtr world, dReal x, dReal y, dReal z);
	}
}

