using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using StillDesign.PhysX;
using StillDesign.PhysX.MathPrimitives;

namespace SLP
{
    /// <summary>
    /// BulletX用Vector
    /// </summary>
    public struct btVector3
    {
        static public btVector3 Zero = new btVector3(Vector3.Zero);

        public float X;
        public float Y;
        public float Z;
        public float W;

        public btVector3(float x, float y, float z)
        {
            X = x;
            Y = y;
            Z = z;
            W = 0;
        }

        public btVector3(Vector3 vec)
        {
            X = vec.X;
            Y = vec.Y;
            Z = vec.Z;
            W = 0;
        }

        public btVector3(float[] x)
        {
            X = x[0];
            Y = x[1];
            Z = x[2];
            W = 0;
        }

        public Vector3 getVector3()
        {
            return new Vector3(X, Y, Z);
        }

        public float dot(btVector3 v)
        {
            return X * v.X + Y * v.Y + Z * v.Z;
        }

        public static btVector3 operator -(btVector3 value)
        {
            return new btVector3(-value.X, -value.Y, -value.Z);
        }
    }

    /// <summary>
    /// BulletX用Matrix
    /// </summary>
    public struct btTransform
    {
        public static btTransform Identity { get { return new btTransform(btMatrix3x3.Identity); } }

        public btMatrix3x3 Basis;
        public btVector3 Origin;
        Matrix matrix;

        public btTransform(btMatrix3x3 b)
        {
            Basis = b;
            Origin = new btVector3();
            matrix = new Matrix();
        }

        public btTransform(btMatrix3x3 b, btVector3 c)
        {
            Basis = b;
            Origin = c;
            matrix = new Matrix();
        }

        public Matrix getMatrix()
        {
            matrix.M11 = Basis.el0.X;
            matrix.M21 = Basis.el0.Y;
            matrix.M31 = Basis.el0.Z;
            matrix.M12 = Basis.el1.X;
            matrix.M22 = Basis.el1.Y;
            matrix.M32 = Basis.el1.Z;
            matrix.M13 = Basis.el2.X;
            matrix.M23 = Basis.el2.Y;
            matrix.M33 = Basis.el2.Z;
            matrix.M41 = Origin.X;
            matrix.M42 = Origin.Y;
            matrix.M43 = Origin.Z;
            return matrix;
        }

        public void setMatrix(Matrix m)
        {
            Basis.el0.X = m.M11;
            Basis.el0.Y = m.M21;
            Basis.el0.Z = m.M31;
            Basis.el1.X = m.M12;
            Basis.el1.Y = m.M22;
            Basis.el1.Z = m.M32;
            Basis.el2.X = m.M13;
            Basis.el2.Y = m.M23;
            Basis.el2.Z = m.M33;
            Origin.X = m.M41;
            Origin.Y = m.M42;
            Origin.Z = m.M43;
        }

        public btTransform inverse()
        {
            btMatrix3x3 inv;
            Basis.transpose(out inv);
            btVector3 origin, temp;
            temp = -Origin;
            btMatrix3x3.Multiply(ref inv, ref temp, out origin);
            return new btTransform(inv, origin);
        }

        public static btVector3 operator *(btTransform t, btVector3 x)
        {
            return new btVector3(t.Basis.el0.dot(x) + t.Origin.X,
                    t.Basis.el1.dot(x) + t.Origin.Y,
                    t.Basis.el2.dot(x) + t.Origin.Z);
        }

        static public btTransform operator *(btTransform value1, btTransform t2)
        {
            btMatrix3x3 temp;
            btMatrix3x3.Multiply(ref value1.Basis, ref t2.Basis, out temp);
            return new btTransform(temp, value1 * t2.Origin);
        }
    }

    /// <summary>
    /// BulletX用Matrix
    /// </summary>
    public struct btMatrix3x3
    {
        public btVector3 el0;
        public btVector3 el1;
        public btVector3 el2;

        public static btMatrix3x3 Identity
        {
            get
            {
                return new btMatrix3x3(1, 0, 0,
                                        0, 1, 0,
                                        0, 0, 1);
            }
        }

        public btMatrix3x3(float xx, float xy, float xz,
            float yx, float yy, float yz,
            float zx, float zy, float zz)
        {
            el0 = new btVector3(xx, xy, xz);
            el1 = new btVector3(yx, yy, yz);
            el2 = new btVector3(zx, zy, zz);
        }

        public static void Multiply(ref btMatrix3x3 m, ref btVector3 v, out btVector3 result)
        {
            result.X = m.el0.dot(v);
            result.Y = m.el1.dot(v);
            result.Z = m.el2.dot(v);
            result.W = 0;
        }

        public static void Multiply(ref btMatrix3x3 m1, ref btMatrix3x3 m2, out btMatrix3x3 result)
        {
            result.el0.X = m2.tdotx(ref m1.el0);
            result.el0.Y = m2.tdoty(ref m1.el0);
            result.el0.Z = m2.tdotz(ref m1.el0);
            result.el0.W = 0;
            result.el1.X = m2.tdotx(ref m1.el1);
            result.el1.Y = m2.tdoty(ref m1.el1);
            result.el1.Z = m2.tdotz(ref m1.el1);
            result.el1.W = 0;
            result.el2.X = m2.tdotx(ref m1.el2);
            result.el2.Y = m2.tdoty(ref m1.el2);
            result.el2.Z = m2.tdotz(ref m1.el2);
            result.el2.W = 0;
        }

        public float tdotx(ref btVector3 v)
        {
            return el0.X * v.X + el1.X * v.Y + el2.X * v.Z;
        }

        public float tdoty(ref btVector3 v)
        {
            return el0.Y * v.X + el1.Y * v.Y + el2.Y * v.Z;
        }

        public float tdotz(ref btVector3 v)
        {
            return el0.Z * v.X + el1.Z * v.Y + el2.Z * v.Z;
        }

        public void transpose(out btMatrix3x3 result)
        {
            result.el0.X = el0.X;
            result.el0.Y = el1.X;
            result.el0.Z = el2.X;
            result.el0.W = 0;
            result.el1.X = el0.Y;
            result.el1.Y = el1.Y;
            result.el1.Z = el2.Y;
            result.el1.W = 0;
            result.el2.X = el0.Z;
            result.el2.Y = el1.Z;
            result.el2.Z = el2.Z;
            result.el2.W = 0;
        }
    }

}
