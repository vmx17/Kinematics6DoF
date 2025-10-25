using System;

namespace Math4
{
    // 3D vector
    public struct Vec3
    {
        public double X, Y, Z;
        public Vec3(double x, double y, double z)
        {
            X = x; Y = y; Z = z;
        }

        public static Vec3 operator +(Vec3 a, Vec3 b) => new Vec3(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        public static Vec3 operator -(Vec3 a, Vec3 b) => new Vec3(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        public static Vec3 operator *(double s, Vec3 a) => new Vec3(s * a.X, s * a.Y, s * a.Z);

        public double Norm() => Math.Sqrt(X * X + Y * Y + Z * Z);
        public Vec3 Normalize()
        {
            var n = Norm(); return n > 0 ? new Vec3(X / n, Y / n, Z / n) : new Vec3(0, 0, 0);
        }

        public static double Dot(Vec3 a, Vec3 b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        public static Vec3 Cross(Vec3 a, Vec3 b) =>
            new Vec3(a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X);
    }

    // 4x4 homogeneous matrix (row-major, post-multiplication convention)
    public class Mat4
    {
        public double[,] M = new double[4, 4];
        public Mat4()
        {
            for (int i = 0; i < 4; i++) M[i, i] = 1.0;
        }
        public static Mat4 Identity() => new Mat4();

        public static Mat4 Multiply(Mat4 A, Mat4 B)
        {
            var C = new Mat4();
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                {
                    double s = 0;
                    for (int k = 0; k < 4; k++) s += A.M[i, k] * B.M[k, j];
                    C.M[i, j] = s;
                }
            return C;
        }

        public Vec3 TransformPoint(Vec3 p)
        {
            double x = M[0, 0] * p.X + M[0, 1] * p.Y + M[0, 2] * p.Z + M[0, 3];
            double y = M[1, 0] * p.X + M[1, 1] * p.Y + M[1, 2] * p.Z + M[1, 3];
            double z = M[2, 0] * p.X + M[2, 1] * p.Y + M[2, 2] * p.Z + M[2, 3];
            return new Vec3(x, y, z);
        }

        public Vec3 TransformVector(Vec3 v)
        {
            double x = M[0, 0] * v.X + M[0, 1] * v.Y + M[0, 2] * v.Z;
            double y = M[1, 0] * v.X + M[1, 1] * v.Y + M[1, 2] * v.Z;
            double z = M[2, 0] * v.X + M[2, 1] * v.Y + M[2, 2] * v.Z;
            return new Vec3(x, y, z);
        }

        // MDH primitive transforms
        public static Mat4 RotX(double alpha)
        {
            var c = Math.Cos(alpha); var s = Math.Sin(alpha);
            var R = Identity();
            R.M[1, 1] = c; R.M[1, 2] = -s;
            R.M[2, 1] = s; R.M[2, 2] = c;
            return R;
        }

        public static Mat4 RotZ(double theta)
        {
            var c = Math.Cos(theta); var s = Math.Sin(theta);
            var R = Identity();
            R.M[0, 0] = c; R.M[0, 1] = -s;
            R.M[1, 0] = s; R.M[1, 1] = c;
            return R;
        }

        public static Mat4 TransX(double a)
        {
            var T = Identity(); T.M[0, 3] = a; return T;
        }

        public static Mat4 TransZ(double d)
        {
            var T = Identity(); T.M[2, 3] = d; return T;
        }

        public double[,] Rotation3()
        {
            var R = new double[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    R[i, j] = M[i, j];
            return R;
        }

        public static Mat4 InverseRigid(Mat4 T)
        {
            var Rt = new double[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    Rt[i, j] = T.M[j, i]; // transpose
            var t = new Vec3(T.M[0, 3], T.M[1, 3], T.M[2, 3]);
            var tInv = new Vec3(
                    -(Rt[0, 0] * t.X + Rt[0, 1] * t.Y + Rt[0, 2] * t.Z),
                    -(Rt[1, 0] * t.X + Rt[1, 1] * t.Y + Rt[1, 2] * t.Z),
                    -(Rt[2, 0] * t.X + Rt[2, 1] * t.Y + Rt[2, 2] * t.Z));
            return FromRotationTranslation(Rt, tInv);
        }

        public static Mat4 FromRotationTranslation(double[,] R3, Vec3 t)
        {
            var T = Identity();
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    T.M[i, j] = R3[i, j];
            T.M[0, 3] = t.X; T.M[1, 3] = t.Y; T.M[2, 3] = t.Z;
            return T;
        }


        // Yaskawa rz→ry→rx rotation
        public static double[,] RzRyRx(double rz, double ry, double rx)
        {
            double cz = Math.Cos(rz), sz = Math.Sin(rz);
            double cy = Math.Cos(ry), sy = Math.Sin(ry);
            double cx = Math.Cos(rx), sx = Math.Sin(rx);

            double[,] Rz = { { cz, -sz, 0 }, { sz, cz, 0 }, { 0, 0, 1 } };
            double[,] Ry = { { cy, 0, sy }, { 0, 1, 0 }, { -sy, 0, cy } };
            double[,] Rx = { { 1, 0, 0 }, { 0, cx, -sx }, { 0, sx, cx } };

            return Multiply3x3(Multiply3x3(Rz, Ry), Rx);
        }

        public static double[,] Multiply3x3(double[,] A, double[,] B)
        {
            var C = new double[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                {
                    double s = 0;
                    for (int k = 0; k < 3; k++) s += A[i, k] * B[k, j];
                    C[i, j] = s;
                }
            return C;
        }

        public static double Clamp(double x, double min, double max) => Math.Max(min, Math.Min(max, x));
    }
}