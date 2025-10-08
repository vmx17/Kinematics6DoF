using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ColumnMeasureUtility;
public class CMLib
{
    public struct Vec3(double x, double y, double z)
    {
        public double X = x, Y = y, Z = z;

        public static Vec3 operator +(Vec3 a, Vec3 b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        public static Vec3 operator -(Vec3 a, Vec3 b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        public static Vec3 operator *(double s, Vec3 v) => new(s * v.X, s * v.Y, s * v.Z);
        public readonly override string ToString() => $"({X}, {Y}, {Z})";
        public readonly string ToString3() => $"({X:F3}, {Y:F3}, {Z:F3})";
        public readonly string ToString4() => $"({X:F4}, {Y:F4}, {Z:F4})";
    }

    public struct Mat3(Vec3 x, Vec3 y, Vec3 z)
    {
        public Vec3 Xaxis = x, Yaxis = y, Zaxis = z; // Columns
    }


    /// <summary>
    /// 4x4同次変換行列の逆行列を計算します。
    /// </summary>
    internal static double[,] MatrixInverse4x4(double[,] T_matrix)
    {
        var inv = new double[4, 4];

        // R' (転置行列)
        var R_T = new double[3, 3];
        for (var i = 0; i < 3; i++)
        {
            for (var j = 0; j < 3; j++)
            {
                R_T[i, j] = T_matrix[j, i];
            }
        }

        // p
        double[] p = { T_matrix[0, 3], T_matrix[1, 3], T_matrix[2, 3] };

        // -R' * p
        var neg_RT_p = new double[3];
        for (var i = 0; i < 3; i++)
        {
            neg_RT_p[i] = 0;
            for (var j = 0; j < 3; j++)
            {
                neg_RT_p[i] -= R_T[i, j] * p[j];
            }
        }

        // 逆行列を組み立てる
        for (var i = 0; i < 3; i++)
        {
            for (var j = 0; j < 3; j++)
            {
                inv[i, j] = R_T[i, j];
            }
            inv[i, 3] = neg_RT_p[i];
        }
        inv[3, 0] = inv[3, 1] = inv[3, 2] = 0;
        inv[3, 3] = 1;

        return inv;
    }

    /// <summary>
    /// Denavit-Hartenbergパラメータから同次変換行列を生成します。
    /// </summary>
    internal static double[,] T(double theta, double d, double a, double alpha)
    {
        var cosTheta = Math.Cos(theta);
        var sinTheta = Math.Sin(theta);
        var cosAlpha = Math.Cos(alpha);
        var sinAlpha = Math.Sin(alpha);

        return new double[,]
        {
            { cosTheta, -sinTheta * cosAlpha,  sinTheta * sinAlpha, a * cosTheta },
            { sinTheta,  cosTheta * cosAlpha, -cosTheta * sinAlpha, a * sinTheta },
            { 0,         sinAlpha,            cosAlpha,             d            },
            { 0,         0,                   0,                    1            }
        };
    }

    /// <summary>
    /// 4x4行列の乗算を行います。
    /// </summary>
    internal static double[,] MatMul4x4(double[,] A, double[,] B)
    {
        var C = new double[4, 4];
        for (var i = 0; i < 4; i++)
        {
            for (var j = 0; j < 4; j++)
            {
                for (var k = 0; k < 4; k++)
                {
                    C[i, j] += A[i, k] * B[k, j];
                }
            }
        }
        return C;
    }

    /// <summary>
    /// 3x3行列の乗算を行います。
    /// </summary>
    internal static double[,] MatMul3x3(double[,] A, double[,] B)
    {
        var C = new double[3, 3];
        for (var i = 0; i < 3; i++)
        {
            for (var j = 0; j < 3; j++)
            {
                for (var k = 0; k < 3; k++)
                {
                    C[i, j] += A[i, k] * B[k, j];
                }
            }
        }
        return C;
    }

    /// <summary>
    /// 4x4行列の転置を行います。
    /// </summary>
    internal static double[,] Transpose(double[,] matrix)
    {
        var rows = matrix.GetLength(0);
        var cols = matrix.GetLength(1);
        var result = new double[cols, rows];
        for (var i = 0; i < rows; i++)
        {
            for (var j = 0; j < cols; j++)
            {
                result[j, i] = matrix[i, j];
            }
        }
        return result;
    }

    /// <summary>
    /// 3要素ベクトルの外積を計算します。
    /// </summary>
    internal static double[] CrossProduct(double[] a, double[] b)
    {
        return new double[]
        {
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]
        };
    }

    /// <summary>
    /// 行列とベクトルの乗算を行います。
    /// </summary>
    internal static double[] MatVecMul(double[,] M, double[] v)
    {
        var rows = M.GetLength(0);
        var cols = M.GetLength(1);
        var result = new double[rows];
        for (var i = 0; i < rows; i++)
        {
            for (var j = 0; j < cols; j++)
            {
                result[i] += M[i, j] * v[j];
            }
        }
        return result;
    }

    /// <summary>
    /// 単位行列を生成します。
    /// </summary>
    internal static double[,] Identity4x4()
    {
        return new double[,]
        {
            { 1, 0, 0, 0 },
            { 0, 1, 0, 0 },
            { 0, 0, 1, 0 },
            { 0, 0, 0, 1 }
        };
    }

    internal static (double Rz, double Ry, double Rx) DecomposeZYX(Mat3 R)
    {
        var r11 = R.Xaxis.X;
        var r21 = R.Xaxis.Y;
        var r31 = R.Xaxis.Z;
        var r32 = R.Yaxis.Z;
        var r33 = R.Zaxis.Z;

        var Ry = Math.Asin(-r31);
        var c = Math.Cos(Ry);
        double Rz, Rx;
        if (Math.Abs(c) > 1e-9)
        {
            Rz = Math.Atan2(r21, r11);
            Rx = Math.Atan2(r32, r33);
        }
        else
        {
            Rz = 0;
            var r12 = R.Yaxis.X;
            var r13 = R.Zaxis.X;
            Rx = Math.Atan2(-r12, -r13);
        }
        return (Rz, Ry, Rx);
    }

    internal static Mat3 Transpose(Mat3 M) =>
        new(
            new Vec3(M.Xaxis.X, M.Yaxis.X, M.Zaxis.X),
            new Vec3(M.Xaxis.Y, M.Yaxis.Y, M.Zaxis.Y),
            new Vec3(M.Xaxis.Z, M.Yaxis.Z, M.Zaxis.Z)
        );

    internal static Mat3 Multiply3x3(Mat3 A, Mat3 B)
    {
        Vec3 Mul(Vec3 v) => new(
            A.Xaxis.X * v.X + A.Yaxis.X * v.Y + A.Zaxis.X * v.Z,
            A.Xaxis.Y * v.X + A.Yaxis.Y * v.Y + A.Zaxis.Y * v.Z,
            A.Xaxis.Z * v.X + A.Yaxis.Z * v.Y + A.Zaxis.Z * v.Z
        );
        return new Mat3(Mul(B.Xaxis), Mul(B.Yaxis), Mul(B.Zaxis));
    }

    internal static Vec3 Cross(Vec3 a, Vec3 b) =>
        new(a.Y * b.Z - a.Z * b.Y,
            a.Z * b.X - a.X * b.Z,
            a.X * b.Y - a.Y * b.X);

    internal static double Norm(Vec3 v) =>
        Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);


    internal static double NormalizeAngle(double angle)
    {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
