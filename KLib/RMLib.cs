using System;

namespace RowMeasureUtility;

/// <summary>
/// Row-Major oriented math helpers (RMLib).
/// Intent: internal support for Kinematics3 where intermediate 3x3 rotation
/// storage uses "row-major = each row is an axis (X,Y,Z)".
/// NOTE:
/// - 4x4 homogeneous matrices returned here remain standard column-major
///   (columns = basis vectors), to stay compatible with existing LibD API,
///   unless explicitly stated.
/// - Row-major 3x3 in this file: Rr[row, col] where:
///     row 0 = X-axis vector components (x,y,z)
///     row 1 = Y-axis vector
///     row 2 = Z-axis vector
///   This is exactly the internal layout now used in Kinematics3.
/// </summary>
public static class RMLib
{
    #region Basic structs (lightweight copies to avoid coupling)
    public readonly struct Vec3
    {
        public readonly double X, Y, Z;
        public Vec3(double x, double y, double z)
        {
            X = x; Y = y; Z = z;
        }

        public static Vec3 operator +(Vec3 a, Vec3 b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        public static Vec3 operator -(Vec3 a, Vec3 b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        public static Vec3 operator *(double s, Vec3 v) => new(s * v.X, s * v.Y, s * v.Z);
        public double Dot(Vec3 b) => X * b.X + Y * b.Y + Z * b.Z;
        public double Norm() => Math.Sqrt(X * X + Y * Y + Z * Z);
        public Vec3 Normalized()
        {
            var n = Norm();
            return n < 1e-15 ? new Vec3(0, 0, 0) : new Vec3(X / n, Y / n, Z / n);
        }
        public override string ToString() => $"({X:F6},{Y:F6},{Z:F6})";
    }
    #endregion

    #region Row-Major Martix utilities

    /// <summary>
    /// Allocate a new row-major 3x3 from three axis vectors (assumed already orthonormal).
    /// </summary>
    public static double[,] MakeRowMajor3x3(Vec3 xAxis, Vec3 yAxis, Vec3 zAxis)
    {
        return new double[3, 3]
        {
            { xAxis.X, xAxis.Y, xAxis.Z },
            { yAxis.X, yAxis.Y, yAxis.Z },
            { zAxis.X, zAxis.Y, zAxis.Z }
        };
    }

    /// <summary>
    /// Convert a row-major 3x3 (rows=axes) to a column-major one (columns=axes).
    /// </summary>
    public static double[,] RowToColumn(double[,] Rr)
    {
        // Rr[rowAxis, component] -> Rc[component, axis]
        return new double[3, 3]
        {
            { Rr[0,0], Rr[1,0], Rr[2,0] }, // column 0 = X
            { Rr[0,1], Rr[1,1], Rr[2,1] }, // column 1 = Y
            { Rr[0,2], Rr[1,2], Rr[2,2] }  // column 2 = Z
        };
    }

    /// <summary>
    /// Convert a column-major 3x3 (columns=axes) to row-major (rows=axes).
    /// </summary>
    public static double[,] ColumnToRow(double[,] Rc)
    {
        return new double[3, 3]
        {
            { Rc[0,0], Rc[1,0], Rc[2,0] }, // row 0 = X
            { Rc[0,1], Rc[1,1], Rc[2,1] }, // row 1 = Y
            { Rc[0,2], Rc[1,2], Rc[2,2] }  // row 2 = Z
        };
    }

    /// <summary>
    /// Row-major transpose (returns a new row-major storing the transpose).
    /// If Rr rows=axes, its transpose has columns=axes. To keep row-major
    /// layout, we reinterpret (so effectively swap roles).
    /// </summary>
    public static double[,] TransposeRM3x3(double[,] Rr)
    {
        var Rt = new double[3, 3];
        for (var i = 0; i < 3; i++)
            for (var j = 0; j < 3; j++)
                Rt[i, j] = Rr[j, i];
        return Rt;
    }

    internal static double[,] TransposeRM(double[,] M)
    {
        int rows = M.GetLength(0), cols = M.GetLength(1);
        var Tt = new double[cols, rows];
        for (var r = 0; r < rows; r++)
            for (var c = 0; c < cols; c++)
                Tt[c, r] = M[r, c];
        return Tt;
    }

    /// <summary>
    /// Multiply3x3 two row-major rotation matrices (conceptually): R = A * B
    /// Inputs A,B: rows are axes of each rotation in parent frame.
    /// Output is also row-major. We internally convert to column-major,
    /// multiply, then convert back to row-major for clarity & safety.
    /// </summary>
    public static double[,] MatMul3x3(double[,] A_r, double[,] B_r)
    {
        var A_c = RowToColumn(A_r);
        var B_c = RowToColumn(B_r);
        // Standard column-major multiplication C_c = A_c * B_c
        var C_c = new double[3, 3];
        for (var r = 0; r < 3; r++)
            for (var c = 0; c < 3; c++)
                for (var k = 0; k < 3; k++)
                    C_c[r, c] += A_c[r, k] * B_c[k, c];
        return ColumnToRow(C_c);
    }

    internal static double[,] MatMul4x4(double[,] A, double[,] B)
    {
        var C = new double[4, 4];
        for (var i = 0; i < 4; i++)
            for (var j = 0; j < 4; j++)
                for (var k = 0; k < 4; k++)
                    C[i, j] += A[i, k] * B[k, j];
        return C;
    }

    internal static double[] MatVecMul(double[,] M, double[] v)
    {
        int rows = M.GetLength(0), cols = M.GetLength(1);
        var r = new double[rows];
        for (var i = 0; i < rows; i++)
            for (var j = 0; j < cols; j++)
                r[i] += M[i, j] * v[j];
        return r;
    }

    /// <summary>
    /// Rotate a vector by a row-major rotation (rows=axes). Equivalent to
    /// v' = R_c * v where R_c is the column-major form. Implementation:
    /// convert once, multiply, return Vec3.
    /// </summary>
    public static Vec3 RowMajorRotate(double[,] Rr, Vec3 v)
    {
        var Rc = RowToColumn(Rr);
        return new Vec3(
            Rc[0, 0] * v.X + Rc[0, 1] * v.Y + Rc[0, 2] * v.Z,
            Rc[1, 0] * v.X + Rc[1, 1] * v.Y + Rc[1, 2] * v.Z,
            Rc[2, 0] * v.X + Rc[2, 1] * v.Y + Rc[2, 2] * v.Z
        );
    }

    /// <summary>
    /// Build a row-major rotation from a Z axis and a primary reference axis (in local parent frame).
    /// Semantic twin of the implementation inside Kinematics3; exposed here for reuse/tests.
    /// </summary>
    public static double[,] BuildRowMajorFromZ(Vec3 zDir, Vec3 primaryRef,
                                               Vec3? altRef = null)
    {
        var z = zDir.Normalized();
        if (z.Norm() < 1e-12)
            return Identity3x3();

        // Remove component along Z to get candidate X
        var dot = primaryRef.Dot(z);
        var Xc = new Vec3(primaryRef.X - dot * z.X,
                          primaryRef.Y - dot * z.Y,
                          primaryRef.Z - dot * z.Z);

        if (Xc.Norm() < 1e-9)
        {
            var alt = altRef ?? new Vec3(1, 0, 0);
            var d2 = alt.Dot(z);
            Xc = new Vec3(alt.X - d2 * z.X,
                          alt.Y - d2 * z.Y,
                          alt.Z - d2 * z.Z);
            if (Xc.Norm() < 1e-9)
            {
                // Last resort static fallback
                var fallback = Math.Abs(z.Z) < 0.9 ? new Vec3(0, 0, 1) : new Vec3(0, 1, 0);
                var d3 = fallback.Dot(z);
                Xc = new Vec3(fallback.X - d3 * z.X,
                              fallback.Y - d3 * z.Y,
                              fallback.Z - d3 * z.Z);
                if (Xc.Norm() < 1e-12)
                    return Identity3x3();
            }
        }

        var x = Xc.Normalized();
        var y = CrossVec3(z, x).Normalized();

        return MakeRowMajor3x3(x, y, z);
    }

    #endregion

    #region Homogeneous (row-major ↔ column-major interop)

    /// <summary>
    /// Construct a standard (column-major) 4x4 homogeneous transform from:
    /// row-major rotation (rows=axes) and translation vector (in same parent frame).
    /// </summary>
    public static double[,] ComposeHomogeneousFromRow(double[,] Rr, Vec3 t)
    {
        var Rc = RowToColumn(Rr);
        var T = new double[4, 4];
        for (var r = 0; r < 3; r++)
        {
            T[r, 0] = Rc[r, 0];
            T[r, 1] = Rc[r, 1];
            T[r, 2] = Rc[r, 2];
        }
        T[0, 3] = t.X; T[1, 3] = t.Y; T[2, 3] = t.Z;
        T[3, 0] = 0; T[3, 1] = 0; T[3, 2] = 0; T[3, 3] = 1;
        return T;
    }

    /// <summary>
    /// Invert a homogeneous transform whose rotation part is in row-major (rows=axes).
    /// Returned matrix is column-major standard.
    /// </summary>
    public static double[,] InvertHomogeneousFromRow(double[,] Rr, Vec3 t)
    {
        // R_inv = R^T; p' = -R^T * t
        var Rc = RowToColumn(Rr);
        var Rt = new double[3, 3];
        for (var i = 0; i < 3; i++)
            for (var j = 0; j < 3; j++)
                Rt[i, j] = Rc[j, i];

        var pInv = new double[3];
        for (var i = 0; i < 3; i++)
            pInv[i] = -(Rt[i, 0] * t.X + Rt[i, 1] * t.Y + Rt[i, 2] * t.Z);

        var T = new double[4, 4];
        for (var r = 0; r < 3; r++)
        {
            T[r, 0] = Rt[r, 0];
            T[r, 1] = Rt[r, 1];
            T[r, 2] = Rt[r, 2];
        }
        T[0, 3] = pInv[0]; T[1, 3] = pInv[1]; T[2, 3] = pInv[2];
        T[3, 0] = 0; T[3, 1] = 0; T[3, 2] = 0; T[3, 3] = 1;
        return T;
    }

    internal static double[,] InvertHomogeneous4x4(double[,] T)
    {
        var R = new double[3, 3];
        for (var r = 0; r < 3; r++)
            for (var c = 0; c < 3; c++)
                R[r, c] = T[r, c];
        var Rt = new double[3, 3];
        for (var r = 0; r < 3; r++)
            for (var c = 0; c < 3; c++)
                Rt[r, c] = R[c, r];
        var p = new[] { T[0, 3], T[1, 3], T[2, 3] };
        var pInv = new double[3];
        for (var r = 0; r < 3; r++)
            pInv[r] = -(Rt[r, 0] * p[0] + Rt[r, 1] * p[1] + Rt[r, 2] * p[2]);

        var Inv = new double[4, 4];
        for (var r = 0; r < 3; r++)
        {
            Inv[r, 0] = Rt[r, 0]; Inv[r, 1] = Rt[r, 1]; Inv[r, 2] = Rt[r, 2]; Inv[r, 3] = pInv[r];
        }
        Inv[3, 0] = 0; Inv[3, 1] = 0; Inv[3, 2] = 0; Inv[3, 3] = 1;
        return Inv;
    }
    #endregion

    #region Math helpers
    internal static double[] Cross3x3(double[] a, double[] b) =>
    [
            a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0]
    ];
    public static Vec3 CrossVec3(Vec3 a, Vec3 b) =>
        new(a.Y * b.Z - a.Z * b.Y,
            a.Z * b.X - a.X * b.Z,
            a.X * b.Y - a.Y * b.X);

    internal static double[,] DHTransform(double theta, double d, double a, double alpha)
    {
        double ct = Math.Cos(theta), st = Math.Sin(theta);
        double ca = Math.Cos(alpha), sa = Math.Sin(alpha);
        return new double[4, 4]
        {
            { ct, -st*ca,  st*sa, a*ct },
            { st,  ct*ca, -ct*sa, a*st },
            { 0,      sa,     ca,    d },
            { 0,       0,      0,    1 }
        };
    }

    // 追加: Modified DH (Craig) 変換
    // 引数: theta_i, d_i, a_{i-1}, alpha_{i-1}
    internal static double[,] MDHTransform(double theta, double d, double aPrev, double alphaPrev)
    {
        double ct = Math.Cos(theta), st = Math.Sin(theta);
        double ca = Math.Cos(alphaPrev), sa = Math.Sin(alphaPrev);
        // MDH:
        // [  ct     -st      0     a_{i-1} ]
        // [  st ca  ct ca   -sa   -sa d_i  ]
        // [  st sa  ct sa    ca    ca d_i  ]
        // [   0       0       0      1     ]
        return new double[4,4]
        {
            { ct,      -st,       0,    aPrev },
            { st*ca,  ct*ca,    -sa,  -sa*d   },
            { st*sa,  ct*sa,     ca,   ca*d   },
            { 0,        0,       0,     1     }
        };
    }

    internal static double[,] Identity3x3() =>
        new double[3, 3] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

    // Added: 4x4 identity matrix (column-major standard homogeneous)
    internal static double[,] Identity4x4() =>
        new double[4, 4]
        {
            {1,0,0,0},
            {0,1,0,0},
            {0,0,1,0},
            {0,0,0,1}
        };

    internal static double NormalizeAngleSigned(double angle)
    {
        const double TwoPi = 2.0 * Math.PI;
        angle %= TwoPi;
        if (angle <= -Math.PI) angle += TwoPi;
        else if (angle > Math.PI) angle -= TwoPi;
        return angle;
    }

    internal static double NormalizeAnglePositive(double angle)
    {
        const double TwoPi = 2.0 * Math.PI;
        angle %= TwoPi;
        if (angle < 0) angle += TwoPi;
        return angle;
    }
    #endregion
}