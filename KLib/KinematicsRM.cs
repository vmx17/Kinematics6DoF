using System;
using System.Collections.Generic;
using System.Linq;

namespace KinematicsRM;

// MDH parameter container
public class MDHParameters
{
    public double[] Alpha { get; }
    public double[] A { get; }
    public double[] D { get; }
    public double[] Offset { get; }
    public double[] MinAnglesDeg { get; }
    public double[] MaxAnglesDeg { get; }

    public MDHParameters(double[] alpha, double[] a, double[] d, double[] offset, double[]? minAngles = null, double[]? maxAngles = null)
    {
        if (alpha.Length != 6 || a.Length != 6 || d.Length != 6 || offset.Length != 6)
            throw new ArgumentException("All MDH parameter arrays must have 6 elements.");
        Alpha  = alpha;
        A      = a;
        D      = d;
        Offset = offset;
        MinAnglesDeg = minAngles ?? new double[] { -360, -360, -360, -360, -360, -360 };
        MaxAnglesDeg = maxAngles ?? new double[] {  360,  360,  360,  360,  360,  360 };
    }
}

// Factory
public static class ManipulatorFactory
{
    private static readonly Dictionary<string, MDHParameters> _mdhTable = new(StringComparer.OrdinalIgnoreCase)
    {
        ["MiRobot"] = new MDHParameters(
            [0, Math.PI/2, Math.PI, -Math.PI/2, Math.PI/2, Math.PI/2],
            [0, 29.69, 108, 20, 0, 0],
            [127, 0, 0, 168.98, 0, 24.29],
            [0, Math.PI/2, 0, 0, -Math.PI/2, 0],
            [-179, -125, -155, -180, -120, -180],
            [ 179,  125,  155,  180,  120,  180]
        )
    };

    private static readonly Dictionary<string, double[]> _toolTable = new(StringComparer.OrdinalIgnoreCase)
    {
        ["null"]  = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ["tool0"] = [1.41421356, 0.0, 2.41421356, 0.70710678, 0.0, 0.70710678]
    };

    public static Manipulator6DoF CreateManipulator(string? robotName = null, string? toolName = null)
    {
        var r = string.IsNullOrWhiteSpace(robotName) ? "MiRobot" : robotName;
        var t = string.IsNullOrWhiteSpace(toolName)  ? "null"   : toolName;
        if (!_mdhTable.TryGetValue(r, out var mdh))   throw new ArgumentException($"Unknown robot name: {r}");
        if (!_toolTable.TryGetValue(t, out var tool)) throw new ArgumentException($"Unknown tool name: {t}");
        return new Manipulator6DoF(mdh, tool);
    }

    public static IEnumerable<string> GetAvailableRobotNames() => _mdhTable.Keys;
    public static IEnumerable<string> GetAvailableToolNames()  => _toolTable.Keys;
}

public class Manipulator6DoF
{
    private readonly MDHParameters _mdh;
    private readonly double[] _tool;
    private readonly double[] _minAnglesRad;
    private readonly double[] _maxAnglesRad;

    private static readonly double Deg2Rad = Math.PI / 180.0;

    public enum ToolReferenceAxis
    {
        FlangeY = 0,
        FlangeX = 1
    }

    private ToolReferenceAxis _toolRefAxis = ToolReferenceAxis.FlangeY;

    public Manipulator6DoF(MDHParameters mdh, double[] tool)
    {
        if (tool is null || tool.Length != 6)
            throw new ArgumentException("Tool must have 6 elements.");
        _mdh = mdh;
        _tool = (double[])tool.Clone();
        _minAnglesRad = _mdh.MinAnglesDeg.Select(a => a * Deg2Rad).ToArray();
        _maxAnglesRad = _mdh.MaxAnglesDeg.Select(a => a * Deg2Rad).ToArray();
    }

    public void SetToolReferenceAxis(ToolReferenceAxis axis) => _toolRefAxis = axis;

    // Forward (row-major internal axes; uses RMLib row-major helpers)
    public double[,] Forward(double[] q, int verbose = 0)
    {
        if (q.Length != 6) throw new ArgumentException("q must have 6 elements");

        var alpha = _mdh.Alpha;
        var a = _mdh.A;
        var d = _mdh.D;
        var offset = _mdh.Offset;

        var origins = new double[7][];
        var axes = new double[7][,]; // row-major: row=axis (X,Y,Z)

        origins[0] = new[] { 0.0, 0.0, 0.0 };
        axes[0] = new double[,] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

        for (var i = 0; i < 6; i++)
        {
            var theta = q[i] + offset[i];
            double ct = Math.Cos(theta), st = Math.Sin(theta);
            double ca = Math.Cos(alpha[i]), sa = Math.Sin(alpha[i]);

            double[] Xprev = { axes[i][0, 0], axes[i][0, 1], axes[i][0, 2] };
            double[] Yprev = { axes[i][1, 0], axes[i][1, 1], axes[i][1, 2] };
            double[] Zprev = { axes[i][2, 0], axes[i][2, 1], axes[i][2, 2] };

            // Same modified DH style as earlier implementation (row-major adaptation)
            var Xi = new double[3];
            Xi[0] = ct * Xprev[0] + st * (ca * Yprev[0] + sa * Zprev[0]);
            Xi[1] = ct * Xprev[1] + st * (ca * Yprev[1] + sa * Zprev[1]);
            Xi[2] = ct * Xprev[2] + st * (ca * Yprev[2] + sa * Zprev[2]);

            var Yi = new double[3];
            Yi[0] = -st * Xprev[0] + ct * (ca * Yprev[0] + sa * Zprev[0]);
            Yi[1] = -st * Xprev[1] + ct * (ca * Yprev[1] + sa * Zprev[1]);
            Yi[2] = -st * Xprev[2] + ct * (ca * Yprev[2] + sa * Zprev[2]);

            var Zi = new double[3];
            Zi[0] = -sa * Yprev[0] + ca * Zprev[0];
            Zi[1] = -sa * Yprev[1] + ca * Zprev[1];
            Zi[2] = -sa * Yprev[2] + ca * Zprev[2];

            axes[i + 1] = new double[,]
            {
                { Xi[0], Xi[1], Xi[2] },
                { Yi[0], Yi[1], Yi[2] },
                { Zi[0], Zi[1], Zi[2] }
            };

            origins[i + 1] = new double[3];
            origins[i + 1][0] = origins[i][0] + a[i] * Xprev[0] + d[i] * Zi[0];
            origins[i + 1][1] = origins[i][1] + a[i] * Xprev[1] + d[i] * Zi[1];
            origins[i + 1][2] = origins[i][2] + a[i] * Xprev[2] + d[i] * Zi[2];
        }

        // Build flange homogeneous (columns = axes)
        var T06 = new double[4, 4];
        for (var row = 0; row < 3; row++)
        {
            T06[row, 0] = axes[6][0, row]; // X
            T06[row, 1] = axes[6][1, row]; // Y
            T06[row, 2] = axes[6][2, row]; // Z
            T06[row, 3] = origins[6][row];
        }
        T06[3, 0] = 0; T06[3, 1] = 0; T06[3, 2] = 0; T06[3, 3] = 1;

        double xt = _tool[0], yt = _tool[1], zt = _tool[2];
        double zx = _tool[3], zy = _tool[4], zz = _tool[5];

        var R_tool_row = BuildToolRotationRowMajor(zx, zy, zz, _toolRefAxis);
        var T6_tool = new double[4, 4];
        for (var row = 0; row < 3; row++)
        {
            T6_tool[row, 0] = R_tool_row[0, row];
            T6_tool[row, 1] = R_tool_row[1, row];
            T6_tool[row, 2] = R_tool_row[2, row];
        }
        T6_tool[0, 3] = xt; T6_tool[1, 3] = yt; T6_tool[2, 3] = zt;
        T6_tool[3, 0] = 0; T6_tool[3, 1] = 0; T6_tool[3, 2] = 0; T6_tool[3, 3] = 1;

        var T0_tool = MatMul4(T06, T6_tool);

        if (verbose >= 1)
        {
            Console.WriteLine("---- Origins ----");
            for (var i = 0; i <= 6; i++)
                Console.WriteLine($"O{i}: ({origins[i][0]:F3}, {origins[i][1]:F3}, {origins[i][2]:F3})");
            Console.WriteLine($"Tool: ({T0_tool[0, 3]:F3}, {T0_tool[1, 3]:F3}, {T0_tool[2, 3]:F3})");
        }
        if (verbose >= 2)
        {
            Console.WriteLine("---- Axes (row-major internal) ----");
            for (var i = 1; i <= 6; i++)
            {
                Console.WriteLine($"Frame {i} X=({axes[i][0, 0]:F3},{axes[i][0, 1]:F3},{axes[i][0, 2]:F3})");
                Console.WriteLine($"         Y=({axes[i][1, 0]:F3},{axes[i][1, 1]:F3},{axes[i][1, 2]:F3})");
                Console.WriteLine($"         Z=({axes[i][2, 0]:F3},{axes[i][2, 1]:F3},{axes[i][2, 2]:F3})");
            }
        }

        return T0_tool;
    }

    public double[][] InverseGeometric(double[,] T_target, out bool[] flags, int verbose = 0)
    {
        var solutions = new double[8][]; for (int i = 0; i < 8; i++) solutions[i] = new double[6];
        flags = new bool[8];
        var printed = false;

        double xt = _tool[0], yt = _tool[1], zt = _tool[2];
        double zx = _tool[3], zy = _tool[4], zz = _tool[5];
        var R_tool_row = BuildToolRotationRowMajor(zx, zy, zz, ToolReferenceAxis.FlangeY);
        var T_tool = new double[4, 4];
        for (var r = 0; r < 3; r++)
        {
            T_tool[r, 0] = R_tool_row[0, r];
            T_tool[r, 1] = R_tool_row[1, r];
            T_tool[r, 2] = R_tool_row[2, r];
        }
        T_tool[0, 3] = xt; T_tool[1, 3] = yt; T_tool[2, 3] = zt; T_tool[3, 3] = 1;

        var T_tool_inv = InvertHomogeneous(T_tool);
        var T_target_flange = MatMul4(T_target, T_tool_inv);

        var a = _mdh.A; var d = _mdh.D; var off = _mdh.Offset;
        double d1 = d[0], a2 = a[1], a3 = a[2], d4 = d[3], d6 = d[5];

        double ax = T_target_flange[0, 2], ay = T_target_flange[1, 2], az = T_target_flange[2, 2];
        double Px = T_target_flange[0, 3], Py = T_target_flange[1, 3], Pz = T_target_flange[2, 3];

        var Pwx = Px - d6 * ax;
        var Pwy = Py - d6 * ay;
        var Pwz = Pz - d6 * az;

        var th1a = Math.Atan2(Pwy, Pwx);
        var th1b = th1a + Math.PI;

        var r_sq = Pwx * Pwx + Pwy * Pwy;
        var s = Pwz - d1;
        var D_sq = r_sq + s * s;

        var L3 = Math.Sqrt(a3 * a3 + d4 * d4);
        var cosBase = (D_sq - a2 * a2 - L3 * L3) / (2 * a2 * L3);
        if (Math.Abs(cosBase) > 1) return solutions;
        var baseAng = Math.Acos(cosBase);
        var phi = Math.Atan2(d4, a3);
        var th3a = baseAng - phi;
        var th3b = -baseAng - phi;

        var pairs = new (double t1, double t3)[] { (th1a, th3a), (th1a, th3b), (th1b, th3a), (th1b, th3b) };
        var idx = 0;
        foreach (var (t1, t3) in pairs)
        {
            double s1 = Math.Sin(t1), c1 = Math.Cos(t1);
            double s3p = Math.Sin(t3 + phi), c3p = Math.Cos(t3 + phi);
            var k1 = a2 + L3 * c3p;
            var k2 = L3 * s3p;
            var proj = c1 * Pwx + s1 * Pwy;
            var c2 = (k1 * proj + k2 * s) / (k1 * k1 + k2 * k2);
            var s2 = (k2 * proj - k1 * s) / (k1 * k1 + k2 * k2);
            var t2 = Math.Atan2(s2, c2);

            var T01 = DHTransform(t1 + off[0], d[0], a[0], _mdh.Alpha[0]);
            var T12 = DHTransform(t2 + off[1], d[1], a[1], _mdh.Alpha[1]);
            var T23 = DHTransform(t3 + off[2], d[2], a[2], _mdh.Alpha[2]);
            var T03 = MatMul4(MatMul4(T01, T12), T23);
            var T03_inv = InvertHomogeneous(T03);
            var T36 = MatMul4(T03_inv, T_target_flange);

            var cos_t5 = T36[1, 2];
            if (Math.Abs(cos_t5) > 1) { idx += 2; continue; }
            var t5p = Math.Acos(cos_t5);
            var t5m = -t5p;

            foreach (var t5 in new[] { t5p, t5m })
            {
                var s5 = Math.Sin(t5);
                double t4, t6;
                if (Math.Abs(s5) > 1e-6)
                {
                    t4 = Math.Atan2(T36[2, 2] / s5, T36[0, 2] / s5);
                    t6 = Math.Atan2(T36[1, 1] / s5, -T36[1, 0] / s5);
                }
                else
                {
                    t4 = 0;
                    t6 = Math.Atan2(T36[2, 1], T36[2, 0]);
                }

                solutions[idx][0] = NormalizeAngle(t1 - off[0]);
                solutions[idx][1] = NormalizeAngle(t2 - off[1]);
                solutions[idx][2] = NormalizeAngle(t3 - off[2]);
                solutions[idx][3] = NormalizeAngle(t4 - off[3]);
                solutions[idx][4] = NormalizeAngle(t5 - off[4]);
                solutions[idx][5] = NormalizeAngle(t6 - off[5]);

                var within = true;
                for (var j = 0; j < 6; j++)
                {
                    if (solutions[idx][j] < _minAnglesRad[j] || solutions[idx][j] > _maxAnglesRad[j]) { within = false; break; }
                }
                if (within)
                {
                    flags[idx] = true;
                    if (verbose == 1 && !printed)
                    {
                        Console.WriteLine("--- InverseGeometric first valid ---");
                        Forward(solutions[idx], 1);
                        printed = true;
                    }
                }
                idx++;
            }
        }
        return solutions;
    }

    public double[] InverseJacobian(double[,] T_target, double[] q_initial, out bool success, int maxIterations = 1000, double tolerance = 1e-6, double alpha = 0.1, int verbose = 0)
    {
        double xt = _tool[0], yt = _tool[1], zt = _tool[2];
        double zx = _tool[3], zy = _tool[4], zz = _tool[5];
        var R_tool_row = BuildToolRotationRowMajor(zx, zy, zz, ToolReferenceAxis.FlangeY);
        var T_tool = new double[4, 4];
        for (var r = 0; r < 3; r++)
        {
            T_tool[r, 0] = R_tool_row[0, r];
            T_tool[r, 1] = R_tool_row[1, r];
            T_tool[r, 2] = R_tool_row[2, r];
        }
        T_tool[0, 3] = xt; T_tool[1, 3] = yt; T_tool[2, 3] = zt; T_tool[3, 3] = 1;

        var T_tool_inv = InvertHomogeneous(T_tool);
        var _ = MatMul4(T_target, T_tool_inv); // kept for parity

        var q = (double[])q_initial.Clone();
        var dX = new double[6];

        for (var iter = 0; iter < maxIterations; iter++)
        {
            var T_cur = Forward(q, 0);
            dX[0] = T_target[0, 3] - T_cur[0, 3];
            dX[1] = T_target[1, 3] - T_cur[1, 3];
            dX[2] = T_target[2, 3] - T_cur[2, 3];

            double[] n_c = { T_cur[0, 0], T_cur[1, 0], T_cur[2, 0] };
            double[] o_c = { T_cur[0, 1], T_cur[1, 1], T_cur[2, 1] };
            double[] a_c = { T_cur[0, 2], T_cur[1, 2], T_cur[2, 2] };
            double[] n_t = { T_target[0, 0], T_target[1, 0], T_target[2, 0] };
            double[] o_t = { T_target[0, 1], T_target[1, 1], T_target[2, 1] };
            double[] a_t = { T_target[0, 2], T_target[1, 2], T_target[2, 2] };

            var cn = Cross3(n_c, n_t);
            var co = Cross3(o_c, o_t);
            var ca = Cross3(a_c, a_t);
            dX[3] = 0.5 * (cn[0] + co[0] + ca[0]);
            dX[4] = 0.5 * (cn[1] + co[1] + ca[1]);
            dX[5] = 0.5 * (cn[2] + co[2] + ca[2]);

            double err = 0; for (var k = 0; k < 6; k++) err += dX[k] * dX[k];
            if (err < tolerance * tolerance)
            {
                success = true;
                if (verbose == 1)
                {
                    Console.WriteLine("--- InverseJacobian converged ---");
                    Forward(q, 1);
                }
                return q;
            }

            var J = CalculateJacobian(q);
            var JT = Transpose(J);
            var dq = MatVecMul(JT, dX);
            for (var j = 0; j < 6; j++)
                q[j] = NormalizeAngle(q[j] + alpha * dq[j]);
        }
        success = false;
        return q;
    }

    private double[,] CalculateJacobian(double[] q)
    {
        var J = new double[6, 6];
        var T_eff = Forward(q, 0);
        double[] p_eff = { T_eff[0, 3], T_eff[1, 3], T_eff[2, 3] };

        var alpha = _mdh.Alpha; var a = _mdh.A; var d = _mdh.D; var offset = _mdh.Offset;
        var origins = new double[7][];
        var axes = new double[7][,];
        origins[0] = [0.0, 0.0, 0.0];
        axes[0] = new double[,] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

        for (var i = 0; i < 6; i++)
        {
            var theta = q[i] + offset[i];
            double ct = Math.Cos(theta), st = Math.Sin(theta);
            double ca = Math.Cos(alpha[i]), sa = Math.Sin(alpha[i]);

            double[] Xprev = { axes[i][0, 0], axes[i][0, 1], axes[i][0, 2] };
            double[] Yprev = { axes[i][1, 0], axes[i][1, 1], axes[i][1, 2] };
            double[] Zprev = { axes[i][2, 0], axes[i][2, 1], axes[i][2, 2] };

            var Xi = new double[3];
            Xi[0] = ct * Xprev[0] + st * (ca * Yprev[0] + sa * Zprev[0]);
            Xi[1] = ct * Xprev[1] + st * (ca * Yprev[1] + sa * Zprev[1]);
            Xi[2] = ct * Xprev[2] + st * (ca * Yprev[2] + sa * Zprev[2]);

            var Yi = new double[3];
            Yi[0] = -st * Xprev[0] + ct * (ca * Yprev[0] + sa * Zprev[0]);
            Yi[1] = -st * Xprev[1] + ct * (ca * Yprev[1] + sa * Zprev[1]);
            Yi[2] = -st * Xprev[2] + ct * (ca * Yprev[2] + sa * Zprev[2]);

            var Zi = new double[3];
            Zi[0] = -sa * Yprev[0] + ca * Zprev[0];
            Zi[1] = -sa * Yprev[1] + ca * Zprev[1];
            Zi[2] = -sa * Yprev[2] + ca * Zprev[2];

            axes[i + 1] = new double[,]
            {
                { Xi[0], Xi[1], Xi[2] },
                { Yi[0], Yi[1], Yi[2] },
                { Zi[0], Zi[1], Zi[2] }
            };

            origins[i + 1] = new double[3];
            origins[i + 1][0] = origins[i][0] + a[i] * Xprev[0] + d[i] * Zi[0];
            origins[i + 1][1] = origins[i][1] + a[i] * Xprev[1] + d[i] * Zi[1];
            origins[i + 1][2] = origins[i][2] + a[i] * Xprev[2] + d[i] * Zi[2];

            double[] z_i = { axes[i][2, 0], axes[i][2, 1], axes[i][2, 2] };
            var p_i = origins[i];
            double[] p_diff = { p_eff[0] - p_i[0], p_eff[1] - p_i[1], p_eff[2] - p_i[2] };
            var Jv = Cross3(z_i, p_diff);

            J[0, i] = Jv[0]; J[1, i] = Jv[1]; J[2, i] = Jv[2];
            J[3, i] = z_i[0]; J[4, i] = z_i[1]; J[5, i] = z_i[2];
        }
        return J;
    }

    private static double NormalizeAngle(double a)
    {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }

    // --- Tool rotation via RMLib (row-major) ---
    private static double[,] BuildToolRotationRowMajor(double zx, double zy, double zz, ToolReferenceAxis refAxis)
    {
        var primary = refAxis == ToolReferenceAxis.FlangeY
            ? new RowMeasureUtility.RMLib.Vec3(0, 1, 0)
            : new RowMeasureUtility.RMLib.Vec3(1, 0, 0);
        var alt = refAxis == ToolReferenceAxis.FlangeY
            ? new RowMeasureUtility.RMLib.Vec3(1, 0, 0)
            : new RowMeasureUtility.RMLib.Vec3(0, 1, 0);
        return RowMeasureUtility.RMLib.BuildRowMajorFromZ(
            new RowMeasureUtility.RMLib.Vec3(zx, zy, zz),
            primary,
            alt);
    }

    // --- Classic DH 4x4 (local, to avoid LibD dependency here) ---
    private static double[,] DHTransform(double theta, double d, double a, double alpha)
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

    // --- 4x4 multiply ---
    private static double[,] MatMul4(double[,] A, double[,] B)
    {
        var C = new double[4, 4];
        for (var i = 0; i < 4; i++)
            for (var j = 0; j < 4; j++)
                for (var k = 0; k < 4; k++)
                    C[i, j] += A[i, k] * B[k, j];
        return C;
    }

    // --- Homogeneous inverse (rotation orthonormal assumption) ---
    private static double[,] InvertHomogeneous(double[,] T)
    {
        var R = new double[3, 3];
        for (var r = 0; r < 3; r++)
            for (var c = 0; c < 3; c++)
                R[r, c] = T[r, c];
        // Transpose
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

    // --- Cross product (double[] size 3) ---
    private static double[] Cross3(double[] a, double[] b) =>
        [
            a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0]
        ];

    // --- Transpose generic matrix ---
    private static double[,] Transpose(double[,] M)
    {
        var rows = M.GetLength(0);
        var cols = M.GetLength(1);
        var T = new double[cols, rows];
        for (var r = 0; r < rows; r++)
            for (var c = 0; c < cols; c++)
                T[c, r] = M[r, c];
        return T;
    }

    // --- Matrix-vector multiply ---
    private static double[] MatVecMul(double[,] M, double[] v)
    {
        var rows = M.GetLength(0);
        var cols = M.GetLength(1);
        var r = new double[rows];
        for (var i = 0; i < rows; i++)
            for (var j = 0; j < cols; j++)
                r[i] += M[i, j] * v[j];
        return r;
    }
}
