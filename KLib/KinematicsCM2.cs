using System;
using System.Collections.Generic;
using System.Linq;
using ColumnMeasureUtility;

namespace KinematicsCM2;

// MDH parameter container
public class MDHParameters
{
    public double[] Alpha { get; }
    public double[] A { get; }
    public double[] D { get; }
    public double[] Offset { get; }
    public double[] MinAnglesDeg { get; } // Joint limits in degrees
    public double[] MaxAnglesDeg { get; } // Joint limits in degrees

    public MDHParameters(double[] alpha, double[] a, double[] d, double[] offset, double[]? minAngles = null, double[]? maxAngles = null)
    {
        if (alpha.Length != 6 || a.Length != 6 || d.Length != 6 || offset.Length != 6)
            throw new ArgumentException("All MDH parameter arrays must have 6 elements.");
        Alpha = alpha;
        A = a;
        D = d;
        Offset = offset;

        // Use default wide limits if not provided
        MinAnglesDeg = minAngles ?? [-360, -360, -360, -360, -360, -360];
        MaxAnglesDeg = maxAngles ?? [360, 360, 360, 360, 360, 360];
    }
}

// Factory class for robot creation
public static class ManipulatorFactory
{
    // ロボット名とMDHパラメータのマッピング
    private static readonly Dictionary<string, MDHParameters> _mdhTable = new(StringComparer.OrdinalIgnoreCase)
    {
        // "MiRobot" のMDH定義（デフォルト）
        ["MiRobot"] = new MDHParameters(
            [0, Math.PI / 2, Math.PI, -Math.PI / 2, Math.PI / 2, Math.PI / 2],
            [0, 29.69, 108, 20, 0, 0],
            [127, 0, 0, 168.98, 0, 24.29],
            [0, Math.PI / 2, 0, 0, -Math.PI / 2, 0],
            [-170, -125, -155, -180, -120, -180], // Min Angles [deg]
            [170, 125, 155, 180, 120, 180]  // Max Angles [deg]
        )
    };

    // ツール名とツール情報のマッピング (位置 ＋ ベクトルonTool座標)
    private static readonly Dictionary<string, double[]> _toolTable = new(StringComparer.OrdinalIgnoreCase)
    {
        // "null" = [tx, ty, tz, vx, vy, vz] (vx,vy,vz is Z-axis vector)
        ["null"] = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        // "test0" = 45deg rotation around Y-axis
        ["tool0"] = [1.41421356, 0.0, 2.41421356, 0.70710678, 0.0, 0.70710678]
    };

    /// <summary>
    /// 指定したロボット名・ツール名のManipulator6DoFインスタンスを返す
    /// </summary>
    public static Manipulator6DoF CreateManipulator(
        string? robotName = null,
        string? toolName = null
    )
    {
        var rname = string.IsNullOrWhiteSpace(robotName) ? "MiRobot" : robotName;
        var tname = string.IsNullOrWhiteSpace(toolName) ? "null" : toolName;

        if (!_mdhTable.TryGetValue(rname, out var mdh))
            throw new ArgumentException($"Unknown robot name: {rname}");
        if (!_toolTable.TryGetValue(tname, out var tool))
            throw new ArgumentException($"Unknown tool name: {tname}");

        return new Manipulator6DoF(mdh, tool);
    }

    /// <summary>
    /// デフォルト生成（MiRobot＋nullツール）
    /// </summary>
    public static Manipulator6DoF CreateDefaultManipulator()
        => CreateManipulator("MiRobot", "null");

    /// <summary>
    /// ロボット名一覧を取得
    /// </summary>
    public static IEnumerable<string> GetAvailableRobotNames()
        => _mdhTable.Keys;

    /// <summary>
    /// ツール名一覧を取得
    /// </summary>
    public static IEnumerable<string> GetAvailableToolNames()
        => _toolTable.Keys;
}


public class Manipulator6DoF
{
    // === Added enum (place near other type-level declarations if desired) ===
    public enum ToolReferenceAxis
    {
        // フランジ座標と、ツール座標のZ軸方向は一致させる前提で、残り2軸のいずれをプライマリリファレンス人するかを選択する。FlangeYを選択すると、現状、フランジ座標とツール座標の向きは同じとなる。FlangeXを選ぶとフランジ座標Z軸回りに90度回転したものがツール座標になる。
        // Use flange Y axis as primary reference (classic behavior but now via projection)
        FlangeY = 0,
        // Use flange X axis as primary reference
        FlangeX = 1
    }

    private ToolReferenceAxis _toolRefAxis = ToolReferenceAxis.FlangeY;

    private readonly MDHParameters _mdh;
    private readonly double[] _tool;
    private readonly double[] _minAnglesRad;
    private readonly double[] _maxAnglesRad;

    private static readonly double Deg2Rad = Math.PI / 180.0;

    /// <summary>
    /// Constructor
    /// </summary>
    /// <param name="mdh"></param>
    /// <param name="tool"></param>
    /// <exception cref="ArgumentException"></exception>
    public Manipulator6DoF(MDHParameters mdh, double[] tool)
    {
        if (tool == null || tool.Length != 6)
            throw new ArgumentException("Tool must have 6 elements.");
        _mdh = mdh;
        _tool = (double[])tool.Clone();

        // Convert joint limits from degrees to radians for internal use
        _minAnglesRad = _mdh.MinAnglesDeg.Select(a => a * Deg2Rad).ToArray();
        _maxAnglesRad = _mdh.MaxAnglesDeg.Select(a => a * Deg2Rad).ToArray();
    }

    public void SetToolReferenceAxis(ToolReferenceAxis axis) => _toolRefAxis = axis;

    public double[,] Forward(double[] q, int verbose = 0)
    {
        if (q.Length != 6) throw new ArgumentException("q must have 6 elements");

        var alpha = _mdh.Alpha;
        var a = _mdh.A;
        var d = _mdh.D;
        var offset = _mdh.Offset;

        var origins = new double[7][];
        var axes = new double[7][,];

        origins[0] = [0, 0, 0];
        axes[0] = new double[,] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

        for (var i = 0; i < 6; i++)
        {
            var theta = q[i] + offset[i];
            double ct = Math.Cos(theta), st = Math.Sin(theta);
            double ca = Math.Cos(alpha[i]), sa = Math.Sin(alpha[i]);

            double[] Xprev = { axes[i][0, 0], axes[i][1, 0], axes[i][2, 0] };
            double[] Yprev = { axes[i][0, 1], axes[i][1, 1], axes[i][2, 1] };
            double[] Zprev = { axes[i][0, 2], axes[i][1, 2], axes[i][2, 2] };

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

            axes[i + 1] = new double[,] {
                { Xi[0], Yi[0], Zi[0] },
                { Xi[1], Yi[1], Zi[1] },
                { Xi[2], Yi[2], Zi[2] }
            };

            origins[i + 1] = new double[3];
            origins[i + 1][0] = origins[i][0] + a[i] * Xprev[0] + d[i] * Zi[0];
            origins[i + 1][1] = origins[i][1] + a[i] * Xprev[1] + d[i] * Zi[1];
            origins[i + 1][2] = origins[i][2] + a[i] * Xprev[2] + d[i] * Zi[2];
        }

        var T06_flange = new double[4, 4];
        for (var row = 0; row < 3; row++)
        {
            T06_flange[row, 0] = axes[6][row, 0];
            T06_flange[row, 1] = axes[6][row, 1];
            T06_flange[row, 2] = axes[6][row, 2];
            T06_flange[row, 3] = origins[6][row];
        }
        T06_flange[3, 0] = 0; T06_flange[3, 1] = 0; T06_flange[3, 2] = 0; T06_flange[3, 3] = 1;

        double xt = _tool[0], yt = _tool[1], zt = _tool[2];
        double vx = _tool[3], vy = _tool[4], vz = _tool[5];

        var R_tool = GetToolRotationMatrixFromZVectorCM(
            vx, vy, vz,
            referenceAxis: _toolRefAxis);

        var T6_tool = new double[4, 4];
        for (var row = 0; row < 3; row++)
        {
            for (var col = 0; col < 3; col++)
            {
                T6_tool[row, col] = R_tool[row, col];
            }
        }
        T6_tool[0, 3] = xt; T6_tool[1, 3] = yt; T6_tool[2, 3] = zt;
        T6_tool[3, 0] = 0; T6_tool[3, 1] = 0; T6_tool[3, 2] = 0; T6_tool[3, 3] = 1;

        var T0_tool = CMLib.MatMul4x4(T06_flange, T6_tool);

        if (verbose >= 1)
        {
            Console.WriteLine("---- Origins (absolute coords) ----");
            for (var i = 0; i <= 6; i++)
                Console.WriteLine($"O{i}: ({origins[i][0]:F3}, {origins[i][1]:F3}, {origins[i][2]:F3})");
            Console.WriteLine($"Ot: ({T0_tool[0, 3]:F3}, {T0_tool[1, 3]:F3}, {T0_tool[2, 3]:F3})");
        }

        if (verbose >= 2)
        {
            Console.WriteLine("---- Local axes in absolute coordinates ----");
            for (var i = 1; i <= 6; i++)
            {
                Console.WriteLine($"Frame {i}:");
                Console.WriteLine($"  X-axis = ({axes[i][0, 0]:F3}, {axes[i][1, 0]:F3}, {axes[i][2, 0]:F3})");
                Console.WriteLine($"  Y-axis = ({axes[i][0, 1]:F3}, {axes[i][1, 1]:F3}, {axes[i][2, 1]:F3})");
                Console.WriteLine($"  Z-axis = ({axes[i][0, 2]:F3}, {axes[i][1, 2]:F3}, {axes[i][2, 2]:F3})");
            }
            // ツール先端の座標系を追加
            Console.WriteLine("Frame Tool Tip:");
            Console.WriteLine($"  X-axis = ({T0_tool[0, 0]:F3}, {T0_tool[1, 0]:F3}, {T0_tool[2, 0]:F3})");
            Console.WriteLine($"  Y-axis = ({T0_tool[0, 1]:F3}, {T0_tool[1, 1]:F3}, {T0_tool[2, 1]:F3})");
            Console.WriteLine($"  Z-axis = ({T0_tool[0, 2]:F3}, {T0_tool[1, 2]:F3}, {T0_tool[2, 2]:F3})");
        }

        return T0_tool;
    }

    public double[][] InverseGeometric(double[,] T_target, out bool[] flags, int verbose = 0)
    {
        var solutions = new double[8][];
        for (var i = 0; i < 8; i++) solutions[i] = new double[6];
        flags = new bool[8];
        var verboseOutputDone = false;

        double xt = _tool[0], yt = _tool[1], zt = _tool[2];
        double vx = _tool[3], vy = _tool[4], vz = _tool[5];
        var R_tool = GetToolRotationMatrixFromZVectorCM(vx, vy, vz, ToolReferenceAxis.FlangeY);
        var T_tool = new double[4, 4];
        for (var row = 0; row < 3; row++)
            for (var col = 0; col < 3; col++)
                T_tool[row, col] = R_tool[row, col];
        T_tool[0, 3] = xt; T_tool[1, 3] = yt; T_tool[2, 3] = zt;
        T_tool[3, 3] = 1;

        var T_tool_inv = CMLib.MatrixInverse4x4(T_tool);
        var T_target_flange = CMLib.MatMul4x4(T_target, T_tool_inv);

        var a = _mdh.A;
        var d = _mdh.D;
        var offset = _mdh.Offset;

        var current_d1 = d[0];
        var current_a2 = a[1];
        var current_a3 = a[2];
        var current_d4 = d[3];
        var current_d6 = d[5];

        var ax_flange = T_target_flange[0, 2];
        var ay_flange = T_target_flange[1, 2];
        var az_flange = T_target_flange[2, 2];
        var Px_flange = T_target_flange[0, 3];
        var Py_flange = T_target_flange[1, 3];
        var Pz_flange = T_target_flange[2, 3];

        var Pwx = Px_flange - current_d6 * ax_flange;
        var Pwy = Py_flange - current_d6 * ay_flange;
        var Pwz = Pz_flange - current_d6 * az_flange;

        var theta1_1 = Math.Atan2(Pwy, Pwx);
        var theta1_2 = theta1_1 + Math.PI;

        var r_sq = Pwx * Pwx + Pwy * Pwy;
        var s_sq = (Pwz - current_d1) * (Pwz - current_d1);
        var D_sq = r_sq + s_sq;
        var cos_theta3_num = (D_sq - current_a2 * current_a2 - current_a3 * current_a3 - current_d4 * current_d4);
        var cos_theta3_den = 2 * current_a2 * Math.Sqrt(current_a3 * current_a3 + current_d4 * current_d4);
        var phi = Math.Atan2(current_d4, current_a3);

        if (Math.Abs(cos_theta3_den) < 1e-9) return solutions;
        var cos_theta3_base = cos_theta3_num / cos_theta3_den;
        if (Math.Abs(cos_theta3_base) > 1) return solutions;

        var theta3_base = Math.Acos(cos_theta3_base);
        var theta3_1 = theta3_base - phi;
        var theta3_2 = -theta3_base - phi;

        var combinations = new[] { (theta1_1, theta3_1), (theta1_1, theta3_2), (theta1_2, theta3_1), (theta1_2, theta3_2) };
        var sol_idx = 0;

        for (var i = 0; i < combinations.Length; i++)
        {
            var t1 = combinations[i].Item1;
            var t3 = combinations[i].Item2;

            var s1 = Math.Sin(t1);
            var c1 = Math.Cos(t1);
            var s3 = Math.Sin(t3 + phi);
            var c3 = Math.Cos(t3 + phi);

            var k1 = current_a2 + Math.Sqrt(current_a3 * current_a3 + current_d4 * current_d4) * c3;
            var k2 = Math.Sqrt(current_a3 * current_a3 + current_d4 * current_d4) * s3;
            var den_t2 = k1 * k1 + k2 * k2;
            var c2 = (k1 * (c1 * Pwx + s1 * Pwy) + k2 * (Pwz - current_d1)) / den_t2;
            var s2 = (k2 * (c1 * Pwx + s1 * Pwy) - k1 * (Pwz - current_d1)) / den_t2;
            var t2 = Math.Atan2(s2, c2);

            var T01 = CMLib.T(t1 + offset[0], d[0], a[0], _mdh.Alpha[0]);
            var T12 = CMLib.T(t2 + offset[1], d[1], a[1], _mdh.Alpha[1]);
            var T23 = CMLib.T(t3 + offset[2], d[2], a[2], _mdh.Alpha[2]);
            var T03 = CMLib.MatMul4x4(CMLib.MatMul4x4(T01, T12), T23);
            var T03_inv = CMLib.MatrixInverse4x4(T03);
            var T36 = CMLib.MatMul4x4(T03_inv, T_target_flange);

            var cos_t5 = T36[1, 2];
            if (Math.Abs(cos_t5) > 1) { sol_idx += 2; continue; }

            var t5_1 = Math.Acos(cos_t5);
            var t5_2 = -t5_1;
            var t5_solutions = new[] { t5_1, t5_2 };

            foreach (var t5 in t5_solutions)
            {
                double t4, t6;
                var sin_t5 = Math.Sin(t5);
                if (Math.Abs(sin_t5) > 1e-6)
                {
                    t4 = Math.Atan2(T36[2, 2] / sin_t5, T36[0, 2] / sin_t5);
                    t6 = Math.Atan2(T36[1, 1] / sin_t5, -T36[1, 0] / sin_t5);
                }
                else
                {
                    t4 = 0;
                    t6 = Math.Atan2(T36[2, 1], T36[2, 0]);
                }

                solutions[sol_idx][0] = t1;
                solutions[sol_idx][1] = t2;
                solutions[sol_idx][2] = t3;
                solutions[sol_idx][3] = t4;
                solutions[sol_idx][4] = t5;
                solutions[sol_idx][5] = t6;

                for (var j = 0; j < 6; j++) solutions[sol_idx][j] = CMLib.NormalizeAngle(solutions[sol_idx][j] - offset[j]);

                var within_limits = true;
                for (var j = 0; j < 6; j++)
                {
                    if (solutions[sol_idx][j] < _minAnglesRad[j] || solutions[sol_idx][j] > _maxAnglesRad[j])
                    {
                        within_limits = false;
                        break;
                    }
                }

                if (within_limits)
                {
                    flags[sol_idx] = true;
                    if (verbose == 1 && !verboseOutputDone)
                    {
                        Console.WriteLine("\n--- InverseGeometric KinematicsCM (Geometric) Debug Output ---");
                        Forward(solutions[sol_idx], 1);
                        verboseOutputDone = true;
                    }
                }
                sol_idx++;
            }
        }
        return solutions;
    }

    public double[] InverseJacobian(double[,] T_target, double[] q_initial, out bool success, int maxIterations = 1000, double tolerance = 1e-6, double alpha = 0.1, int verbose = 0)
    {
        double xt = _tool[0], yt = _tool[1], zt = _tool[2];
        double vx = _tool[3], vy = _tool[4], vz = _tool[5];
        var R_tool = GetToolRotationMatrixFromZVectorCM(vx, vy, vz, ToolReferenceAxis.FlangeY);
        var T_tool = new double[4, 4];
        for (var row = 0; row < 3; row++)
            for (var col = 0; col < 3; col++)
                T_tool[row, col] = R_tool[row, col];
        T_tool[0, 3] = xt; T_tool[1, 3] = yt; T_tool[2, 3] = zt;
        T_tool[3, 3] = 1;

        var T_tool_inv = CMLib.MatrixInverse4x4(T_tool);
        var T_target_flange = CMLib.MatMul4x4(T_target, T_tool_inv);

        var q = (double[])q_initial.Clone();
        var dX = new double[6];

        for (var i = 0; i < maxIterations; i++)
        {
            var T_current = Forward(q, 0);
            dX[0] = T_target[0, 3] - T_current[0, 3];
            dX[1] = T_target[1, 3] - T_current[1, 3];
            dX[2] = T_target[2, 3] - T_current[2, 3];

            double[] n_curr = { T_current[0, 0], T_current[1, 0], T_current[2, 0] };
            double[] o_curr = { T_current[0, 1], T_current[1, 1], T_current[2, 1] };
            double[] a_curr = { T_current[0, 2], T_current[1, 2], T_current[2, 2] };
            double[] n_targ = { T_target[0, 0], T_target[1, 0], T_target[2, 0] };
            double[] o_targ = { T_target[0, 1], T_target[1, 1], T_target[2, 1] };
            double[] a_targ = { T_target[0, 2], T_target[1, 2], T_target[2, 2] };

            var rot_err = new double[3];
            rot_err[0] = 0.5 * (CMLib.CrossProduct(n_curr, n_targ)[0] + CMLib.CrossProduct(o_curr, o_targ)[0] + CMLib.CrossProduct(a_curr, a_targ)[0]);
            rot_err[1] = 0.5 * (CMLib.CrossProduct(n_curr, n_targ)[1] + CMLib.CrossProduct(o_curr, o_targ)[1] + CMLib.CrossProduct(a_curr, a_targ)[1]);
            rot_err[2] = 0.5 * (CMLib.CrossProduct(n_curr, n_targ)[2] + CMLib.CrossProduct(o_curr, o_targ)[2] + CMLib.CrossProduct(a_curr, a_targ)[2]);
            dX[3] = rot_err[0]; dX[4] = rot_err[1]; dX[5] = rot_err[2];

            double errorNorm = 0;
            foreach (var val in dX) errorNorm += val * val;
            if (errorNorm < tolerance)
            {
                success = true;
                if (verbose == 1)
                {
                    Console.WriteLine("\n--- InverseGeometric KinematicsCM (Jacobian) Debug Output ---");
                    Forward(q, 1);
                }
                return q;
            }

            var J = CalculateJacobian(q);
            var J_T = CMLib.Transpose(J);
            var dq = CMLib.MatVecMul(J_T, dX);
            for (var j = 0; j < dq.Length; j++) dq[j] *= alpha;
            for (var j = 0; j < q.Length; j++) q[j] = CMLib.NormalizeAngle(q[j] + dq[j]);
        }
        success = false;
        return q;
    }

    private double[,] CalculateJacobian(double[] q)
    {
        var J = new double[6, 6];
        var T_eff = Forward(q, 0);
        double[] p_eff = { T_eff[0, 3], T_eff[1, 3], T_eff[2, 3] };

        var alpha = _mdh.Alpha;
        var a = _mdh.A;
        var d = _mdh.D;
        var offset = _mdh.Offset;

        var origins = new double[7][];
        var axes = new double[7][,];
        origins[0] = [0, 0, 0];
        axes[0] = new double[,] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

        for (var i = 0; i < 6; i++)
        {
            var theta = q[i] + offset[i];
            double ct = Math.Cos(theta), st = Math.Sin(theta);
            double ca = Math.Cos(alpha[i]), sa = Math.Sin(alpha[i]);

            double[] Xprev = { axes[i][0, 0], axes[i][1, 0], axes[i][2, 0] };
            double[] Yprev = { axes[i][0, 1], axes[i][1, 1], axes[i][2, 1] };
            double[] Zprev = { axes[i][0, 2], axes[i][1, 2], axes[i][2, 2] };

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

            axes[i + 1] = new double[,] { { Xi[0], Yi[0], Zi[0] }, { Xi[1], Yi[1], Zi[1] }, { Xi[2], Yi[2], Zi[2] } };
            origins[i + 1] = new double[3];
            origins[i + 1][0] = origins[i][0] + a[i] * Xprev[0] + d[i] * Zi[0];
            origins[i + 1][1] = origins[i][1] + a[i] * Xprev[1] + d[i] * Zi[1];
            origins[i + 1][2] = origins[i][2] + a[i] * Xprev[2] + d[i] * Zi[2];

            double[] z_i = { axes[i][0, 2], axes[i][1, 2], axes[i][2, 2] };
            var p_i = origins[i];
            double[] p_diff = { p_eff[0] - p_i[0], p_eff[1] - p_i[1], p_eff[2] - p_i[2] };
            var Jv_i = CMLib.CrossProduct(z_i, p_diff);
            J[0, i] = Jv_i[0]; J[1, i] = Jv_i[1]; J[2, i] = Jv_i[2];
            J[3, i] = z_i[0]; J[4, i] = z_i[1]; J[5, i] = z_i[2];
        }
        return J;
    }

    /// <summary>
    /// Build a tool rotation matrix from a supplied Z direction (tool approach) expressed in the
    /// flange frame. Instead of hard-coded up vectors (0,1,0) / (1,0,0), we:
    /// 1. Pick a reference flange axis (Y or X) per configuration.
    /// 2. Project that reference axis onto the plane perpendicular to Z to define X (roll = 0).
    /// 3. If projection degenerates, fall back to the other flange axis, then finally a static fallback.
    /// 4. Compute Y = Z × X.
    /// Returns a 3x3 rotation whose columns are (X, Y, Z) in flange coordinates.
    /// </summary>
    private static double[,] GetToolRotationMatrixFromZVectorCM(
        double zx, double zy, double zz,
        ToolReferenceAxis referenceAxis = ToolReferenceAxis.FlangeY)
    {
        // 1. Normalize Z
        var n = Math.Sqrt(zx * zx + zy * zy + zz * zz);
        if (n < 1e-12)
        {
            // Degenerate: return identity (flange axes)
            return new double[3, 3] {
                { 1, 0, 0 },
                { 0, 1, 0 },
                { 0, 0, 1 }
            };
        }
        double Zx = zx / n, Zy = zy / n, Zz = zz / n;

        // 2. Select primary reference axis (in flange frame these are canonical unit vectors)
        //    We keep code general in case you later pre-transform them.
        var refA = referenceAxis == ToolReferenceAxis.FlangeY
            ? [0.0, 1.0, 0.0] // Flange Y
            : new[] { 1.0, 0.0, 0.0 }; // Flange X

        // 3. Project refA onto plane orthogonal to Z: Xcand = refA - (refA·Z)Z
        var dotRefZ = refA[0] * Zx + refA[1] * Zy + refA[2] * Zz;
        var Xcx = refA[0] - dotRefZ * Zx;
        var Xcy = refA[1] - dotRefZ * Zy;
        var Xcz = refA[2] - dotRefZ * Zz;

        var xn = Math.Sqrt(Xcx * Xcx + Xcy * Xcy + Xcz * Xcz);

        if (xn < 1e-9)
        {
            // 4. Fallback to the other flange axis
            var alt = referenceAxis == ToolReferenceAxis.FlangeY
                ? [1.0, 0.0, 0.0]
                : new[] { 0.0, 1.0, 0.0 };

            var dotAltZ = alt[0] * Zx + alt[1] * Zy + alt[2] * Zz;
            Xcx = alt[0] - dotAltZ * Zx;
            Xcy = alt[1] - dotAltZ * Zy;
            Xcz = alt[2] - dotAltZ * Zz;
            xn = Math.Sqrt(Xcx * Xcx + Xcy * Xcy + Xcz * Xcz);

            if (xn < 1e-9)
            {
                // 5. Final static fallback (choose a vector not parallel to Z)
                var staticAlt = Math.Abs(Zz) < 0.9
                    ? [0.0, 0.0, 1.0]
                    : new[] { 0.0, 1.0, 0.0 };
                var dotStatic = staticAlt[0] * Zx + staticAlt[1] * Zy + staticAlt[2] * Zz;
                Xcx = staticAlt[0] - dotStatic * Zx;
                Xcy = staticAlt[1] - dotStatic * Zy;
                Xcz = staticAlt[2] - dotStatic * Zz;
                xn = Math.Sqrt(Xcx * Xcx + Xcy * Xcy + Xcz * Xcz);

                if (xn < 1e-12)
                {
                    // If still degenerate, return identity
                    return new double[3, 3] {
                        { 1, 0, 0 },
                        { 0, 1, 0 },
                        { 0, 0, 1 }
                    };
                }
            }
        }

        // Normalize X
        double Xx = Xcx / xn, Xy = Xcy / xn, Xz = Xcz / xn;

        // 6. Y = Z × X
        var Yx = Zy * Xz - Zz * Xy;
        var Yy = Zz * Xx - Zx * Xz;
        var Yz = Zx * Xy - Zy * Xx;

        // (Optional) Normalize Y for numerical safety
        var yn = Math.Sqrt(Yx * Yx + Yy * Yy + Yz * Yz);
        if (yn > 1e-12)
        {
            Yx /= yn; Yy /= yn; Yz /= yn;
        }

        // 7. Return rotation with columns (X,Y,Z)
        return new double[3, 3]
        {
            { Xx, Yx, Zx },
            { Xy, Yy, Zy },
            { Xz, Yz, Zz }
        };
    }
}