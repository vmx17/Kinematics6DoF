using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using RowMeasureUtility;

namespace KinematicsRM3
{
    // ============================================================
    // MDHParameters: includes Theta0, fixed elbow link, base/tool transforms
    // ============================================================
    public sealed class MDHParameters
    {
        public double[] Alpha;   // α_{i-1}
        public double[] A;       // a_{i-1}
        public double[] D;       // d_i
        public double[] Theta0;  // θ_i オフセット
        public double[] MinDeg;
        public double[] MaxDeg;

        // 固定肘リンク（Σ3→Σ3e→Σ4で使用）
        public double ElbowAlpha;
        public double ElbowA;
        public double ElbowD;
        public double ElbowTheta;

        // 設置（ベース）変換：world←base
        public double[,] T_base;

        public MDHParameters(double[] alpha, double[] a, double[] d, double[] theta0,
                             double[] minDeg, double[] maxDeg,
                             double elbowAlpha, double elbowA, double elbowD, double elbowTheta,
                             double[,] Tbase)
        {
            Alpha = alpha; A = a; D = d; Theta0 = theta0;
            MinDeg = minDeg; MaxDeg = maxDeg;
            ElbowAlpha = elbowAlpha; ElbowA = elbowA; ElbowD = elbowD; ElbowTheta = elbowTheta;
            T_base = Tbase;
        }
    }


    // ============================================================
    // ManipulatorFactory
    // ============================================================
    public static class ManipulatorFactory
    {
        // ロボット定義
        private static readonly Dictionary<string, MDHParameters> _mdhTable = new(StringComparer.OrdinalIgnoreCase)
        {
            ["MiRobot"] = new MDHParameters(
                alpha: [0.0, Math.PI / 2, 0.0, -Math.PI / 2, Math.PI, 0.0],
                a: [0.0, 29.69, 108.0, 168.98, 0.0, 24.29],
                d: [127.0, 0.0, 0.0, 0.0, 0.0, 100.0], // 例として d6=100
                theta0: [0.0, Math.PI / 2, -Math.PI / 2, -Math.PI, -Math.PI / 2, 0.0],
                minDeg: [-180.0, -180, -90, -180, -180, -360],
                maxDeg: [180.0, 90, 180, 180, 90, 360],
                elbowAlpha: Math.PI / 2, elbowA: 0.0, elbowD: -20.0, elbowTheta: 0.0,
                Tbase: Math4.Identity() // 据付：必要ならここを任意の同次行列に
            )
        };

        // ツール定義（[px,py,pz,zx,zy,zz]：TCP原点とツールZ）
        private static readonly Dictionary<string, double[]> _toolTable = new(StringComparer.OrdinalIgnoreCase)
        {
            ["null"] = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        };

        // 拡張API
        public static void RegisterRobot(string name, MDHParameters mdh) => _mdhTable[name] = mdh;
        public static void RegisterTool(string name, double[] tool) => _toolTable[name] = tool;

        // 生成（デフォルト MiRobot／null を必ず提供）
        public static Manipulator6DoF CreateManipulator(string robotName = "MiRobot", string toolName = "null")
        {
            var mdh = _mdhTable[robotName];
            var tool = _toolTable[toolName];
            return new Manipulator6DoF(mdh, tool);
        }
    }

    // ============================================================
    // Manipulator6DoF
    // ============================================================
    public sealed class Manipulator6DoF
    {
        private readonly MDHParameters _mdh;
        private readonly double[] _tool; // [px,py,pz,zx,zy,zz]
        private readonly double[] _minAnglesRad;
        private readonly double[] _maxAnglesRad;
        // Manipulator6DoF fields
        public double PositionTolerance { get; set; } = 1e-3;
        public double DirectionTolerance { get; set; } = 1e-3;

        public Manipulator6DoF(MDHParameters mdh, double[] tool)
        {
            _mdh = mdh;
            _tool = (double[])tool.Clone();
            _minAnglesRad = _mdh.MinDeg.Select(d => d * Math.PI / 180.0).ToArray();
            _maxAnglesRad = _mdh.MaxDeg.Select(d => d * Math.PI / 180.0).ToArray();
        }

        // Σ0→Σ6 (フランジ) のチェーン（肘リンク含む）
        private double[,] ForwardMatrixChain(double[] theta)
        {
            var T = Math4.Identity();
            for (var i = 0; i < 6; i++)
            {
                var Ti = Math4.BuildLinkTransform(_mdh.Alpha[i], _mdh.A[i], _mdh.Theta0[i] + theta[i], _mdh.D[i]);
                T = Math4.MatMul(T, Ti);
                if (i == 2)
                {
                    var Te = Math4.BuildLinkTransform(_mdh.ElbowAlpha, _mdh.ElbowA, _mdh.ElbowTheta, _mdh.ElbowD);
                    T = Math4.MatMul(T, Te);
                }
            }
            return T;
        }

        // World←TCP (ベースとツールを含む)
        public double[,] ForwardMatrixWorld(double[] theta)
        {
            var Tchain = ForwardMatrixChain(theta);
            // ツールを同次行列に変換
            var Ttool = Math4.Identity();
            Ttool[0, 3] = _tool[0];
            Ttool[1, 3] = _tool[1];
            Ttool[2, 3] = _tool[2];
            // ツール方向ベクトルは Forward() で別途扱う
            return Math4.MatMul(Math4.MatMul(_mdh.T_base, Tchain), Ttool);
        }

        // FK: [px,py,pz,zx,zy,zz] in world
        public double[] Forward(double[] theta, int verbose = 0)
        {
            var T = Math4.Identity();
            var origins = new List<double[]>();
            var transforms = new List<double[,]>();

            // Σ0（ベース）を初期化
            origins.Add(new[] { 0.0, 0.0, 0.0 });
            transforms.Add(Math4.Identity());

            // 各リンクを順に適用
            for (var i = 0; i < 6; i++)
            {
                var Ti = Math4.BuildLinkTransform(_mdh.Alpha[i], _mdh.A[i], _mdh.Theta0[i] + theta[i], _mdh.D[i]);
                T = Math4.MatMul(T, Ti);
                if (i == 2)
                {
                    var Te = Math4.BuildLinkTransform(_mdh.ElbowAlpha, _mdh.ElbowA, _mdh.ElbowTheta, _mdh.ElbowD);
                    T = Math4.MatMul(T, Te);
                }
                origins.Add([T[0, 3], T[1, 3], T[2, 3]]);
                transforms.Add((double[,])T.Clone());
            }

            // TCP位置と方向
            var Tworld = ForwardMatrixWorld(theta);
            var p = Math4.MulPoint(Tworld, [0.0, 0.0, 0.0]);
            var z = Math4.MulDir(Tworld, [_tool[3], _tool[4], _tool[5]]);
            NormalizeInPlace(z);

            // verbose 出力
            if (verbose >= 1)
            {
                for (var i = 0; i < origins.Count; i++)
                {
                    Console.WriteLine($"Σ{i} origin: ({origins[i][0]:F3}, {origins[i][1]:F3}, {origins[i][2]:F3})");
                    if (verbose >= 2)
                    {
                        var M = transforms[i];
                        Console.WriteLine($"Σ{i} transform:");
                        for (var r = 0; r < 4; r++)
                            Console.WriteLine($"{M[r, 0]:F3} {M[r, 1]:F3} {M[r, 2]:F3} {M[r, 3]:F3}");
                    }
                }
            }

            return [p[0], p[1], p[2], z[0], z[1], z[2]];
        }

        // 幾何学的IK: 入力は world←tcp
        public double[][] InverseGeometric(double[,] T_target_world, out bool[] flags, int verbose = 0)
        {
            // world←tcp → base←flange に変換
            var Tbase_inv = Math4.InvertHomogeneous(_mdh.T_base);
            var Ttool_inv = Math4.Identity();
            Ttool_inv[0, 3] = -_tool[0];
            Ttool_inv[1, 3] = -_tool[1];
            Ttool_inv[2, 3] = -_tool[2];
            var T_target_flange_base = Math4.MatMul(Tbase_inv, Math4.MatMul(T_target_world, Ttool_inv));

            // 幾何IK本体（省略せず実装）
            var a = _mdh.A; var d = _mdh.D; var off = _mdh.Theta0;
            double a3 = a[2], a4 = a[3], d4 = d[3], d6 = d[5];
            var Lw = Math.Sqrt(a4 * a4 + d4 * d4);
            var psi = Math.Atan2(d4, a4);

            double Px = T_target_flange_base[0, 3], Py = T_target_flange_base[1, 3], Pz = T_target_flange_base[2, 3];
            double ZX = T_target_flange_base[0, 2], ZY = T_target_flange_base[1, 2], ZZ = T_target_flange_base[2, 2];
            double PwX = Px - d6 * ZX, PwY = Py - d6 * ZY, PwZ = Pz - d6 * ZZ;

            var solutions = new List<double[]>();
            var q1cands = new[]{
                NormalizeAngleSigned(Math.Atan2(PwY, PwX)),
                NormalizeAngleSigned(Math.Atan2(PwY, PwX)+Math.PI)
            };

            foreach (var q1 in q1cands)
            {
                if (!WithinLimit(0, q1)) continue;
                var q2Roots = SolveQ2ByRadius(q1, PwX, PwY, PwZ, a3, Lw, 61, verbose);
                foreach (var q2 in q2Roots)
                {
                    if (!WithinLimit(1, q2)) continue;
                    var W2 = PwToSigma2(q1, q2, PwX, PwY, PwZ);
                    double dx = W2[0] - a3, vy = W2[1];
                    if (Math.Abs(dx * dx + vy * vy - Lw * Lw) > 1e-2) continue;
                    var q3_eff = Math.Atan2(vy, dx);
                    var q3 = NormalizeAngleSigned(q3_eff - psi);
                    if (!WithinLimit(2, q3)) continue;

                    // Wrist orientation
                    var T01 = Math4.BuildLinkTransform(_mdh.Alpha[0], _mdh.A[0], off[0] + q1, _mdh.D[0]);
                    var T12 = Math4.BuildLinkTransform(_mdh.Alpha[1], _mdh.A[1], off[1] + q2, _mdh.D[1]);
                    var T23 = Math4.BuildLinkTransform(_mdh.Alpha[2], _mdh.A[2], off[2] + q3, _mdh.D[2]);
                    var T03 = Math4.MatMul(Math4.MatMul(T01, T12), T23);
                    var T34 = Math4.BuildLinkTransform(_mdh.Alpha[3], _mdh.A[3], off[3], _mdh.D[3]);
                    var T03q4zero = Math4.MatMul(T03, T34);
                    var T03_inv = Math4.InvertHomogeneous(T03q4zero);
                    var T36 = Math4.MatMul(T03_inv, T_target_flange_base);

                    var cos_q5 = T36[1, 2];
                    if (cos_q5 < -1 || cos_q5 > 1) continue;
                    foreach (var q5cand in new[] { Math.Acos(cos_q5), -Math.Acos(cos_q5) })
                    {
                        var q5 = NormalizeAngleSigned(q5cand);
                        var s5 = Math.Sin(q5);
                        double q4, q6;
                        if (Math.Abs(s5) > 1e-6)
                        {
                            q4 = NormalizeAngleSigned(Math.Atan2(T36[2, 2] / s5, T36[0, 2] / s5));
                            q6 = NormalizeAngleSigned(Math.Atan2(T36[1, 1] / s5, -T36[1, 0] / s5));
                        }
                        else { q4 = 0; q6 = NormalizeAngleSigned(Math.Atan2(T36[2, 1], T36[2, 0])); }
                        if (!WithinLimit(3, q4) || !WithinLimit(4, q5) || !WithinLimit(5, q6)) continue;
                        var cand = new[] { q1 - off[0], q2 - off[1], q3 - off[2], q4 - off[3], q5 - off[4], q6 - off[5] }
                                 .Select(NormalizeAngleSigned).ToArray();
                        if (!WithinLimits(cand)) continue;
                        if (IsDuplicate(solutions, cand, 1e-4)) continue;
                        var Tworld = ForwardMatrixWorld(cand);
                        if (!PoseMatch(Tworld, T_target_world, PositionTolerance, DirectionTolerance)) continue;
                        solutions.Add(cand);
                    }
                }
            }
            flags = Enumerable.Repeat(true, solutions.Count).ToArray();
            return solutions.ToArray();
        }

        // 数値IK（転置ヤコビアン法）
        public double[] InverseJacobian(double[,] T_target_world, double[] q_initial,
                                        out bool success, int maxIterations = 600,
                                        double tolerance = 1e-6, double alpha = 0.1, int verbose = 0)
        {
            var q = (double[])q_initial.Clone();
            var Tbase_inv = Math4.InvertHomogeneous(_mdh.T_base);
            var Ttool_inv = Math4.Identity();
            Ttool_inv[0, 3] = -_tool[0];
            Ttool_inv[2, 3] = -_tool[2];
            var dX = new double[6];

            for (var iter = 0; iter < maxIterations; iter++)
            {
                // 現在姿勢（world←tcp）と比較用のフランジ（base←flange）
                var Tworld = ForwardMatrixWorld(q);
                var Tflange_base = Math4.MatMul(Tbase_inv, Math4.MatMul(Tworld, Ttool_inv));
                var Tt_flange_base = Math4.MatMul(Tbase_inv, Math4.MatMul(T_target_world, Ttool_inv));

                // 位置誤差
                dX[0] = Tt_flange_base[0, 3] - Tflange_base[0, 3];
                dX[1] = Tt_flange_base[1, 3] - Tflange_base[1, 3];
                dX[2] = Tt_flange_base[2, 3] - Tflange_base[2, 3];

                // 姿勢誤差（行ベクトル：行の外積平均）
                double[] n_c = { Tflange_base[0, 0], Tflange_base[1, 0], Tflange_base[2, 0] };
                double[] o_c = { Tflange_base[0, 1], Tflange_base[1, 1], Tflange_base[2, 1] };
                double[] a_c = { Tflange_base[0, 2], Tflange_base[1, 2], Tflange_base[2, 2] };
                double[] n_t = { Tt_flange_base[0, 0], Tt_flange_base[1, 0], Tt_flange_base[2, 0] };
                double[] o_t = { Tt_flange_base[0, 1], Tt_flange_base[1, 1], Tt_flange_base[2, 1] };
                double[] a_t = { Tt_flange_base[0, 2], Tt_flange_base[1, 2], Tt_flange_base[2, 2] };

                var cn = Cross(n_c, n_t);
                var co = Cross(o_c, o_t);
                var ca = Cross(a_c, a_t);
                dX[3] = 0.5 * (cn[0] + co[0] + ca[0]);
                dX[4] = 0.5 * (cn[1] + co[1] + ca[1]);
                dX[5] = 0.5 * (cn[2] + co[2] + ca[2]);

                var errSq = dX[0] * dX[0] + dX[1] * dX[1] + dX[2] * dX[2] + dX[3] * dX[3] + dX[4] * dX[4] + dX[5] * dX[5];
                if (errSq < tolerance * tolerance)
                {
                    success = true;
                    return q;
                }

                var J = CalculateJacobianFlangeInBase(q);
                var JT = Math4.Transpose(J);
                var dq = MatVecMul(JT, dX); // 転置ヤコビアン

                for (var j = 0; j < 6; j++)
                {
                    q[j] = NormalizeAngleSigned(q[j] + alpha * dq[j]);
                    if (q[j] < _minAnglesRad[j]) q[j] = _minAnglesRad[j];
                    if (q[j] > _maxAnglesRad[j]) q[j] = _maxAnglesRad[j];
                }

                if (verbose > 1) Console.WriteLine($"Iter {iter}: err={Math.Sqrt(errSq):E6}");
            }

            success = false;
            return q;
        }

        // ===== ヤコビアン（base←flange） =====
        private double[,] CalculateJacobianFlangeInBase(double[] q)
        {
            var J = new double[6, 6];
            var T = Math4.Identity();
            var origins = new List<double[]> { new[] { 0.0, 0.0, 0.0 } };
            var zAxes = new List<double[]> { new[] { 0.0, 0.0, 1.0 } };

            for (var i = 0; i < 6; i++)
            {
                var Ti = Math4.BuildLinkTransform(_mdh.Alpha[i], _mdh.A[i], _mdh.Theta0[i] + q[i], _mdh.D[i]);
                T = Math4.MatMul(T, Ti);
                if (i == 2)
                {
                    var Te = Math4.BuildLinkTransform(_mdh.ElbowAlpha, _mdh.ElbowA, _mdh.ElbowTheta, _mdh.ElbowD);
                    T = Math4.MatMul(T, Te);
                }
                origins.Add([T[0, 3], T[1, 3], T[2, 3]]);
                zAxes.Add([T[0, 2], T[1, 2], T[2, 2]]);
            }

            var p_eff = origins[^1];
            for (var i = 0; i < 6; i++)
            {
                var zi = zAxes[i];
                var pi = origins[i];
                var dp = new[] { p_eff[0] - pi[0], p_eff[1] - pi[1], p_eff[2] - pi[2] };
                var Jv = Cross(zi, dp); // translational part

                J[0, i] = Jv[0]; J[1, i] = Jv[1]; J[2, i] = Jv[2];
                J[3, i] = zi[0]; J[4, i] = zi[1]; J[5, i] = zi[2];
            }
            return J;
        }

        // ===== ヘルパ群 =====
        private static double[] Cross(double[] a, double[] b)
        {
            return [a[1]*b[2]-a[2]*b[1],
                         a[2]*b[0]-a[0]*b[2],
                         a[0]*b[1]-a[1]*b[0]];
        }

        private static double[] MatVecMul(double[,] A, double[] x)
        {
            var y = new double[A.GetLength(0)];
            for (var i = 0; i < A.GetLength(0); i++)
            {
                double s = 0;
                for (var j = 0; j < A.GetLength(1); j++) s += A[i, j] * x[j];
                y[i] = s;
            }
            return y;
        }

        private static bool PoseMatch(double[,] A, double[,] B, double posTol, double dirTol)
        {
            var pa = new[] { A[0, 3], A[1, 3], A[2, 3] };
            var pb = new[] { B[0, 3], B[1, 3], B[2, 3] };
            var za = new[] { A[0, 2], A[1, 2], A[2, 2] };
            var zb = new[] { B[0, 2], B[1, 2], B[2, 2] };
            NormalizeInPlace(za); NormalizeInPlace(zb);
            return Norm(Sub(pa, pb)) < posTol && Norm(Sub(za, zb)) < dirTol;
        }

        private bool WithinLimit(int joint, double angle)
        {
            return angle >= _minAnglesRad[joint] - 1e-9 && angle <= _maxAnglesRad[joint] + 1e-9;
        }

        private bool WithinLimits(double[] q)
        {
            for (var i = 0; i < 6; i++) if (!WithinLimit(i, q[i])) return false;
            return true;
        }

        private static bool IsDuplicate(List<double[]> sols, double[] cand, double tol)
        {
            foreach (var s in sols)
            {
                var same = true;
                for (var i = 0; i < 6; i++)
                    if (Math.Abs(NormalizeAngleSigned(s[i] - cand[i])) > tol) { same = false; break; }
                if (same) return true;
            }
            return false;
        }

        private static double[] Sub(double[] a, double[] b) => new[] { a[0] - b[0], a[1] - b[1], a[2] - b[2] };
        private static double Norm(double[] v) => Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

        private static double NormalizeAngleSigned(double a)
        {
            while (a > Math.PI) a -= 2 * Math.PI;
            while (a < -Math.PI) a += 2 * Math.PI;
            return a;
        }

        private static void NormalizeInPlace(double[] v)
        {
            var n = Norm(v); if (n < 1e-15) { v[0] = v[1] = v[2] = 0; return; }
            v[0] /= n; v[1] /= n; v[2] /= n;
        }

        // Pw（手首中心）をΣ2へ写像
        private double[] PwToSigma2(double q1, double q2, double PwX, double PwY, double PwZ)
        {
            var T01 = Math4.BuildLinkTransform(_mdh.Alpha[0], _mdh.A[0], _mdh.Theta0[0] + q1, _mdh.D[0]);
            var T12 = Math4.BuildLinkTransform(_mdh.Alpha[1], _mdh.A[1], _mdh.Theta0[1] + q2, _mdh.D[1]);
            var T02 = Math4.MatMul(T01, T12);
            var T20 = Math4.InvertHomogeneous(T02);
            return Math4.MulPoint(T20, new[] { PwX, PwY, PwZ });
        }

        // 半径拘束から q2 候補を探索
        private List<double> SolveQ2ByRadius(double q1, double PwX, double PwY, double PwZ,
                                             double a3, double Lw, int samples, int verbose)
        {
            var roots = new List<double>();
            double minRes = double.PositiveInfinity, best = 0.0;
            double qmin = _minAnglesRad[1], qmax = _maxAnglesRad[1];
            for (var k = 0; k < samples; k++)
            {
                var q2 = qmin + (qmax - qmin) * k / (samples - 1);
                if (!WithinLimit(1, q2)) continue;

                var W2 = PwToSigma2(q1, q2, PwX, PwY, PwZ);
                double dx = W2[0] - a3, vy = W2[1];
                var res = Math.Abs(dx * dx + vy * vy - Lw * Lw);

                if (res < 1e-2) roots.Add(NormalizeAngleSigned(q2));
                if (res < minRes) { minRes = res; best = q2; }
            }
            if (roots.Count == 0 && minRes < 1e-1) roots.Add(NormalizeAngleSigned(best));
            if (verbose >= 3) Console.WriteLine($"   q2 roots: {roots.Count}, best residual={minRes:E2}");
            return roots;
        }
    }
}
