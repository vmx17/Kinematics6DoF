using System;
using System.Collections.Generic;
using System.Linq;

namespace KinematicsRM2
{
    #region Data Structures

    public sealed class MDHParameters
    {
        public double[] Alpha { get; }
        public double[] A { get; }
        public double[] D { get; }
        public double[] Offset { get; }
        public double[] MinAnglesRad { get; }
        public double[] MaxAnglesRad { get; }
        public double ElbowAlpha { get; }
        public double ElbowA { get; }
        public double ElbowD { get; }
        public double ElbowTheta { get; }

        public MDHParameters(
            double[] alpha,
            double[] a,
            double[] d,
            double[] offset,
            double[] minDeg,
            double[] maxDeg,
            double eAlpha, double eA, double eD, double eTheta)
        {
            Alpha = alpha; A = a; D = d; Offset = offset;
            ElbowAlpha = eAlpha; ElbowA = eA; ElbowD = eD; ElbowTheta = eTheta;
            MinAnglesRad = minDeg.Select(deg => deg * Math.PI / 180.0).ToArray();
            MaxAnglesRad = maxDeg.Select(deg => deg * Math.PI / 180.0).ToArray();
        }
    }

    public sealed class ToolSpecification
    {
        public double[] PositionLocal { get; }   // Translation from flange(frame6) to TCP in flange frame
        public double[] DirectionLocal { get; }  // Tool axis direction (currently only stored)
        public ToolSpecification(double[] positionLocal, double[] directionLocal)
        {
            if (positionLocal is not { Length: 3 }) throw new ArgumentException("positionLocal must be length 3");
            if (directionLocal is not { Length: 3 }) throw new ArgumentException("directionLocal must be length 3");
            PositionLocal = (double[])positionLocal.Clone();
            var n = Math.Sqrt(directionLocal[0]*directionLocal[0]+directionLocal[1]*directionLocal[1]+directionLocal[2]*directionLocal[2]);
            if (n < 1e-12) throw new ArgumentException("directionLocal magnitude must be >0");
            DirectionLocal = new[] { directionLocal[0] / n, directionLocal[1] / n, directionLocal[2] / n };
        }
    }

    #endregion

    #region Factory

    public static class ManipulatorFactory
    {
        private static readonly Dictionary<string, MDHParameters> _mdhTable = new(StringComparer.OrdinalIgnoreCase)
        {
            ["MiRobot"] = new MDHParameters(
                    alpha:  new[] { 0.0, Math.PI / 2, 0.0, -Math.PI / 2, -Math.PI / 2, 0.0 },
                    a:      new[] { 0.0, 29.69, 108.0, 168.98, 0.0, 0.0 },
                    d:      new[] { 127.0, 0.0, 0.0, 0.0, 0.0, 24.29 },
                    offset: new[] { 0.0, Math.PI / 2, -Math.PI / 2, -Math.PI, 0.0, 0.0 },
                    minDeg: new double[] { -180, -180, -90, -180, -180, -360 },
                    maxDeg: new double[] {  180,   90, 180,  180,   90,  360 },
                    eAlpha: Math.PI / 2, eA: 0.0, eD: -20.0, eTheta: 0.0
                )
        };

        private static readonly Dictionary<string, ToolSpecification> _toolTable = new(StringComparer.OrdinalIgnoreCase)
        {
            ["null"] = new ToolSpecification(new[]{0.0,0.0,0.0}, new[]{0.0,0.0,1.0})
        };

        public static Manipulator6DoF CreateManipulator(string? robotName = null, string? toolName = null)
        {
            var r = string.IsNullOrWhiteSpace(robotName) ? "MiRobot" : robotName;
            if (!_mdhTable.TryGetValue(r, out var mdh))
                throw new ArgumentException($"Unknown robot name: {r}");

            ToolSpecification tool;
            if (string.IsNullOrWhiteSpace(toolName) || !_toolTable.TryGetValue(toolName, out tool!))
                tool = _toolTable["null"];

            return new Manipulator6DoF(mdh, tool);
        }

        public static void RegisterTool(string name, double[] positionLocal, double[] directionLocal, bool overwrite = false)
        {
            if (_toolTable.ContainsKey(name) && !overwrite)
                throw new InvalidOperationException($"Tool '{name}' already exists.");
            _toolTable[name] = new ToolSpecification(positionLocal, directionLocal);
        }
    }

    #endregion

    #region Math Library

    internal static class MathUtil
    {
        public static double[,] DHTransform(double theta, double d, double a, double alpha)
        {
            var rX = RotX(alpha);
            var tX = TransX(a);
            var rZ = RotZ(theta);
            var tZ = TransZ(d);
            return MatMul(MatMul(MatMul(rX, tX), rZ), tZ);
        }
        public static double[,] RotX(double a)
        {
            var c = Math.Cos(a); var s = Math.Sin(a);
            var T = Identity();
            T[1, 1] = c; T[1, 2] = -s;
            T[2, 1] = s; T[2, 2] = c;
            return T;
        }
        public static double[,] RotZ(double a)
        {
            var c = Math.Cos(a); var s = Math.Sin(a);
            var T = Identity();
            T[0, 0] = c; T[0, 1] = -s;
            T[1, 0] = s; T[1, 1] = c;
            return T;
        }
        public static double[,] TransX(double a)
        {
            var T = Identity(); T[0, 3] = a; return T;
        }
        public static double[,] TransZ(double d)
        {
            var T = Identity(); T[2, 3] = d; return T;
        }
        public static double[,] MatMul(double[,] A, double[,] B)
        {
            var C = new double[4, 4];
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    for (int k = 0; k < 4; k++)
                        C[i, j] += A[i, k] * B[k, j];
            return C;
        }
        public static double[] MatVecMul(double[,] A, double[] v)
        {
            var r = new double[A.GetLength(0)];
            for (int i = 0; i < A.GetLength(0); i++)
                for (int j = 0; j < v.Length; j++)
                    r[i] += A[i, j] * v[j];
            return r;
        }
        public static double[,] Transpose(double[,] A)
        {
            var T = new double[A.GetLength(1), A.GetLength(0)];
            for (int i = 0; i < A.GetLength(0); i++)
                for (int j = 0; j < A.GetLength(1); j++)
                    T[j, i] = A[i, j];
            return T;
        }
        public static double[] Cross(double[] a, double[] b) =>
            new[] { a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] };
        public static double Norm(double[] v)
        {
            double s = 0; foreach (var x in v) s += x * x; return Math.Sqrt(s);
        }
        public static double[] Sub(double[] a, double[] b)
        {
            var r = new double[a.Length];
            for (int i = 0; i < a.Length; ++i) r[i] = a[i] - b[i];
            return r;
        }
        public static double[,] Identity() =>
            new double[4, 4] {
                    {1,0,0,0},
                    {0,1,0,0},
                    {0,0,1,0},
                    {0,0,0,1}
            };
        public static double[,] InvertHomogeneous(double[,] T)
        {
            var R = new double[3, 3];
            var p = new double[3];
            for (int i = 0; i < 3; i++)
            {
                p[i] = T[i, 3];
                for (int j = 0; j < 3; j++) R[i, j] = T[i, j];
            }
            var RT = new double[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    RT[i, j] = R[j, i];
            var negRTp = new double[3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    negRTp[i] -= RT[i, j] * p[j];
            var inv = Identity();
            for (int i = 0; i < 3; i++)
            {
                inv[i, 3] = negRTp[i];
                for (int j = 0; j < 3; j++)
                    inv[i, j] = RT[i, j];
            }
            return inv;
        }
        public static double NormalizeAngle(double a)
        {
            while (a > Math.PI) a -= 2 * Math.PI;
            while (a <= -Math.PI) a += 2 * Math.PI;
            return a;
        }
    }

    #endregion

    public sealed class Manipulator6DoF
    {
        private readonly MDHParameters _mdh;
        private readonly ToolSpecification _tool;

        public ToolSpecification Tool => _tool;

        public Manipulator6DoF(MDHParameters mdh, ToolSpecification tool)
        {
            _mdh = mdh;
            _tool = tool;
        }

        #region Forward Kinematics

        // verbose=0: none, 1: origins O0..O7, 2: + axes
        public double[,] Forward(double[] q, int verbose = 0)
        {
            var transforms = GetFrameTransforms(q); // Up to flange O6 (index last-1), O7 is TCP
            if (verbose > 0)
            {
                string[] names = { "O0","O1","O2","O3","Oe","O4","O5","O6","O7" };
                int count = Math.Min(names.Length, transforms.Count);
                Console.WriteLine();
                for (int i = 0; i < count; i++)
                {
                    var T = transforms[i];
                    Console.Write($"[{names[i]}] Pos: ({T[0, 3]:F3}, {T[1, 3]:F3}, {T[2, 3]:F3})");
                    if (verbose == 2)
                    {
                        Console.Write($"  X:({T[0, 0]:F3},{T[1, 0]:F3},{T[2, 0]:F3})");
                        Console.Write($"  Z:({T[0, 2]:F3},{T[1, 2]:F3},{T[2, 2]:F3})");
                    }
                    Console.WriteLine();
                }
            }
            return transforms[^1];
        }

        private List<double[,]> GetFrameTransforms(double[] q)
        {
            var list = new List<double[,]> { MathUtil.Identity() }; // O0
            var T = MathUtil.Identity();
            for (int i = 0; i < 6; i++)
            {
                T = MathUtil.MatMul(T, MathUtil.DHTransform(
                    q[i] + _mdh.Offset[i],
                    _mdh.D[i],
                    _mdh.A[i],
                    _mdh.Alpha[i]));
                list.Add(T); // joints origins O1..O3 then O4..O6
                if (i == 2)
                {
                    T = MathUtil.MatMul(T, MathUtil.DHTransform(
                        _mdh.ElbowTheta,
                        _mdh.ElbowD,
                        _mdh.ElbowA,
                        _mdh.ElbowAlpha));
                    list.Add(T); // Oe
                }
            }

            // Flange frame is last in list (O6). Add TCP (O7) using tool position (orientation same).
            var flange = list[^1];
            var tcp = ApplyTool(flange, _tool.PositionLocal);
            list.Add(tcp);
            return list;
        }

        private static double[,] ApplyTool(double[,] Tflange, double[] pLocal)
        {
            var Ttool = MathUtil.Identity();
            Ttool[0, 3] = pLocal[0];
            Ttool[1, 3] = pLocal[1];
            Ttool[2, 3] = pLocal[2];
            return MathUtil.MatMul(Tflange, Ttool);
        }

        #endregion

        #region Inverse Kinematics (Jacobian numerical)

        public double[]? InverseJacobian(
            double[,] T_target,              // Target TCP pose (includes tool)
            double[] q_initial,
            out bool success,
            int maxIter = 300,
            double tol = 1e-6,
            double alpha = 0.1,
            int verbose = 0)
        {
            if (verbose == 1)
            {
                // Extract flange and wrist center from target (remove tool then d6)
                ExtractFlangeAndPw(T_target, out var T_flange_target, out var Pw);
                Console.WriteLine($"[InverseJacobian] Pw: ({Pw[0]:F3}, {Pw[1]:F3}, {Pw[2]:F3})");
            }

            var q = (double[])q_initial.Clone();
            for (int iter = 0; iter < maxIter; iter++)
            {
                var T_cur = Forward(q);
                var err = PoseError(T_cur, T_target);
                if (MathUtil.Norm(err) < tol)
                {
                    success = true;
                    return q;
                }
                var J = CalculateJacobian(q);
                var JT = MathUtil.Transpose(J);
                var dq = MathUtil.MatVecMul(JT, err);
                for (int i = 0; i < 6; i++)
                {
                    q[i] = MathUtil.NormalizeAngle(q[i] + alpha * dq[i]);
                }
                ClampToLimits(q);
            }
            success = false;
            return null;
        }

        private double[,] CalculateJacobian(double[] q)
        {
            var J = new double[6, 6];
            var transforms = GetFrameTransforms(q);
            var p_eff = new[] { transforms[^1][0, 3], transforms[^1][1, 3], transforms[^1][2, 3] }; // TCP
            for (int i = 0; i < 6; i++)
            {
                int frame_idx = i < 4 ? i : i + 1; // skip elbow frame
                var T_i = transforms[frame_idx];
                var z = new[] { T_i[0, 2], T_i[1, 2], T_i[2, 2] };
                var o = new[] { T_i[0, 3], T_i[1, 3], T_i[2, 3] };
                var Jv = MathUtil.Cross(z, MathUtil.Sub(p_eff, o));
                for (int r = 0; r < 3; r++)
                {
                    J[r, i] = Jv[r];
                    J[r + 3, i] = z[r];
                }
            }
            return J;
        }

        private double[] PoseError(double[,] T_cur, double[,] T_target)
        {
            var err = new double[6];
            for (int i = 0; i < 3; i++) err[i] = T_target[i, 3] - T_cur[i, 3];
            // orientation error (average cross of columns)
            for (int axis = 0; axis < 3; axis++)
            {
                var rc = new[] { T_cur[0, axis], T_cur[1, axis], T_cur[2, axis] };
                var rt = new[] { T_target[0, axis], T_target[1, axis], T_target[2, axis] };
                var c = MathUtil.Cross(rc, rt);
                err[3] += c[0]; err[4] += c[1]; err[5] += c[2];
            }
            err[3] /= 3.0; err[4] /= 3.0; err[5] /= 3.0;
            return err;
        }

        #endregion

        #region Inverse Kinematics (Geometric hybrid)

        public double[][] InverseGeometric(double[,] T_target, int verbose = 0)
        {
            // Extract flange pose and wrist center from TCP target
            ExtractFlangeAndPw(T_target, out var T_flange_target, out var Pw);
            if (verbose == 1)
                Console.WriteLine($"[InverseGeometric] Pw: ({Pw[0]:F3}, {Pw[1]:F3}, {Pw[2]:F3})");

            // Use flange target orientation for orientation solving (identical to TCP orientation here)
            var R_flange = new double[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    R_flange[i, j] = T_flange_target[i, j];

            var solutions = new List<double[]>();

            // Shoulder branches
            var q1cand = new List<double>
                {
                    Math.Atan2(Pw[1], Pw[0]),
                    MathUtil.NormalizeAngle(Math.Atan2(Pw[1], Pw[0]) + Math.PI)
                };

            foreach (var q1_raw in q1cand)
            {
                double q1 = q1_raw - _mdh.Offset[0];
                if (!WithinLimit(0, q1)) continue;

                var q23Candidates = SolveQ2Q3(q1_raw, Pw);
                foreach (var (q2_raw, q3_raw) in q23Candidates)
                {
                    double q2 = q2_raw - _mdh.Offset[1];
                    double q3 = q3_raw - _mdh.Offset[2];
                    if (!WithinLimit(1, q2) || !WithinLimit(2, q3)) continue;

                    // Solve wrist orientation using TCP target (orientation identical to flange)
                    var wristSols = SolveWristAngles(q1_raw, q2_raw, q3_raw, T_flange_target);
                    foreach (var w in wristSols)
                    {
                        double q4_raw = w[0];
                        double q5_raw = w[1];
                        double q6_raw = w[2];
                        double q4 = q4_raw - _mdh.Offset[3];
                        double q5 = q5_raw - _mdh.Offset[4];
                        double q6 = q6_raw - _mdh.Offset[5];
                        if (!WithinLimit(3, q4) || !WithinLimit(4, q5) || !WithinLimit(5, q6))
                            continue;

                        var full = new[]
                            {
                                MathUtil.NormalizeAngle(q1),
                                MathUtil.NormalizeAngle(q2),
                                MathUtil.NormalizeAngle(q3),
                                MathUtil.NormalizeAngle(q4),
                                MathUtil.NormalizeAngle(q5),
                                MathUtil.NormalizeAngle(q6)
                            };

                        var Tchk = Forward(full);
                        if (PoseClose(Tchk, T_target, 1e-3, 2e-3) && !Duplicate(solutions, full, 1e-4))
                            solutions.Add(full);
                    }
                }
            }

            return solutions.ToArray();
        }

        private void ExtractFlangeAndPw(double[,] T_tcp, out double[,] T_flange, out double[] Pw)
        {
            // T_tcp = T_flange * Trans(toolPos)
            var R_tcp = new double[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    R_tcp[i, j] = T_tcp[i, j];
            var P_tcp = new[] { T_tcp[0, 3], T_tcp[1, 3], T_tcp[2, 3] };

            // Flange position = TCP - R * toolPositionLocal
            var toolPos = _tool.PositionLocal;
            var R_toolPos = new[] {
                    R_tcp[0,0]*toolPos[0] + R_tcp[0,1]*toolPos[1] + R_tcp[0,2]*toolPos[2],
                    R_tcp[1,0]*toolPos[0] + R_tcp[1,1]*toolPos[1] + R_tcp[1,2]*toolPos[2],
                    R_tcp[2,0]*toolPos[0] + R_tcp[2,1]*toolPos[1] + R_tcp[2,2]*toolPos[2]
                };
            var P_flange = new[] { P_tcp[0]-R_toolPos[0], P_tcp[1]-R_toolPos[1], P_tcp[2]-R_toolPos[2] };

            T_flange = MathUtil.Identity();
            for (int i = 0; i < 3; i++)
            {
                T_flange[i, 0] = R_tcp[i, 0];
                T_flange[i, 1] = R_tcp[i, 1];
                T_flange[i, 2] = R_tcp[i, 2];
                T_flange[i, 3] = P_flange[i];
            }

            // Wrist center Pw = flange - R * (0,0,d6)
            var d6 = _mdh.D[5];
            Pw = new[] {
                    P_flange[0] - R_tcp[0,2]*d6,
                    P_flange[1] - R_tcp[1,2]*d6,
                    P_flange[2] - R_tcp[2,2]*d6
                };
        }

        #endregion

        #region Geometric IK Helpers (unchanged core logic except tool handling)

        private List<(double, double)> SolveQ2Q3(double q1_raw, double[] Pw)
        {
            var results = new List<(double, double)>();
            int samplesQ2 = 40, samplesQ3 = 40;
            double q2Min = _mdh.MinAnglesRad[1] + _mdh.Offset[1];
            double q2Max = _mdh.MaxAnglesRad[1] + _mdh.Offset[1];
            double q3Min = _mdh.MinAnglesRad[2] + _mdh.Offset[2];
            double q3Max = _mdh.MaxAnglesRad[2] + _mdh.Offset[2];

            var seedList = new List<(double q2, double q3, double err)>();
            for (int i = 0; i <= samplesQ2; i++)
            {
                double q2 = q2Min + (q2Max - q2Min) * i / samplesQ2;
                for (int j = 0; j <= samplesQ3; j++)
                {
                    double q3 = q3Min + (q3Max - q3Min) * j / samplesQ3;
                    var o4 = ComputeO4Position(q1_raw, q2, q3);
                    double err = Dist3(o4, Pw);
                    if (err < 20.0) seedList.Add((q2, q3, err));
                }
            }

            seedList.Sort((a, b) => a.err.CompareTo(b.err));
            int keep = Math.Min(40, seedList.Count);
            for (int k = 0; k < keep; k++)
            {
                var (q2s, q3s, _) = seedList[k];
                var refined = RefineQ2Q3(q1_raw, q2s, q3s, Pw);
                var o4r = ComputeO4Position(q1_raw, refined.q2, refined.q3);
                double err = Dist3(o4r, Pw);
                if (err < 1e-2)
                {
                    bool dup = false;
                    foreach (var ex in results)
                        if (Math.Abs(MathUtil.NormalizeAngle(ex.Item1 - refined.q2)) < 1e-3 &&
                            Math.Abs(MathUtil.NormalizeAngle(ex.Item2 - refined.q3)) < 1e-3)
                        { dup = true; break; }
                    if (!dup)
                        results.Add((MathUtil.NormalizeAngle(refined.q2), MathUtil.NormalizeAngle(refined.q3)));
                }
            }
            return results;
        }

        private (double q2, double q3) RefineQ2Q3(double q1_raw, double q2Init, double q3Init, double[] Pw)
        {
            double q2 = q2Init, q3 = q3Init, eps = 1e-5;
            for (int iter = 0; iter < 50; iter++)
            {
                var f = Sub3(ComputeO4Position(q1_raw, q2, q3), Pw);
                double err = Norm3(f);
                if (err < 1e-3) break;

                var J = new double[3, 2];
                double q2p = q2 + eps; var fp = Sub3(ComputeO4Position(q1_raw, q2p, q3), Pw);
                double q2m = q2 - eps; var fm = Sub3(ComputeO4Position(q1_raw, q2m, q3), Pw);
                for (int i = 0; i < 3; i++) J[i, 0] = (fp[i] - fm[i]) / (2 * eps);

                double q3p = q3 + eps; fp = Sub3(ComputeO4Position(q1_raw, q2, q3p), Pw);
                double q3m = q3 - eps; fm = Sub3(ComputeO4Position(q1_raw, q2, q3m), Pw);
                for (int i = 0; i < 3; i++) J[i, 1] = (fp[i] - fm[i]) / (2 * eps);

                double JTJ00=0, JTJ01=0, JTJ11=0, JTf0=0, JTf1=0;
                for (int i = 0; i < 3; i++)
                {
                    var Ji0 = J[i,0]; var Ji1 = J[i,1];
                    JTJ00 += Ji0 * Ji0; JTJ01 += Ji0 * Ji1; JTJ11 += Ji1 * Ji1;
                    JTf0 += Ji0 * f[i]; JTf1 += Ji1 * f[i];
                }
                double lambda = 1e-6;
                JTJ00 += lambda; JTJ11 += lambda;
                double det = JTJ00*JTJ11 - JTJ01*JTJ01;
                if (Math.Abs(det) < 1e-12) break;
                double inv00 = JTJ11/det;
                double inv01 = -JTJ01/det;
                double inv11 = JTJ00/det;
                double dq2 = -(inv00*JTf0 + inv01*JTf1);
                double dq3 = -(inv01*JTf0 + inv11*JTf1);

                q2 = MathUtil.NormalizeAngle(q2 + dq2);
                q3 = MathUtil.NormalizeAngle(q3 + dq3);

                q2 = Math.Max(_mdh.MinAnglesRad[1] + _mdh.Offset[1], Math.Min(_mdh.MaxAnglesRad[1] + _mdh.Offset[1], q2));
                q3 = Math.Max(_mdh.MinAnglesRad[2] + _mdh.Offset[2], Math.Min(_mdh.MaxAnglesRad[2] + _mdh.Offset[2], q3));
            }
            return (q2, q3);
        }

        private List<double[]> SolveWristAngles(double q1_raw, double q2_raw, double q3_raw, double[,] T_flange_target)
        {
            var results = new List<double[]>();
            var T03e = GetT03Elbow(new[] { q1_raw, q2_raw, q3_raw });
            var pre4 = MathUtil.MatMul(T03e, MathUtil.DHTransform(0.0, 0.0, _mdh.A[3], _mdh.Alpha[3]));
            var TpreInv = MathUtil.InvertHomogeneous(pre4);
            var Tgoal = MathUtil.MatMul(TpreInv, T_flange_target);

            double[] q4Seeds = { -Math.PI, -Math.PI/2, 0, Math.PI/2, Math.PI };
            double[] q5Seeds = { -Math.PI/2, 0, Math.PI/2 };
            double[] q6Seeds = { -Math.PI, -Math.PI/2, 0, Math.PI/2, Math.PI };

            foreach (var s4 in q4Seeds)
                foreach (var s5 in q5Seeds)
                    foreach (var s6 in q6Seeds)
                    {
                        var sol = RefineWrist(Tgoal, s4, s5, s6);
                        if (sol != null)
                        {
                            var q4r = MathUtil.NormalizeAngle(sol[0]);
                            var q5r = MathUtil.NormalizeAngle(sol[1]);
                            var q6r = MathUtil.NormalizeAngle(sol[2]);
                            bool dup = false;
                            foreach (var w in results)
                                if (Math.Abs(MathUtil.NormalizeAngle(w[0] - q4r)) < 1e-3 &&
                                    Math.Abs(MathUtil.NormalizeAngle(w[1] - q5r)) < 1e-3 &&
                                    Math.Abs(MathUtil.NormalizeAngle(w[2] - q6r)) < 1e-3)
                                { dup = true; break; }
                            if (!dup) results.Add(new[] { q4r + _mdh.Offset[3], q5r + _mdh.Offset[4], q6r + _mdh.Offset[5] });
                        }
                    }
            return results;
        }

        private double[]? RefineWrist(double[,] Tgoal, double q4Init, double q5Init, double q6Init)
        {
            double q4 = q4Init, q5 = q5Init, q6 = q6Init;
            double eps = 1e-5;
            for (int iter = 0; iter < 60; iter++)
            {
                var Twrist = WristForward(q4, q5, q6);
                var err = WristOrientError(Twrist, Tgoal);
                double n = Norm3(err);
                if (n < 5e-3) break;

                var J = new double[3,3];
                var errp = WristOrientError(WristForward(q4+eps,q5,q6), Tgoal);
                for (int i = 0; i < 3; i++) J[i, 0] = (errp[i] - err[i]) / eps;
                errp = WristOrientError(WristForward(q4, q5 + eps, q6), Tgoal);
                for (int i = 0; i < 3; i++) J[i, 1] = (errp[i] - err[i]) / eps;
                errp = WristOrientError(WristForward(q4, q5, q6 + eps), Tgoal);
                for (int i = 0; i < 3; i++) J[i, 2] = (errp[i] - err[i]) / eps;

                double[,] JTJ = new double[3,3];
                double[] JTe = new double[3];
                for (int r = 0; r < 3; r++)
                {
                    for (int c = 0; c < 3; c++)
                        for (int k = 0; k < 3; k++)
                            JTJ[r, c] += J[k, r] * J[k, c];
                    for (int k = 0; k < 3; k++) JTe[r] += J[k, r] * err[k];
                }
                double lambda = 1e-5;
                JTJ[0, 0] += lambda; JTJ[1, 1] += lambda; JTJ[2, 2] += lambda;
                var dq = Solve3x3(JTJ, Negate(JTe));
                if (dq == null) break;

                double scale=1.0;
                for (int ls = 0; ls < 5; ls++)
                {
                    var q4t = MathUtil.NormalizeAngle(q4 + scale*dq[0]);
                    var q5t = MathUtil.NormalizeAngle(q5 + scale*dq[1]);
                    var q6t = MathUtil.NormalizeAngle(q6 + scale*dq[2]);
                    var errTest = WristOrientError(WristForward(q4t,q5t,q6t), Tgoal);
                    if (Norm3(errTest) < n)
                    {
                        q4 = q4t; q5 = q5t; q6 = q6t;
                        break;
                    }
                    scale *= 0.5;
                }
            }
            return new[] { q4, q5, q6 };
        }

        private double[,] WristForward(double q4, double q5, double q6)
        {
            var T = MathUtil.Identity();
            T = MathUtil.MatMul(T, MathUtil.DHTransform(q4 + _mdh.Offset[3], 0, 0, 0));
            T = MathUtil.MatMul(T, MathUtil.DHTransform(q5 + _mdh.Offset[4], 0, 0, _mdh.Alpha[4]));
            T = MathUtil.MatMul(T, MathUtil.DHTransform(q6 + _mdh.Offset[5], _mdh.D[5], 0, _mdh.Alpha[5]));
            return T;
        }

        private double[] WristOrientError(double[,] Twrist, double[,] Tgoal)
        {
            var err = new double[3];
            for (int axis = 0; axis < 3; axis++)
            {
                var rc = new[] { Twrist[0,axis], Twrist[1,axis], Twrist[2,axis] };
                var rt = new[] { Tgoal[0,axis], Tgoal[1,axis], Tgoal[2,axis] };
                var c = MathUtil.Cross(rc, rt);
                err[0] += c[0]; err[1] += c[1]; err[2] += c[2];
            }
            err[0] /= 3.0; err[1] /= 3.0; err[2] /= 3.0;
            return err;
        }

        #endregion

        #region Shared Helpers

        private static double[] Sub3(double[] a, double[] b) => new[] { a[0] - b[0], a[1] - b[1], a[2] - b[2] };
        private static double Dist3(double[] a, double[] b) =>
            Math.Sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
        private static double Norm3(double[] v) => Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

        private double[] ComputeO4Position(double q1_raw, double q2_raw, double q3_raw)
        {
            var T03e = GetT03Elbow(new[]{ q1_raw, q2_raw, q3_raw });
            var T = MathUtil.MatMul(T03e, MathUtil.DHTransform(0.0, 0.0, _mdh.A[3], _mdh.Alpha[3]));
            return new[] { T[0, 3], T[1, 3], T[2, 3] };
        }

        private double[,] GetT03Elbow(double[] q_raw)
        {
            var T = MathUtil.Identity();
            for (int i = 0; i < 3; i++)
                T = MathUtil.MatMul(T, MathUtil.DHTransform(
                    q_raw[i] + _mdh.Offset[i],
                    _mdh.D[i],
                    _mdh.A[i],
                    _mdh.Alpha[i]));
            T = MathUtil.MatMul(T, MathUtil.DHTransform(
                _mdh.ElbowTheta,
                _mdh.ElbowD,
                _mdh.ElbowA,
                _mdh.ElbowAlpha));
            return T;
        }

        private static double[] MatVecMul3x3(double[,] R, double[] v)
        {
            var r = new double[3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    r[i] += R[i, j] * v[j];
            return r;
        }

        private static double[] Negate(double[] v)
        {
            var r = new double[v.Length];
            for (int i = 0; i < v.Length; i++) r[i] = -v[i];
            return r;
        }

        private static double[]? Solve3x3(double[,] A, double[] b)
        {
            double det =
                    A[0,0]*(A[1,1]*A[2,2]-A[1,2]*A[2,1]) -
                    A[0,1]*(A[1,0]*A[2,2]-A[1,2]*A[2,0]) +
                    A[0,2]*(A[1,0]*A[2,1]-A[1,1]*A[2,0]);
            if (Math.Abs(det) < 1e-14) return null;
            double invDet = 1.0/det;
            var inv = new double[3,3];
            inv[0, 0] = (A[1, 1] * A[2, 2] - A[1, 2] * A[2, 1]) * invDet;
            inv[0, 1] = (A[0, 2] * A[2, 1] - A[0, 1] * A[2, 2]) * invDet;
            inv[0, 2] = (A[0, 1] * A[1, 2] - A[0, 2] * A[1, 1]) * invDet;
            inv[1, 0] = (A[1, 2] * A[2, 0] - A[1, 0] * A[2, 2]) * invDet;
            inv[1, 1] = (A[0, 0] * A[2, 2] - A[0, 2] * A[2, 0]) * invDet;
            inv[1, 2] = (A[0, 2] * A[1, 0] - A[0, 0] * A[1, 2]) * invDet;
            inv[2, 0] = (A[1, 0] * A[2, 1] - A[1, 1] * A[2, 0]) * invDet;
            inv[2, 1] = (A[0, 1] * A[2, 0] - A[0, 0] * A[2, 1]) * invDet;
            inv[2, 2] = (A[0, 0] * A[1, 1] - A[0, 1] * A[1, 0]) * invDet;
            var x = new double[3];
            for (int i = 0; i < 3; i++)
                for (int k = 0; k < 3; k++)
                    x[i] += inv[i, k] * b[k];
            return x;
        }

        private bool PoseClose(double[,] A, double[,] B, double posTol, double oriTol)
        {
            double dp=0;
            for (int i = 0; i < 3; i++) { var d=A[i,3]-B[i,3]; dp += d * d; }
            if (Math.Sqrt(dp) > posTol) return false;
            double[] acc={0,0,0};
            for (int axis = 0; axis < 3; axis++)
            {
                var ac = new[]{ A[0,axis], A[1,axis], A[2,axis] };
                var bc = new[]{ B[0,axis], B[1,axis], B[2,axis] };
                var cr = MathUtil.Cross(ac, bc);
                for (int i = 0; i < 3; i++) acc[i] += cr[i];
            }
            for (int i = 0; i < 3; i++) acc[i] /= 3.0;
            return Norm3(acc) < oriTol;
        }

        private static bool Duplicate(List<double[]> sols, double[] cand, double tol)
        {
            foreach (var s in sols)
            {
                double maxd=0;
                for (int i = 0; i < s.Length; i++)
                {
                    var d = Math.Abs(MathUtil.NormalizeAngle(s[i]-cand[i]));
                    if (d > maxd) maxd = d;
                    if (maxd > tol) break;
                }
                if (maxd <= tol) return true;
            }
            return false;
        }

        private bool WithinLimit(int idx, double val) =>
            val >= _mdh.MinAnglesRad[idx] && val <= _mdh.MaxAnglesRad[idx];

        private void ClampToLimits(double[] q)
        {
            for (int i = 0; i < 6; i++)
            {
                if (q[i] < _mdh.MinAnglesRad[i]) q[i] = _mdh.MinAnglesRad[i];
                if (q[i] > _mdh.MaxAnglesRad[i]) q[i] = _mdh.MaxAnglesRad[i];
            }
        }

        #endregion
    }
}