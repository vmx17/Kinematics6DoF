using System;
using System.Collections.Generic;
using System.Linq;
using RowMeasureUtility;
using static RowMeasureUtility.MathUtil;

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
        public double[] PositionLocal { get; }
        public double[] Rxyz { get; }
        public double[,] Rotation { get; }

        public ToolSpecification(double[] positionLocal, double[] rxyz)
        {
            if (positionLocal is not { Length: 3 }) throw new ArgumentException("positionLocal must be length 3");
            if (rxyz is not { Length: 3 }) throw new ArgumentException("rxyz must be length 3");
            PositionLocal = (double[])positionLocal.Clone();
            Rxyz = (double[])rxyz.Clone();
            Rotation = BuildRotationRzRyRx(Rxyz[0], Rxyz[1], Rxyz[2]);
        }

        public ToolSpecification(double[] positionLocal, double[] directionLocal, double[]? rxyz = null)
            : this(positionLocal, rxyz ?? directionLocal) { }

        private static double[,] BuildRotationRzRyRx(double rx, double ry, double rz)
        {
            double cz = Math.Cos(rz), sz = Math.Sin(rz);
            double cy = Math.Cos(ry), sy = Math.Sin(ry);
            double cx = Math.Cos(rx), sx = Math.Sin(rx);

            var r00 = cz * cy; var r01 = cz * sy * sx - sz * cx; var r02 = cz * sy * cx + sz * sx;
            var r10 = sz * cy; var r11 = sz * sy * sx + cz * cx; var r12 = sz * sy * cx - cz * sx;
            var r20 = -sy; var r21 = cy * sx; var r22 = cy * cx;

            var R = MathUtil.Identity();
            R[0, 0] = r00; R[0, 1] = r01; R[0, 2] = r02;
            R[1, 0] = r10; R[1, 1] = r11; R[1, 2] = r12;
            R[2, 0] = r20; R[2, 1] = r21; R[2, 2] = r22;
            return R;
        }
    }

    #endregion

    #region Factory

    public static class ManipulatorFactory
    {
        private static readonly Dictionary<string, MDHParameters> _mdhTable = new(StringComparer.OrdinalIgnoreCase)
        {
            ["MiRobot"] = new MDHParameters(
                    alpha:  [0.0, Math.PI / 2, 0.0, -Math.PI / 2, -Math.PI / 2, 0.0],
                    a:      [0.0, 29.69, 108.0, 168.98, 0.0, 0.0],
                    d:      [127.0, 0.0, 0.0, 0.0, 0.0, 24.29],
                    offset: [0.0, Math.PI / 2, -Math.PI / 2, -Math.PI, 0.0, 0.0],
                    minDeg: new double[] { -180, -180, -90, -180, -180, -360 },
                    maxDeg: new double[] {  180,   90, 180,  180,   90,  360 },
                    eAlpha: Math.PI / 2, eA: 0.0, eD: -20.0, eTheta: 0.0
                )
        };

        private static readonly Dictionary<string, ToolSpecification> _toolTable = new(StringComparer.OrdinalIgnoreCase)
        {
            ["null"]  = new ToolSpecification([0.0,0.0,0.0],[0.0,0.0,0.0]),
            ["TOOL0"] = new ToolSpecification([0.0,0.0,1.0],[0.0,0.0,0.0]),
            ["TOOL1"] = new ToolSpecification([0.707106781187,0.0,1.707106781187],[0.0,45.0,0.0])
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

        public static void RegisterTool(string name, double[] positionLocal, double[] rxyz, bool overwrite = false)
        {
            if (_toolTable.ContainsKey(name) && !overwrite)
                throw new InvalidOperationException($"Tool '{name}' already exists.");
            _toolTable[name] = new ToolSpecification(positionLocal, rxyz);
        }
    }

    #endregion

    public sealed class Manipulator6DoF
    {
        private readonly MDHParameters _mdh;
        private readonly ToolSpecification _tool;

        // Flip angle threshold (deg) - solutions with orientation diff >= this are considered flipped.
        public double OrientationFlipRejectDeg { get; set; } = 170.0;

        // Added for ranking: axis alignment tolerance (deg) for classifying pure approach-axis flips.
        public double FlipAxisAlignToleranceDeg { get; set; } = 5.0;

        public ToolSpecification Tool => _tool;

        public Manipulator6DoF(MDHParameters mdh, ToolSpecification tool)
        {
            _mdh = mdh;
            _tool = tool;
        }

        #region Forward Kinematics

        public double[,] Forward(double[] q, int verbose = 0)
        {
            var transforms = GetFrameTransforms(q); // includes TCP
            if (verbose > 0)
            {
                string[] names = { "O0","O1","O2","O3","Oe","O4","O5","O6","O7" };
                var count = Math.Min(names.Length, transforms.Count);
                Console.WriteLine();
                for (var i = 0; i < count; i++)
                {
                    var T = transforms[i];
                    Console.Write($"[{names[i]}] Pos: ({T[0, 3]:F3}, {T[1, 3]:F3}, {T[2, 3]:F3})");
                    if (verbose == 2 && i < transforms.Count)
                    {
                        Console.Write($"  X:({T[0, 0]:F3},{T[1, 0]:F3},{T[2, 0]:F3})");
                        Console.Write($"  Z:({T[0, 2]:F3},{T[1, 2]:F3},{T[2, 2]:F3})");
                    }
                    Console.WriteLine();
                }

                if (transforms.Count > 0)
                {
                    var Ttcp = transforms[^1];
                    Console.WriteLine("[Tool Matrix] (O7 homogeneous transform):");
                    for (var r = 0; r < 4; r++)
                        Console.WriteLine($"  {Ttcp[r, 0]:F6}  {Ttcp[r, 1]:F6}  {Ttcp[r, 2]:F6}  {Ttcp[r, 3]:F3}");
                }
            }
            return transforms[^1];
        }

        private List<double[,]> GetFrameTransforms(double[] q)
        {
            var list = new List<double[,]> { MathUtil.Identity() }; // O0
            var T = MathUtil.Identity();
            for (var i = 0; i < 6; i++)
            {
                T = MathUtil.MatMul(T, MathUtil.DHTransform(
                    q[i] + _mdh.Offset[i],
                    _mdh.D[i],
                    _mdh.A[i],
                    _mdh.Alpha[i]));
                list.Add(T);
                if (i == 2)
                {
                    T = MathUtil.MatMul(T, MathUtil.DHTransform(
                        _mdh.ElbowTheta,
                        _mdh.ElbowD,
                        _mdh.ElbowA,
                        _mdh.ElbowAlpha));
                    list.Add(T);
                }
            }
            var flange = list[^1];
            var tcp = ApplyTool(flange, _tool);
            list.Add(tcp);
            return list;
        }

        private static double[,] ApplyTool(double[,] Tflange, ToolSpecification tool)
        {
            var Ttool = MathUtil.Identity();
            for (var r = 0; r < 3; r++)
            {
                Ttool[r, 0] = tool.Rotation[r, 0];
                Ttool[r, 1] = tool.Rotation[r, 1];
                Ttool[r, 2] = tool.Rotation[r, 2];
            }
            Ttool[0, 3] = tool.PositionLocal[0];
            Ttool[1, 3] = tool.PositionLocal[1];
            Ttool[2, 3] = tool.PositionLocal[2];
            return MathUtil.MatMul(Tflange, Ttool);
        }

        #endregion

        #region Inverse Kinematics (Re-implementation)

        public double[]? InverseJacobian(
            double[,] T_target,
            double[] q_initial,
            out bool success,
            int maxIterations = 100,
            double tolerance = 1e-5,
            double alpha = 0.5,      // Step size
            double damping = 0.02,     // Damping factor for DLS
            int verbose = 0)
        {
            var q = (double[])q_initial.Clone();

            for (var iter = 0; iter < maxIterations; iter++)
            {
                // 1. Calculate current pose with Forward Kinematics
                var T_cur = Forward(q);

                // 2. Calculate the 6D error vector (position and orientation)
                var err = CalculatePoseError(T_cur, T_target);
                var errNorm = MathUtil.Norm(err);

                if (verbose > 0)
                    Console.WriteLine($" iter {iter} err={errNorm:E4}");

                // 3. Check for convergence
                if (errNorm < tolerance)
                {
                    success = true;
                    if (verbose > 0) Console.WriteLine($"[InverseJacobian] Converged at iteration {iter} with error {errNorm:E4}");
                    return q;
                }

                // 4. Calculate the Geometric Jacobian
                var J = CalculateGeometricJacobian(q);

                // 5. Solve for the change in joint angles (dq) using Damped Least Squares (DLS)
                // (J * J^T + damping^2 * I) * y = err
                // dq = J^T * y
                var JJt = MathUtil.MatMul(J, MathUtil.Transpose(J));
                for (int i = 0; i < 6; i++) JJt[i, i] += damping * damping;

                var y = SolveByCholesky(JJt, err);
                if (y == null)
                {
                    if (verbose > 0) Console.WriteLine("Solver failed (matrix may be non-positive definite). Aborting.");
                    break; // Solver failed, exit loop
                }

                var dq = MathUtil.MatVecMul(MathUtil.Transpose(J), y);

                // 6. Update joint angles
                for (var i = 0; i < 6; i++)
                {
                    q[i] += alpha * dq[i];
                }

                // 7. Enforce joint limits
                ClampToLimits(q);
            }

            success = false;
            return null;
        }

        private double[,] CalculateGeometricJacobian(double[] q)
        {
            int[] axisFrameIdx = { 0, 1, 2, 3, 5, 6 };
            var J = new double[6, 6];
            var transforms = GetFrameTransforms(q);
            var p_eff = new[] { transforms[^1][0, 3], transforms[^1][1, 3], transforms[^1][2, 3] };

            for (var joint = 0; joint < 6; joint++)
            {
                var T_prev = transforms[axisFrameIdx[joint]];
                var z_axis = new[] { T_prev[0, 2], T_prev[1, 2], T_prev[2, 2] };
                var o_origin = new[] { T_prev[0, 3], T_prev[1, 3], T_prev[2, 3] };
                
                var J_linear = MathUtil.Cross(z_axis, MathUtil.Sub(p_eff, o_origin));
                var J_angular = z_axis;

                for (var r = 0; r < 3; r++)
                {
                    J[r, joint] = J_linear[r];
                    J[r + 3, joint] = J_angular[r];
                }
            }
            return J;
        }

        private double[] CalculatePoseError(double[,] T_cur, double[,] T_target)
        {
            var err = new double[6];
            
            // Position error (world frame)
            for (var i = 0; i < 3; i++)
                err[i] = T_target[i, 3] - T_cur[i, 3];

            // Orientation error (world frame)
            var R_err = new double[3, 3];
            for (var i = 0; i < 3; i++)
                for (var j = 0; j < 3; j++)
                    for (var k = 0; k < 3; k++)
                        R_err[i, j] += T_target[i, k] * T_cur[j, k]; // R_target * R_current^T

            // Convert rotation error matrix to axis-angle vector
            var trace = R_err[0, 0] + R_err[1, 1] + R_err[2, 2];
            var angle = Math.Acos(Math.Clamp((trace - 1.0) / 2.0, -1.0, 1.0));

            if (Math.Abs(angle) < 1e-9)
            {
                // No orientation error
                err[3] = 0; err[4] = 0; err[5] = 0;
            }
            else
            {
                var s = 2 * Math.Sin(angle);
                var ax = (R_err[2, 1] - R_err[1, 2]) / s;
                var ay = (R_err[0, 2] - R_err[2, 0]) / s;
                var az = (R_err[1, 0] - R_err[0, 1]) / s;
                err[3] = ax * angle;
                err[4] = ay * angle;
                err[5] = az * angle;
            }
            return err;
        }

        private static double[]? SolveByCholesky(double[,] A, double[] b)
        {
            const int n = 6;
            var L = new double[n, n];

            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j <= i; j++)
                {
                    double sum = 0;
                    for (int k = 0; k < j; k++)
                    {
                        sum += L[i, k] * L[j, k];
                    }

                    if (i == j)
                    {
                        double d = A[i, i] - sum;
                        if (d <= 1e-12) return null; 
                        L[i, i] = Math.Sqrt(d);
                    }
                    else
                    {
                        if (Math.Abs(L[j, j]) < 1e-12) return null; 
                        L[i, j] = (A[i, j] - sum) / L[j, j];
                    }
                }
            }

            var y = new double[n];
            for (int i = 0; i < n; i++)
            {
                double sum = 0;
                for (int k = 0; k < i; k++)
                {
                    sum += L[i, k] * y[k];
                }
                if (Math.Abs(L[i, i]) < 1e-12) return null;
                y[i] = (b[i] - sum) / L[i, i];
            }

            var x = new double[n];
            for (int i = n - 1; i >= 0; i--)
            {
                double sum = 0;
                for (int k = i + 1; k < n; k++)
                {
                    sum += L[k, i] * x[k];
                }
                if (Math.Abs(L[i, i]) < 1e-12) return null;
                x[i] = (y[i] - sum) / L[i, i];
            }
            return x;
        }

        #endregion

        #region Inverse Kinematics (Geometric)

        public double[][] InverseGeometric(double[,] T_target, int verbose = 0)
        {
            ExtractFlangeAndPw(T_target, out var T_flange_target, out var Pw);
            if (verbose > 0)
                Console.WriteLine($"[InverseGeometric] Pw: ({Pw[0]:F3}, {Pw[1]:F3}, {Pw[2]:F3})");

            var solutions = new List<double[]>();

            var phi = Math.Atan2(Pw[1], Pw[0]);
            var q1cands = new[]
                {
                    MathUtil.NormalizeAngle(phi),
                    MathUtil.NormalizeAngle(phi + Math.PI)
                };

            foreach (var q1_var in q1cands)
            {
                if (!WithinLimit(0, q1_var)) continue;

                var q23Candidates = SolveQ2Q3(q1_var, Pw);
                foreach (var (q2_var, q3_var) in q23Candidates)
                {
                    if (!WithinLimit(1, q2_var) || !WithinLimit(2, q3_var))
                        continue;

                    var wristSols = SolveWristAngles(q1_var, q2_var, q3_var, T_flange_target);
                    foreach (var w in wristSols)
                    {
                        var q4_var = MathUtil.NormalizeAngle(w[0]);
                        var q5_var = MathUtil.NormalizeAngle(w[1]);
                        var q6_var = MathUtil.NormalizeAngle(w[2]);
                        if (!WithinLimit(3, q4_var) || !WithinLimit(4, q5_var) || !WithinLimit(5, q6_var))
                            continue;

                        var full = new[]
                            {
                                MathUtil.NormalizeAngle(q1_var),
                                MathUtil.NormalizeAngle(q2_var),
                                MathUtil.NormalizeAngle(q3_var),
                                q4_var, q5_var, q6_var
                            };

                        var Tchk = Forward(full);

                        // Position+coarse orientation gate (existing)
                        if (!PoseClose(Tchk, T_target, 5e-4, 5e-3))
                            continue;

                        // === Added precise orientation flip rejection ===
                        var angDeg = ComputeOrientationAngleDeg(Tchk, T_target);
                        if (angDeg >= OrientationFlipRejectDeg)
                        {
                            if (verbose > 0)
                                Console.WriteLine($"[InverseGeometric] reject flipped orientation ang={angDeg:F4} deg (q6 candidate)");
                            continue;
                        }
                        // ===============================================

                        if (!Duplicate(solutions, full, 1e-4))
                            solutions.Add(full);
                    }
                }
            }

            if (verbose > 0)
                Console.WriteLine($"[InverseGeometric] solutions={solutions.Count}");
            return solutions.ToArray();
        }

        private void ExtractFlangeAndPw(double[,] T_tcp, out double[,] T_flange, out double[] Pw)
        {
            var R_tcp = new double[3, 3];
            for (var i = 0; i < 3; i++)
                for (var j = 0; j < 3; j++)
                    R_tcp[i, j] = T_tcp[i, j];
            var P_tcp = new[] { T_tcp[0, 3], T_tcp[1, 3], T_tcp[2, 3] };

            var R_tool = _tool.Rotation;
            var R_tool_T = new double[3,3];
            for (var r = 0; r < 3; r++)
                for (var c = 0; c < 3; c++)
                    R_tool_T[r, c] = R_tool[c, r];

            var R_flange = new double[3,3];
            for (var r = 0; r < 3; r++)
                for (var c = 0; c < 3; c++)
                    for (var k = 0; k < 3; k++)
                        R_flange[r, c] += R_tcp[r, k] * R_tool_T[k, c];

            var p_tool = _tool.PositionLocal;
            var Rf_pt = new[]
                {
                    R_flange[0,0]*p_tool[0] + R_flange[0,1]*p_tool[1] + R_flange[0,2]*p_tool[2],
                    R_flange[1,0]*p_tool[0] + R_flange[1,1]*p_tool[1] + R_flange[1,2]*p_tool[2],
                    R_flange[2,0]*p_tool[0] + R_flange[2,1]*p_tool[1] + R_flange[2,2]*p_tool[2]
                };
            var P_flange = new[]
                {
                    P_tcp[0] - Rf_pt[0],
                    P_tcp[1] - Rf_pt[1],
                    P_tcp[2] - Rf_pt[2]
                };

            T_flange = MathUtil.Identity();
            for (var i = 0; i < 3; i++)
            {
                T_flange[i, 0] = R_flange[i, 0];
                T_flange[i, 1] = R_flange[i, 1];
                T_flange[i, 2] = R_flange[i, 2];
                T_flange[i, 3] = P_flange[i];
            }

            var d6 = _mdh.D[5];
            Pw =
            [
                P_flange[0] - R_flange[0,2]*d6,
                    P_flange[1] - R_flange[1,2]*d6,
                    P_flange[2] - R_flange[2,2]*d6
            ];
        }

        #endregion

        #region Geometric IK Helpers

        private List<(double, double)> SolveQ2Q3(double q1_var, double[] Pw)
        {
            var results = new List<(double, double)>();
            int samplesQ2 = 40, samplesQ3 = 40;
            var q2Min = _mdh.MinAnglesRad[1];
            var q2Max = _mdh.MaxAnglesRad[1];
            var q3Min = _mdh.MinAnglesRad[2];
            var q3Max = _mdh.MaxAnglesRad[2];

            var seedList = new List<(double q2, double q3, double err)>();
            for (var i = 0; i <= samplesQ2; i++)
            {
                var q2 = q2Min + (q2Max - q2Min) * i / samplesQ2;
                for (var j = 0; j <= samplesQ3; j++)
                {
                    var q3 = q3Min + (q3Max - q3Min) * j / samplesQ3;
                    var o4 = ComputeO4Position(q1_var, q2, q3);
                    var err = Dist3(o4, Pw);
                    if (err < 30.0) seedList.Add((q2, q3, err));
                }
            }

            seedList.Sort((a, b) => a.err.CompareTo(b.err));
            var keep = Math.Min(60, seedList.Count);
            for (var k = 0; k < keep; k++)
            {
                var (q2s, q3s, _) = seedList[k];
                var refined = RefineQ2Q3(q1_var, q2s, q3s, Pw);
                var o4r = ComputeO4Position(q1_var, refined.q2, refined.q3);
                var err = Dist3(o4r, Pw);
                if (err < 5e-3)
                {
                    var dup = false;
                    foreach (var ex in results)
                        if (Math.Abs(MathUtil.NormalizeAngle(ex.Item1 - refined.q2)) < 1e-3 &&
                            Math.Abs(MathUtil.NormalizeAngle(ex.Item2 - refined.q3)) < 1e-3)
                        {
                            dup = true; break;
                        }
                    if (!dup)
                        results.Add((MathUtil.NormalizeAngle(refined.q2), MathUtil.NormalizeAngle(refined.q3)));
                }
            }
            return results;
        }

        private (double q2, double q3) RefineQ2Q3(double q1_var, double q2Init, double q3Init, double[] Pw)
        {
            double q2 = q2Init, q3 = q3Init, eps = 1e-5;
            for (var iter = 0; iter < 60; iter++)
            {
                var f = Sub3(ComputeO4Position(q1_var, q2, q3), Pw);
                var err = Norm3(f);
                if (err < 1e-4) break;

                var J = new double[3, 2];
                var q2p = q2 + eps; var fp = Sub3(ComputeO4Position(q1_var, q2p, q3), Pw);
                var q2m = q2 - eps; var fm = Sub3(ComputeO4Position(q1_var, q2m, q3), Pw);
                for (var i = 0; i < 3; i++) J[i, 0] = (fp[i] - fm[i]) / (2 * eps);

                var q3p = q3 + eps; fp = Sub3(ComputeO4Position(q1_var, q2, q3p), Pw);
                var q3m = q3 - eps; fm = Sub3(ComputeO4Position(q1_var, q2, q3m), Pw);
                for (var i = 0; i < 3; i++) J[i, 1] = (fp[i] - fm[i]) / (2 * eps);

                double JTJ00=0, JTJ01=0, JTJ11=0, JTf0=0, JTf1=0;
                for (var i = 0; i < 3; i++)
                {
                    var Ji0 = J[i,0]; var Ji1 = J[i,1];
                    JTJ00 += Ji0 * Ji0; JTJ01 += Ji0 * Ji1; JTJ11 += Ji1 * Ji1;
                    JTf0 += Ji0 * f[i]; JTf1 += Ji1 * f[i];
                }
                var lambda = 1e-6;
                JTJ00 += lambda; JTJ11 += lambda;
                var det = JTJ00*JTJ11 - JTJ01*JTJ01;
                if (Math.Abs(det) < 1e-18) break;
                var inv00 = JTJ11/det;
                var inv01 = -JTJ01/det;
                var inv11 = JTJ00/det;
                var dq2 = -(inv00*JTf0 + inv01*JTf1);
                var dq3 = -(inv01*JTf0 + inv11*JTf1);

                q2 = MathUtil.NormalizeAngle(q2 + dq2);
                q3 = MathUtil.NormalizeAngle(q3 + dq3);

                q2 = Math.Max(_mdh.MinAnglesRad[1], Math.Min(_mdh.MaxAnglesRad[1], q2));
                q3 = Math.Max(_mdh.MinAnglesRad[2], Math.Min(_mdh.MaxAnglesRad[2], q3));
            }
            return (q2, q3);
        }

        private List<double[]> SolveWristAngles(double q1_var, double q2_var, double q3_var, double[,] T_flange_target)
        {
            var results = new List<double[]>();
            var T03e = GetT03Elbow([q1_var, q2_var, q3_var]);
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
                        if (sol == null) continue;
                        var q4r = MathUtil.NormalizeAngle(sol[0]);
                        var q5r = MathUtil.NormalizeAngle(sol[1]);
                        var q6r = MathUtil.NormalizeAngle(sol[2]);

                        var dup = false;
                        foreach (var w in results)
                            if (Math.Abs(MathUtil.NormalizeAngle(w[0] - q4r)) < 1e-3 &&
                                Math.Abs(MathUtil.NormalizeAngle(w[1] - q5r)) < 1e-3 &&
                                Math.Abs(MathUtil.NormalizeAngle(w[2] - q6r)) < 1e-3)
                            {
                                dup = true; break;
                            }

                        if (!dup)
                            results.Add(new[] { q4r, q5r, q6r });
                    }
            return results;
        }

        private double[]? RefineWrist(double[,] Tgoal, double q4Init, double q5Init, double q6Init)
        {
            double q4 = q4Init, q5 = q5Init, q6 = q6Init;
            var eps = 1e-5;
            for (var iter = 0; iter < 60; iter++)
            {
                var Twrist = WristForward(q4, q5, q6);
                var err = WristOrientError(Twrist, Tgoal);
                var n = Norm3(err);
                if (n < 5e-3) break;

                var J = new double[3,3];
                var errp = WristOrientError(WristForward(q4+eps,q5,q6), Tgoal);
                for (var i = 0; i < 3; i++) J[i, 0] = (errp[i] - err[i]) / eps;
                errp = WristOrientError(WristForward(q4, q5 + eps, q6), Tgoal);
                for (var i = 0; i < 3; i++) J[i, 1] = (errp[i] - err[i]) / eps;
                errp = WristOrientError(WristForward(q4, q5, q6 + eps), Tgoal);
                for (var i = 0; i < 3; i++) J[i, 2] = (errp[i] - err[i]) / eps;

                var JTJ = new double[3,3];
                var JTe = new double[3];
                for (var r = 0; r < 3; r++)
                {
                    for (var c = 0; c < 3; c++)
                        for (var k = 0; k < 3; k++)
                            JTJ[r, c] += J[k, r] * J[k, c];
                    for (var k = 0; k < 3; k++) JTe[r] += J[k, r] * err[k];
                }
                var lambda = 1e-5;
                JTJ[0, 0] += lambda; JTJ[1, 1] += lambda; JTJ[2, 2] += lambda;
                var dq = Solve3x3(JTJ, Negate(JTe));
                if (dq == null) break;

                var scale=1.0;
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
            return [q4, q5, q6];
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
            for (var axis = 0; axis < 3; axis++)
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

        private double[] ComputeO4Position(double q1_var, double q2_var, double q3_var)
        {
            var T03e = GetT03Elbow(new[]{ q1_var, q2_var, q3_var });
            var T = MathUtil.MatMul(T03e, MathUtil.DHTransform(0.0, 0.0, _mdh.A[3], _mdh.Alpha[3]));
            return [T[0, 3], T[1, 3], T[2, 3]];
        }

        private double[,] GetT03Elbow(double[] q_var)
        {
            var T = MathUtil.Identity();
            for (var i = 0; i < 3; i++)
                T = MathUtil.MatMul(T, MathUtil.DHTransform(
                    q_var[i] + _mdh.Offset[i],
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

        private static double[] Negate(double[] v)
        {
            var r = new double[v.Length];
            for (var i = 0; i < v.Length; i++) r[i] = -v[i];
            return r;
        }

        private static double[]? Solve3x3(double[,] A, double[] b)
        {
            var det =
                    A[0,0]*(A[1,1]*A[2,2]-A[1,2]*A[2,1]) -
                    A[0,1]*(A[1,0]*A[2,2]-A[1,2]*A[2,0]) +
                    A[0,2]*(A[1,0]*A[2,1]-A[1,1]*A[2,0]);
            if (Math.Abs(det) < 1e-14) return null;
            var invDet = 1.0/det;
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
            for (var i = 0; i < 3; i++)
                for (var k = 0; k < 3; k++)
                    x[i] += inv[i, k] * b[k];
            return x;
        }

        private bool PoseClose(double[,] A, double[,] B, double posTol, double oriTol)
        {
            double dp=0;
            for (var i = 0; i < 3; i++) { var d=A[i,3]-B[i,3]; dp += d * d; }
            if (Math.Sqrt(dp) > posTol) return false;
            double[] acc={0,0,0};
            for (var axis = 0; axis < 3; axis++)
            {
                var ac = new[]{ A[0,axis], A[1,axis], A[2,axis] };
                var bc = new[]{ B[0,axis], B[1,axis], B[2,axis] };
                var cr = MathUtil.Cross(ac, bc);
                for (var i = 0; i < 3; i++) acc[i] += cr[i];
            }
            for (var i = 0; i < 3; i++) acc[i] /= 3.0;
            return Norm3(acc) < oriTol;
        }

        private static bool Duplicate(List<double[]> sols, double[] cand, double tol)
        {
            foreach (var s in sols)
            {
                double maxd=0;
                for (var i = 0; i < s.Length; i++)
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
            for (var i = 0; i < 6; i++)
            {
                if (q[i] < _mdh.MinAnglesRad[i]) q[i] = _mdh.MinAnglesRad[i];
                if (q[i] > _mdh.MaxAnglesRad[i]) q[i] = _mdh.MaxAnglesRad[i];
            }
        }

        // === Orientation utilities for ranking ===
        private static void RelativeRotationAxisAngle(double[,] A, double[,] B, out double[] axis, out double angleDeg)
        {
            var Rrel = new double[3,3];
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    for (int k = 0; k < 3; k++)
                        Rrel[r, c] += A[k, r] * B[k, c]; // A^T * B

            var trace = Rrel[0,0]+Rrel[1,1]+Rrel[2,2];
            var cosT = Math.Clamp((trace - 1.0)/2.0,-1.0,1.0);
            var theta = Math.Acos(cosT);
            var s = 2.0*Math.Sin(theta);
            if (Math.Abs(s) < 1e-9) s = 1e-9;
            axis = new[]
            {
                    (Rrel[2,1]-Rrel[1,2])/s,
                    (Rrel[0,2]-Rrel[2,0])/s,
                    (Rrel[1,0]-Rrel[0,1])/s
                };
            angleDeg = theta * 180.0 / Math.PI;
        }

        private bool IsAxisAlignedFlip(double[,] cand, double[,] target)
        {
            RelativeRotationAxisAngle(cand, target, out var axis, out var angleDeg);
            if (angleDeg < OrientationFlipRejectDeg) return false;
            var zt = new[]{ target[0,2], target[1,2], target[2,2] };
            var normAxis = Math.Sqrt(axis[0]*axis[0]+axis[1]*axis[1]+axis[2]*axis[2]);
            if (normAxis < 1e-12) return false;
            var dot = (axis[0]*zt[0]+axis[1]*zt[1]+axis[2]*zt[2]) / normAxis;
            dot = Math.Clamp(dot, -1, 1);
            var alignDeg = Math.Acos(Math.Abs(dot))*180.0/Math.PI;
            return alignDeg <= FlipAxisAlignToleranceDeg;
        }

        private static double ComputeOrientationAngleDeg(double[,] A, double[,] B)
        {
            var RA = new double[3,3];
            var RB = new double[3,3];
            for (var r = 0; r < 3; r++)
            {
                RA[r, 0] = A[r, 0]; RA[r, 1] = A[r, 1]; RA[r, 2] = A[r, 2];
                RB[r, 0] = B[r, 0]; RB[r, 1] = B[r, 1]; RB[r, 2] = B[r, 2];
            }
            double tr=0;
            for (var k = 0; k < 3; k++)
                tr += RA[0, k] * RB[0, k] + RA[1, k] * RB[1, k] + RA[2, k] * RB[2, k];
            double cosTheta = Math.Clamp((tr - 1.0)/2.0,-1.0,1.0);
            return Math.Acos(cosTheta) * 180.0 / Math.PI;
        }

        // === Ranking additions ===
        public enum ElbowBranch { Down, Up }

        public sealed record IkRankedSolution(
            double[] Q,
            double ContinuityCost,
            double WristSingularityScore,
            ElbowBranch Branch,
            double FlipAngleDeg,
            bool FlipAxisAligned,
            double TotalScore);

        /// <summary>
        /// Rank IK geometric solutions using continuity, wrist singularity avoidance, elbow preference, and flip penalty.
        /// Solutions are re-enumerated WITHOUT flip rejection so that flips can be penalized instead of removed.
        /// </summary>
        public IkRankedSolution[] InverseGeometricRanked(
            double[,] T_target,
            double[]? previousQ = null,
            bool preferElbowDown = true,
            double continuityWeight = 1.0,
            double wristSingularityWeight = 0.5,
            double elbowPreferenceWeight = 0.5,
            double flipPenaltyWeight = 5.0,
            double[]? continuityJointWeights = null,
            int verbose = 0)
        {
            // Enumerate raw (copy of logic, skipping flip rejection block)
            ExtractFlangeAndPw(T_target, out var T_flange_target, out var Pw);

            var rawSolutions = new List<double[]>();
            var phi = Math.Atan2(Pw[1], Pw[0]);
            var q1cands = new[] { MathUtil.NormalizeAngle(phi), MathUtil.NormalizeAngle(phi + Math.PI) };

            foreach (var q1_var in q1cands)
            {
                if (!WithinLimit(0, q1_var)) continue;
                var q23Candidates = SolveQ2Q3(q1_var, Pw);
                foreach (var (q2_var, q3_var) in q23Candidates)
                {
                    if (!WithinLimit(1, q2_var) || !WithinLimit(2, q3_var)) continue;
                    var wristSols = SolveWristAngles(q1_var, q2_var, q3_var, T_flange_target);
                    foreach (var w in wristSols)
                    {
                        var q4_var = MathUtil.NormalizeAngle(w[0]);
                        var q5_var = MathUtil.NormalizeAngle(w[1]);
                        var q6_var = MathUtil.NormalizeAngle(w[2]);
                        if (!WithinLimit(3, q4_var) || !WithinLimit(4, q5_var) || !WithinLimit(5, q6_var))
                            continue;

                        var full = new[]
                            {
                                MathUtil.NormalizeAngle(q1_var),
                                MathUtil.NormalizeAngle(q2_var),
                                MathUtil.NormalizeAngle(q3_var),
                                q4_var, q5_var, q6_var
                            };

                        var Tchk = Forward(full);
                        if (!PoseClose(Tchk, T_target, 5e-4, 5e-3))
                            continue;

                        if (!Duplicate(rawSolutions, full, 1e-4))
                            rawSolutions.Add(full);
                    }
                }
            }

            var ranked = new List<IkRankedSolution>();
            var jointWeights = continuityJointWeights is { Length: 6 }
                    ? continuityJointWeights
                    : new[] { 1.0, 1.0, 1.0, 0.5, 0.5, 0.2 };

            foreach (var q in rawSolutions)
            {
                var Tcand = Forward(q);
                // Continuity cost
                double cont = 0.0;
                if (previousQ is { Length: 6 })
                    for (int i = 0; i < 6; i++)
                    {
                        var diff = MathUtil.NormalizeAngle(q[i] - previousQ[i]);
                        cont += jointWeights[i] * diff * diff;
                    }

                // Wrist singularity (q5 near 0 -> high penalty)
                var s5 = Math.Sin(q[4]); // q[4] is joint 5 (indexing from 0)
                var wristScore = 1.0 - Math.Abs(s5); // best near ±90° -> 0, worst near 0 -> 1

                // Elbow branch heuristic: Down if q2 < 0
                var branch = q[1] < 0 ? ElbowBranch.Down : ElbowBranch.Up;
                var elbowMismatch = (preferElbowDown && branch == ElbowBranch.Up) ||
                                        (!preferElbowDown && branch == ElbowBranch.Down) ? 1.0 : 0.0;

                // Flip classification
                RelativeRotationAxisAngle(Tcand, T_target, out var axis, out var angleDeg);
                bool flipAxisAligned = angleDeg >= OrientationFlipRejectDeg && IsAxisAlignedFlip(Tcand, T_target);

                var total = continuityWeight * cont +
                                wristSingularityWeight * wristScore +
                                elbowPreferenceWeight * elbowMismatch +
                                flipPenaltyWeight * (flipAxisAligned ? 1.0 : 0.0);

                ranked.Add(new IkRankedSolution(
                    (double[])q.Clone(),
                    cont,
                    wristScore,
                    branch,
                    angleDeg,
                    flipAxisAligned,
                    total));
            }

            var ordered = ranked.OrderBy(r => r.TotalScore).ToArray();

            if (verbose > 0)
            {
                Console.WriteLine("[InverseGeometricRanked] Solutions sorted by TotalScore:");
                for (int i = 0; i < ordered.Length; i++)
                {
                    var s = ordered[i];
                    Console.WriteLine($"  [{i}] Qdeg=[{DegStr(s.Q)}] score={s.TotalScore:F6} cont={s.ContinuityCost:F6} wrist={s.WristSingularityScore:F4} elbow={s.Branch} flipAng={s.FlipAngleDeg:F4} flipAxis={s.FlipAxisAligned}");
                }
            }

            return ordered;
        }

        private static string DegStr(double[] q) =>
            string.Join(", ", q.Select(r => (r * 180.0 / Math.PI).ToString("F4")));

        public void PrintJointZAxes(double[] q)
        {
            var transforms = GetFrameTransforms(q);
            // Use the corrected indices to print the correct Z axes for each joint
            int[] axisFrameIdx = { 0, 1, 2, 3, 5, 6 }; 
            for (var joint = 0; joint < 6; joint++)
            {
                var Tprev = transforms[axisFrameIdx[joint]];
                var z = new[] { Tprev[0, 2], Tprev[1, 2], Tprev[2, 2] };
                Console.WriteLine($"Joint {joint + 1} Z axis: ({z[0]:F6}, {z[1]:F6}, {z[2]:F6})");
            }
        }

        #endregion
    }
}
