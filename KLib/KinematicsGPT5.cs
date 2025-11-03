using System;
using System.Collections.Generic;

namespace KinematicsGPT5
{
    /// <summary>
    /// Provides Forward and Inverse Kinematics functions for a 6 DoF manipulator using MDH parameters.
    /// All calculations use double-based arrays and matrices, without external math libraries.
    /// </summary>
    public class KinematicsSolver
    {
        // MDH parameters mapped directly to the table rows.
        // Each index t corresponds to the MDH row used to build ^{t}T_{t+1}:
        // t = 0 => ^0T1 (uses alpha_0, a_0, d1, theta1)
        // t = 1 => ^1T2 (uses alpha_1, a_1, d2, theta2)
        // ...
        // Note: there is a fixed row "e" between joints 3 and 4 at index 3.
        private static readonly double[] alpha = { 0.0, Math.PI / 2.0, Math.PI, -Math.PI / 2.0, 0.0, -Math.PI / 2.0, Math.PI / 2.0 };
        private static readonly double[] a     = { 0.0, 29.690, 108.0, 20.0, 0.0, 0.0, 0.0 };
        // d array stores d1..d6 with the fixed 'e' row included (d_e = 0)
        private static readonly double[] d     = { 127.0, 0.0, 0.0, 0.0, 168.980, 0.0, 24.290 };
        // theta offsets for theta_i (theta_i = theta_offset[t] + jointAngle when that row corresponds to a joint)
        private static readonly double[] theta_offset = { 0.0, Math.PI / 2.0, 0.0, 0.0, 0.0, -Math.PI / 2.0, 0.0 };

        // --- Joint limits (degrees -> radians) ---
        private const double DEG_TO_RAD = Math.PI / 180.0;
        // θ1..θ5 : [-179.9999, 180.0000], θ6: [-359.9999, 360.0000]
        private static readonly double[] jointMin = {
            -179.9999 * DEG_TO_RAD,
            -179.9999 * DEG_TO_RAD,
            -179.9999 * DEG_TO_RAD,
            -179.9999 * DEG_TO_RAD,
            -179.9999 * DEG_TO_RAD,
            -359.9999 * DEG_TO_RAD
        };
        private static readonly double[] jointMax = {
             180.0000 * DEG_TO_RAD,
             180.0000 * DEG_TO_RAD,
             180.0000 * DEG_TO_RAD,
             180.0000 * DEG_TO_RAD,
             180.0000 * DEG_TO_RAD,
             360.0000 * DEG_TO_RAD
        };

        // Utility: clamp q to joint limits (in-place)
        private static void EnforceJointLimits(double[] q)
        {
            if (q == null) return;
            int n = Math.Min(q.Length, 6);
            for (int i = 0; i < n; i++)
            {
                if (q[i] < jointMin[i]) q[i] = jointMin[i];
                else if (q[i] > jointMax[i]) q[i] = jointMax[i];
            }
        }

        // Utility: check if q is within joint limits
        private static bool WithinJointLimits(double[] q)
        {
            if (q == null || q.Length < 6) return false;
            for (int i = 0; i < 6; i++)
            {
                if (q[i] < jointMin[i] - 1e-12 || q[i] > jointMax[i] + 1e-12) return false;
            }
            return true;
        }

        /// <summary>
        /// Computes the Forward Kinematics for the manipulator.
        /// Uses the MDH row mapping described above. The transform rows include the fixed 'e' row at index 3.
        /// </summary>
        /// <param name="jointAngles">length-6 array: theta1..theta6</param>
        /// <param name="tool">6-element tool (Xt,Yt,Zt,rx,ry,rz) or null for identity</param>
        /// <param name="mount">6-element mount (X0,Y0,Z0,rx0,ry0,rz0) or null for identity</param>
        /// <param name="verbose">0 none, 1 origins, 2 full matrices after each step</param>
        /// <returns>4x4 homogeneous transform of TCP in world coordinates</returns>
        public double[,] Forward(double[] jointAngles, double[] tool, double[] mount, int verbose = 0)
        {
            if (jointAngles == null || jointAngles.Length != 6)
                throw new ArgumentException("jointAngles must be length 6 (theta1..theta6).", nameof(jointAngles));

            double[] toolPose = tool ?? new double[6];
            double[] mountPose = mount ?? new double[6];

            // Compose base mounting transformation (row-major)
            double[,] T_base = KinematicsSolver.ComposeTransform(mountPose);
            // Compose tool transformation (row-major)
            double[,] T_tool = KinematicsSolver.ComposeTransform(toolPose);

            // Start with the base transform
            double[,] T = T_base;

            // There are 7 MDH rows (including the fixed 'e' row). For rows t=0..6:
            // - when row corresponds to joint i, joint index mapping is:
            //     if t < 3 -> jointIndex = t  (t=0->joint1, t=1->joint2, t=2->joint3)
            //     if t == 3 -> fixed 'e' (no joint angle)
            //     if t > 3 -> jointIndex = t-1 (t=4->joint4 (index3), t=5->joint5 (index4), t=6->joint6 (index5))
            for (var t = 0; t < alpha.Length; t++)
            {
                double theta;
                if (t == 3)
                {
                    // fixed row 'e' — no joint angle, only offset
                    theta = theta_offset[t];
                }
                else
                {
                    int jointIndex = (t < 3) ? t : (t - 1);
                    theta = theta_offset[t] + jointAngles[jointIndex];
                }

                double[,] T_i = KinematicsSolver.MDHTransform(alpha[t], a[t], theta, d[t]);

                // Multiply chain: T = T * T_i  (row-major; vectors treated as column vectors)
                T = KinematicsSolver.MatMul(T, T_i);

                if (verbose >= 2)
                {
                    Console.WriteLine($"--- After applying row {t} (MDH row {(t == 3 ? "e" : $"joint { (t < 3 ? t+1 : t) }")} ) ---");
                    KinematicsSolver.PrintMatrix(T_i);
                    Console.WriteLine("--- Cumulative T ---");
                    KinematicsSolver.PrintMatrix(T);
                }
            }

            // Apply tool transform
            T = KinematicsSolver.MatMul(T, T_tool);

            if (verbose == 1)
            {
                // Output local origins in absolute coordinates after each MDH row
                double[,] T_tmp = T_base;
                for (int t = 0; t < alpha.Length; t++)
                {
                    double theta;
                    if (t == 3)
                        theta = theta_offset[t];
                    else
                    {
                        int jointIndex = (t < 3) ? t : (t - 1);
                        theta = theta_offset[t] + jointAngles[jointIndex];
                    }

                    double[,] T_i = KinematicsSolver.MDHTransform(alpha[t], a[t], theta, d[t]);
                    T_tmp = KinematicsSolver.MatMul(T_tmp, T_i);
                    double[] origin = { 0, 0, 0, 1 };
                    double[] absOrigin = KinematicsSolver.MatVecMul(T_tmp, origin);
                    Console.WriteLine($"Origin after row {t} -> ({absOrigin[0]:F3}, {absOrigin[1]:F3}, {absOrigin[2]:F3})");
                }
            }

            return T;
        }

        /// <summary>
        /// Solves Inverse Kinematics using geometric approach.
        /// </summary>
        /// <param name="T_target">4x4 target transformation matrix (double[,]) for TCP.</param>
        /// <param name="mount">Array of 6 base mounting parameters.</param>
        /// <param name="tool">Array of 6 tool parameters.</param>
        /// <param name="verbose">Verbosity level.</param>
        /// <returns>Array of possible joint angle solutions (double[solution][6]).</returns>
        public double[][] InverseGeometric(double[,] T_target, double[] mount, double[] tool, int verbose = 0)
        {
            double[,] T_base = KinematicsSolver.ComposeTransform(mount ?? new double[6]);
            double[,] T_tool = KinematicsSolver.ComposeTransform(tool ?? new double[6]);

            // Work in robot flange frame: remove base and tool
            double[,] T = KinematicsSolver.MatMul(KinematicsSolver.MatInv(T_base), T_target);   // mul target from leftside
            T = KinematicsSolver.MatMul(T, KinematicsSolver.MatInv(T_tool));    //  mul target from rightside

            // R06 and Pw
            // "R0N" denotes the 3x3 rotation sub matrix from frame 0 (base/world) to frame N. It is upper-left 3x3 block of T0N (4x4 transform matrix).
            double[,] R06 = new double[3, 3];
            for (var i = 0; i < 3; i++)
                for (var j = 0; j < 3; j++)
                    R06[i, j] = T[i, j];

            double[] Pw = { T[0, 3] - T[0, 2] * d[6], T[1, 3] - T[1, 2] * d[6], T[2, 3] - T[2, 2] * d[6] };
            if (verbose >= 1)
            {
                Console.WriteLine($"Wrist position Pw: ({Pw[0]:F3}, {Pw[1]:F3}, {Pw[2]:F3})");
                for (var jc = 2; jc <= 6; jc++)
                {
                    double[,] Tj = TransformUpToJointCount(new double[] { 0,0,0,0,0,0 }, jc);
                    double[] Oj = { Tj[0,3], Tj[1,3], Tj[2,3] };
                    double dx = Oj[0] - Pw[0], dy = Oj[1] - Pw[1], dz = Oj[2] - Pw[2];
                    double dist = Math.Sqrt(dx*dx + dy*dy + dz*dz);
                    Console.WriteLine($"Debug: O{jc} = ({Oj[0]:F6},{Oj[1]:F6},{Oj[2]:F6})  dist->Pw={dist:F6}");
                }
                Console.WriteLine($"Debug: Pw = ({Pw[0]:F6},{Pw[1]:F6},{Pw[2]:F6})");

            }

            // local non-null args for Forward calls (silence nullable warnings)
            var toolArg = tool ?? new double[6];
            var mountArg = mount ?? new double[6];

            var solutions = new List<double[]>();
            
            var partialQ0to3 = new List<double[]>();    // for debugging

            // q0 candidates (base)
            double q0a = Math.Atan2(Pw[1], Pw[0]);
            double q0b = KinematicsSolver.NormalizeAngle(q0a + Math.PI);
            double[] q0candidates = { q0a, q0b };

            // For each q0 candidate solve shoulder/elbow in the O2-plane
            foreach (var q0 in q0candidates)
            {
                // We'll compute q1/q2 using geometry around O2 and O3.
                // To do this we need O2 and O3 in world coords for the current (q0, q1_guess=0, q2_guess=0)
                // Build a partial q with q0 set and others zero for computing O2/O3 positions
                double[] q_guess = new double[6];
                q_guess[0] = q0;
                q_guess[1] = 0.0;
                q_guess[2] = 0.0;
                q_guess[3] = 0.0;
                q_guess[4] = 0.0;
                q_guess[5] = 0.0;

                // Compute O2 and O3 in world frame using partial transforms (these depend on q0,q1,q2; we'll adjust q1,q2 later)
                // For accurate geometry we will recompute transforms after we determine q1,q2, but we need O2 reference now:
                // Instead compute O2 for q0 only (q1/q2=0) and we'll measure Pw relative to that O2 (this matches MDH where O2 location does not depend on q1/q2 translation along that joint axes).
                double[,] T02_base = KinematicsSolver.TransformUpToJointCount(new double[] { q_guess[0], 0, 0, 0, 0, 0 }, 2);
                double[] O2_base = { T02_base[0, 3], T02_base[1, 3], T02_base[2, 3] };

                // Compute vector from O2 to Pw (in world coords), then transform into O2-local XY plane using R02^T
                double[,] T02_full = KinematicsSolver.TransformUpToJointCount(new double[] { q_guess[0], 0, 0, 0, 0, 0 }, 2);
                double[,] R02 = new double[3, 3];
                for (var i = 0; i < 3; i++) for (var j = 0; j < 3; j++) R02[i, j] = T02_full[i, j];
                double[,] R02t = new double[3, 3];
                for (var i = 0; i < 3; i++) for (var j = 0; j < 3; j++) R02t[i, j] = R02[j, i];

                double[] vO2Pw = { Pw[0] - O2_base[0], Pw[1] - O2_base[1], Pw[2] - O2_base[2] };
                double[] vO2Pw_local = new double[3];
                for (var i = 0; i < 3; i++)
                    vO2Pw_local[i] = R02t[i, 0] * vO2Pw[0] + R02t[i, 1] * vO2Pw[1] + R02t[i, 2] * vO2Pw[2];

                // In O2-local coordinates, the 2-DOF planar problem lies in X-Z (use X as forward, Z as up)
                double px = vO2Pw_local[0];
                double pz = vO2Pw_local[2];
                double L = Math.Sqrt(px * px + pz * pz); // distance O2-Pw

                // Compute O3 in world coords for q0 and q1=q2=0 (kept for reference)
                double[,] T03_base = KinematicsSolver.TransformUpToJointCount(new double[] { q_guess[0], 0, 0, 0, 0, 0 }, 3);
                double[] O3_base = { T03_base[0, 3], T03_base[1, 3], T03_base[2, 3] };

                // Compute Oe (the flange point after the fixed 'e' row) for the same base guess.
                // Use Oe as the second vertex of the planar triangle because Pw is the wrist center
                // offset from the flange, not from joint-3 center.
                // Compute transform up to joint3 (exclude joint4)
                double[,] T03_base_for_e = KinematicsSolver.TransformUpToJointCount(new double[] { q_guess[0], 0, 0, 0, 0, 0 }, 3);
                // Apply the fixed 'e' MDH row (t == 3) explicitly — this yields the flange origin Oe (after the fixed row, before joint4)
                double theta_e = theta_offset[3]; // fixed row has no joint angle
                double[,] T_e_fixed = KinematicsSolver.MDHTransform(alpha[3], a[3], theta_e, d[3]);
                double[,] T03e_base = KinematicsSolver.MatMul(T03_base_for_e, T_e_fixed);
                double[] Oe_base = { T03e_base[0, 3], T03e_base[1, 3], T03e_base[2, 3] };

                // Use the triangle O2 - Oe - Pw for geometric q1/q2:
                //  - l1 = |O2 - Oe|
                //  - l2 = |Oe - Pw|
                //  - L  = |O2 - Pw|  (computed earlier)
                double l1 = Math.Sqrt(
                    (Oe_base[0] - O2_base[0]) * (Oe_base[0] - O2_base[0]) +
                    (Oe_base[1] - O2_base[1]) * (Oe_base[1] - O2_base[1]) +
                    (Oe_base[2] - O2_base[2]) * (Oe_base[2] - O2_base[2])
                );

                // l2 = distance Oe to Pw (use flange point Oe)
                double l2 = Math.Sqrt(
                    (Pw[0] - Oe_base[0]) * (Pw[0] - Oe_base[0]) +
                    (Pw[1] - Oe_base[1]) * (Pw[1] - Oe_base[1]) +
                    (Pw[2] - Oe_base[2]) * (Pw[2] - Oe_base[2])
                );

                if (l1 < 1e-9 || l2 < 1e-9)
                    continue;

                // Reachability check: L must be between |l1-l2| and l1+l2
                if (L > l1 + l2 + 1e-12 || L < Math.Abs(l1 - l2) - 1e-12)
                {
                    if (verbose >= 1) Console.WriteLine("Geometric: Pw outside reachable sub‑triangle (relative to O2/Oe).");
                    continue;
                }

                // law of cosines to get internal angle at the elbow triangle (angle between l1 and l2)
                double cos_internal = (l1 * l1 + l2 * l2 - L * L) / (2.0 * l1 * l2);
                cos_internal = Math.Max(-1.0, Math.Min(1.0, cos_internal));
                double internalAngle = Math.Acos(cos_internal);

                // Two elbow configurations: elbow-down / elbow-up
                double[] q2_candidates = { Math.PI - internalAngle, internalAngle - Math.PI };

                // compute base angle from O2 frame to Pw
                double gamma = Math.Atan2(pz, px); // angle of O2->Pw in O2-local X-Z plane

                foreach (var q2cand in q2_candidates)
                {
                    // angle between l1 and L
                    double cos_alpha = (l1 * l1 + L * L - l2 * l2) / (2.0 * l1 * L);
                    cos_alpha = Math.Max(-1.0, Math.Min(1.0, cos_alpha));
                    double alpha_angle = Math.Acos(cos_alpha);

                    // two possible alpha signs correspond to elbow choices: pick correct sign by branch
                    double q1cand = gamma - alpha_angle; // candidate for joint2 angle in O2-local plane

                    // Build candidate q0,q1,q2 (normalize)
                    // Insert immediately after constructing q_partial (replace the single assignment block)
                    double[] q_partial = new double[6];
                    q_partial[0] = KinematicsSolver.NormalizeAngle(q0);
                    q_partial[1] = KinematicsSolver.NormalizeAngle(q1cand);
                    q_partial[2] = KinematicsSolver.NormalizeAngle(q2cand);

                    // --- iterative local refinement for q1/q2 to account for MDH frame rotations ---
                    // repeat a few times to converge q1/q2 so O2/Oe/Pw geometry is consistent
                    for (var iterRef = 0; iterRef < 4; iterRef++)
                    {
                        // recompute O2 in world with current q_partial
                        double[,] T02_cur = KinematicsSolver.TransformUpToJointCount(new double[] { q_partial[0], q_partial[1], q_partial[2], 0, 0, 0 }, 2);
                        double[] O2_cur = { T02_cur[0, 3], T02_cur[1, 3], T02_cur[2, 3] };

                        // recompute Oe (apply fixed 'e' after T03 of current q_partial)
                        double[,] T03_cur = KinematicsSolver.TransformUpToJointCount(new double[] { q_partial[0], q_partial[1], q_partial[2], 0, 0, 0 }, 3);
                        double theta_e_local = theta_offset[3];
                        double[,] T_e = KinematicsSolver.MDHTransform(alpha[3], a[3], theta_e_local, d[3]);
                        double[,] T03e_cur = KinematicsSolver.MatMul(T03_cur, T_e);
                        double[] Oe_cur = { T03e_cur[0, 3], T03e_cur[1, 3], T03e_cur[2, 3] };

                        // transform v = Pw - O2 into O2-local
                        double[,] R02_cur = new double[3, 3];
                        for (var i = 0; i < 3; i++) for (var j = 0; j < 3; j++) R02_cur[i, j] = T02_cur[i, j];
                        double[,] R02t_cur = new double[3, 3];
                        for (var i = 0; i < 3; i++) for (var j = 0; j < 3; j++) R02t_cur[i, j] = R02_cur[j, i];
                        double[] vO2Pw_cur = { Pw[0] - O2_cur[0], Pw[1] - O2_cur[1], Pw[2] - O2_cur[2] };
                        double px_cur = R02t_cur[0,0]*vO2Pw_cur[0] + R02t_cur[0,1]*vO2Pw_cur[1] + R02t_cur[0,2]*vO2Pw_cur[2];
                        double pz_cur = R02t_cur[2,0]*vO2Pw_cur[0] + R02t_cur[2,1]*vO2Pw_cur[1] + R02t_cur[2,2]*vO2Pw_cur[2];
                        double L_cur = Math.Sqrt(px_cur*px_cur + pz_cur*pz_cur);

                        // recompute link lengths using Oe (flange) in world
                        double l1_cur = Math.Sqrt(
                            (Oe_cur[0] - O2_cur[0])*(Oe_cur[0] - O2_cur[0]) +
                            (Oe_cur[1] - O2_cur[1])*(Oe_cur[1] - O2_cur[1]) +
                            (Oe_cur[2] - O2_cur[2])*(Oe_cur[2] - O2_cur[2])
                        );
                                            double l2_cur = Math.Sqrt(
                            (Pw[0] - Oe_cur[0])*(Pw[0] - Oe_cur[0]) +
                            (Pw[1] - Oe_cur[1])*(Pw[1] - Oe_cur[1]) +
                            (Pw[2] - Oe_cur[2])*(Pw[2] - Oe_cur[2])
                        );

                        if (l1_cur < 1e-9 || l2_cur < 1e-9 || L_cur < 1e-12) break;

                        // law of cosines and planar base angle
                        double cos_int = (l1_cur*l1_cur + l2_cur*l2_cur - L_cur*L_cur) / (2.0 * l1_cur * l2_cur);
                        cos_int = Math.Max(-1.0, Math.Min(1.0, cos_int));
                        double internal_cur = Math.Acos(cos_int);
                        double gamma_cur = Math.Atan2(pz_cur, px_cur);
                        double cos_alpha_cur = (l1_cur*l1_cur + L_cur*L_cur - l2_cur*l2_cur) / (2.0 * l1_cur * L_cur);
                        cos_alpha_cur = Math.Max(-1.0, Math.Min(1.0, cos_alpha_cur));
                        double alpha_cur = Math.Acos(cos_alpha_cur);

                        double q1cand_new = gamma_cur - alpha_cur;
                        double q2cand_a = Math.PI - internal_cur;
                        double q2cand_b = internal_cur - Math.PI;
                        // choose q2 candidate closer to previous q_partial[2]
                        double pick = Math.Abs(NormalizeAngle(q2cand_a - q_partial[2])) <= Math.Abs(NormalizeAngle(q2cand_b - q_partial[2])) ? q2cand_a : q2cand_b;

                        // update joint variables (they are the joint angles, not MDH thetas)
                        q_partial[1] = KinematicsSolver.NormalizeAngle(q1cand_new);
                        q_partial[2] = KinematicsSolver.NormalizeAngle(pick);
                    }

                    if (verbose >= 1)
                    {
                        Console.WriteLine($"Candidate (q0,q1,q2) = ({q_partial[0]:F6}, {q_partial[1]:F6}, {q_partial[2]:F6})");
                        Console.WriteLine($" DEBUG q_partial (jointAngles): {q_partial[0]:F6}, {q_partial[1]:F6}, {q_partial[2]:F6}");
                        // show what MDH thetas become for rows t=0..3 (map rows -> joint index)
                        for (var t = 0; t <= 3; t++)
                        {
                            int rowJointIndex = (t == 3) ? -1 : (t < 3 ? t : t - 1);
                            if (rowJointIndex == -1)
                            {
                                double th = theta_offset[t];
                                Console.WriteLine($"  MDH row {t} (fixed 'e') theta = {th:F6} (offset only)");
                            }
                            else
                            {
                                double mdhTheta = theta_offset[t] + q_partial[rowJointIndex];
                                Console.WriteLine($"  MDH row {t} -> joint{rowJointIndex + 1}: mdhTheta = {mdhTheta:F6}  (offset {theta_offset[t]:F6} + q={q_partial[rowJointIndex]:F6})");
                            }
                        }
                    }

                    // Now enumerate q3 (θ4) two possibilities (above/below) as before
                    // compute T02/T03/T03e with the candidate q_partial
                    double[,] T02 = KinematicsSolver.TransformUpToJointCount(q_partial, 2);
                    double[,] T03 = KinematicsSolver.TransformUpToJointCount(q_partial, 3);
                    double[,] T03e = KinematicsSolver.TransformUpToJointCount(q_partial, 4);
                    double[] O2 = { T02[0, 3], T02[1, 3], T02[2, 3] };
                    double[] O3 = { T03[0, 3], T03[1, 3], T03[2, 3] };
                    double[] Oe = { T03e[0, 3], T03e[1, 3], T03e[2, 3] };

                    // vectors for q3 candidates in frame3
                    double[] v_e = { Oe[0] - O3[0], Oe[1] - O3[1], Oe[2] - O3[2] };
                    double[] v_pw = { Pw[0] - O3[0], Pw[1] - O3[1], Pw[2] - O3[2] };
                    double[,] R03 = new double[3, 3];
                    for (var i = 0; i < 3; i++) for (int j = 0; j < 3; j++) R03[i, j] = T03[i, j];
                    double[,] R03t = new double[3, 3];
                    for (var i = 0; i < 3; i++) for (var j = 0; j < 3; j++) R03t[i, j] = R03[j, i];

                    double[] v_e_3 = new double[3];
                    double[] v_pw_3 = new double[3];
                    for (var i = 0; i < 3; i++)
                    {
                        v_e_3[i] = R03t[i, 0] * v_e[0] + R03t[i, 1] * v_e[1] + R03t[i, 2] * v_e[2];
                        v_pw_3[i] = R03t[i, 0] * v_pw[0] + R03t[i, 1] * v_pw[1] + R03t[i, 2] * v_pw[2];
                    }

                    double ex = v_e_3[0], ey = v_e_3[1];
                    double px3 = v_pw_3[0], py3 = v_pw_3[1];
                    double dot = ex * px3 + ey * py3;
                    double crossz = ex * py3 - ey * px3;
                    double q3a = Math.Atan2(crossz, dot);
                    double q3b = KinematicsSolver.NormalizeAngle(q3a + Math.PI);
                    double[] q3candidates = { KinematicsSolver.NormalizeAngle(q3a), KinematicsSolver.NormalizeAngle(q3b) };

                    // For each q3 candidate continue with wrist extraction and verification (as implemented previously)
                    foreach (var q3cand in q3candidates)
                    {
                        double[] q_fixed = (double[])q_partial.Clone();
                        q_fixed[3] = KinematicsSolver.NormalizeAngle(q3cand);

                        // record the full partial (q0..q3) after q3 has been assigned
                        partialQ0to3.Add((double[])q_fixed.Clone());

                        if (verbose >= 1)
                        {
                            Console.WriteLine($" Fixed angles (known): q0..q3 = {q_fixed[0]:F6}, {q_fixed[1]:F6}, {q_fixed[2]:F6}, {q_fixed[3]:F6}");
                            Console.WriteLine(" Unknown angles (to solve): q4,q5,q6");
                        }

                        double[,] T04 = KinematicsSolver.TransformUpToJointCount(q_fixed, 4);
                        double[,] R04 = new double[3, 3];
                        for (var i = 0; i < 3; i++) for (int j = 0; j < 3; j++) R04[i, j] = T04[i, j];
                        double[,] R04t = new double[3, 3];
                        for (var i = 0; i < 3; i++) for (int j = 0; j < 3; j++) R04t[i, j] = R04[j, i];

                        double[,] R46 = new double[3, 3];
                        for (var i = 0; i < 3; i++)
                            for (var j = 0; j < 3; j++)
                            {
                                double ssum = 0.0;
                                for (var k = 0; k < 3; k++)
                                    ssum += R04t[i, k] * R06[k, j];
                                R46[i, j] = ssum;
                            }

                        if (KinematicsSolver.TryExtractZYX(R46, out double rx, out double ry, out double rz))
                        {
                            var wristCandidates = new List<double[]>();
                            wristCandidates.Add(new double[] { rx, ry, rz });
                            wristCandidates.Add(new double[] { KinematicsSolver.NormalizeAngle(rx + Math.PI), Math.PI - ry, KinematicsSolver.NormalizeAngle(rz + Math.PI) });

                            foreach (var w in wristCandidates)
                            {
                                double[] qfull = new double[6];
                                qfull[0] = q_fixed[0];
                                qfull[1] = q_fixed[1];
                                qfull[2] = q_fixed[2];
                                qfull[3] = q_fixed[3];
                                qfull[4] = KinematicsSolver.NormalizeAngle(w[0]);
                                qfull[5] = KinematicsSolver.NormalizeAngle(w[1]);

                                // try q6 wraps (±2π) for index 5
                                var qtries = new List<double[]>();
                                qtries.Add((double[])qfull.Clone());
                                var plus = (double[])qfull.Clone(); plus[5] = qfull[5] + 2.0 * Math.PI; qtries.Add(plus);
                                var minus = (double[])qfull.Clone(); minus[5] = qfull[5] - 2.0 * Math.PI; qtries.Add(minus);

                                foreach (var qtry in qtries)
                                {
                                    if (!KinematicsSolver.WithinJointLimits(qtry)) continue;
                                    double[,] T_from_q = this.Forward(qtry, toolArg, mountArg, 0);
                                    double[] err = KinematicsSolver.PoseError(T_from_q, T_target);
                                    double errNorm = 0.0;
                                    for (int i = 0; i < err.Length; i++) errNorm += err[i] * err[i];
                                    errNorm = Math.Sqrt(errNorm);

                                    if (verbose >= 1)
                                        Console.WriteLine($"   Trying candidate q: {KinematicsSolver.FormatQ(qtry)}  errNorm={errNorm:E6}");

                                    if (errNorm < 1e-6)
                                    {
                                        solutions.Add((double[])qtry.Clone());
                                        if (verbose >= 1) Console.WriteLine("   -> FK match: accepted solution.");
                                    }
                                }
                            }
                        }
                        else
                        {
                            if (verbose >= 1) Console.WriteLine("  Wrist extraction failed for this q3 candidate; wrist unknowns remain.");
                            if (verbose >= 1)
                            {
                                var partial = new double[6];
                                partial[0] = q_fixed[0]; partial[1] = q_fixed[1]; partial[2] = q_fixed[2]; partial[3] = q_fixed[3];
                                partial[4] = double.NaN; partial[5] = double.NaN;
                                Console.WriteLine($"  Partial candidate (fixed only): {KinematicsSolver.FormatQ(partial)}");
                            }
                        }
                    } // q3 candidates
                } // q2 candidates
            } // q0 candidates

            if (verbose >= 1)   // for debugging
            {
                Console.WriteLine($"[Geometric IK] collected {partialQ0to3.Count} partial (q0..q3) candidates:");
                foreach (var p in partialQ0to3)
                    Console.WriteLine($"  q0..q3 = {p[0]:F6}, {p[1]:F6}, {p[2]:F6}, {p[3]:F6}");
            }

            return solutions.Count == 0 ? new double[0][] : solutions.ToArray();
        }

        /// <summary>
        /// Solves Inverse Kinematics using Jacobian matrix iterative approach.
        /// </summary>
        /// <param name="T_target">4x4 target transformation matrix (double[,]) for TCP.</param>
        /// <param name="q_initial">Initial guess for joint angles (double[6]).</param>
        /// <param name="mount">Array of 6 base mounting parameters.</param>
        /// <param name="tool">Array of 6 tool parameters.</param>
        /// <param name="success">True if solution found within tolerance.</param>
        /// <param name="maxIterations">Maximum number of iterations.</param>
        /// <param name="tolerance">Solution tolerance.</param>
        /// <param name="alpha">Step size scaling factor.</param>
        /// <param name="damping">Damping factor for Jacobian inversion.</param>
        /// <param name="verbose">Verbosity level.</param>
        /// <returns>Joint angles solution (double[6]).</returns>
        public double[] InverseJacobian(
            double[,] T_target,
            double[] q_initial,
            double[] mount,
            double[] tool,
            out bool success,
            int maxIterations = 300,
            double tolerance = 1e-6,
            double alpha = 1.0,
            double damping = 1e-3,
            int verbose = 0)
        {
            if (q_initial == null || q_initial.Length != 6)
                throw new ArgumentException("q_initial must be length 6", nameof(q_initial));

            var toolArg = tool ?? new double[6];
            var mountArg = mount ?? new double[6];

            double[] q = (double[])q_initial.Clone();
            // Ensure starting guess respects limits
            KinematicsSolver.EnforceJointLimits(q);

            success = false;

            for (int iter = 0; iter < maxIterations; iter++)
            {
                double[,] T_current = this.Forward(q, toolArg, mountArg, 0);
                double[] error = KinematicsSolver.PoseError(T_current, T_target);

                double errNorm = 0;
                for (int i = 0; i < error.Length; i++) errNorm += error[i] * error[i];
                errNorm = Math.Sqrt(errNorm);

                if (verbose == 1)
                {
                    Console.WriteLine($"Iter {iter}: error norm = {errNorm:F6}");
                }

                if (errNorm < tolerance)
                {
                    // also ensure final q is within limits
                    KinematicsSolver.EnforceJointLimits(q);
                    success = KinematicsSolver.WithinJointLimits(q);
                    break;
                }

                double[,] J = this.Jacobian(q, toolArg, mountArg);
                double[,] J_pinv = KinematicsSolver.DampedPseudoInverse(J, damping);

                double[] dq = KinematicsSolver.MatVecMul(J_pinv, error);
                for (int i = 0; i < q.Length; i++)
                    q[i] += alpha * dq[i];

                // Enforce limits immediately after update so solver stays in feasible space
                KinematicsSolver.EnforceJointLimits(q);
            }

            // If exited loop without meeting tolerance, success remains false (or false if outside limits)
            if (!success)
            {
                // Final verification if error small despite loop end
                double[,] T_final = this.Forward(q, toolArg, mountArg, 0);
                double[] finalErr = KinematicsSolver.PoseError(T_final, T_target);
                double finalErrNorm = 0;
                for (int i = 0; i < finalErr.Length; i++) finalErrNorm += finalErr[i] * finalErr[i];
                finalErrNorm = Math.Sqrt(finalErrNorm);
                if (finalErrNorm < tolerance && KinematicsSolver.WithinJointLimits(q))
                    success = true;
            }

            return q;
        }

        // --- Utility functions (unchanged) ---

        // Compose a homogeneous transformation from (X, Y, Z, rx, ry, rz)
        private static double[,] ComposeTransform(double[] pose)
        {
            double[,] T = KinematicsSolver.Identity4();
            if (pose == null) return T;
            T = KinematicsSolver.MatMul(T, KinematicsSolver.RotZ(pose[5]));
            T = KinematicsSolver.MatMul(T, KinematicsSolver.RotY(pose[4]));
            T = KinematicsSolver.MatMul(T, KinematicsSolver.RotX(pose[3]));
            T[0, 3] = pose[0];
            T[1, 3] = pose[1];
            T[2, 3] = pose[2];
            return T;
        }

        // MDH transformation matrix (follows the sequence: RotX(alpha) -> TransX(a) -> RotZ(theta) -> TransZ(d))
        private static double[,] MDHTransform(double alpha, double a, double theta, double d)
        {
            double ca = Math.Cos(alpha), sa = Math.Sin(alpha);
            double ct = Math.Cos(theta), st = Math.Sin(theta);

            // Build RotX(alpha)
            double[,] Rx = new double[,] {
                {1,0,0,0},
                {0,ca,-sa,0},
                {0,sa,ca,0},
                {0,0,0,1}
            };
            // TransX(a)
            double[,] Tx = KinematicsSolver.Identity4();
            Tx[0, 3] = a;
            // RotZ(theta)
            double[,] Rz = new double[,] {
                {ct,-st,0,0},
                {st,ct,0,0},
                {0,0,1,0},
                {0,0,0,1}
            };
            // TransZ(d)
            double[,] Tz = KinematicsSolver.Identity4();
            Tz[2, 3] = d;

            // Sequence: RotX(alpha) -> TransX(a) -> RotZ(theta) -> TransZ(d)
            double[,] T = KinematicsSolver.MatMul(Rx, Tx);
            T = KinematicsSolver.MatMul(T, Rz);
            T = KinematicsSolver.MatMul(T, Tz);
            return T;
        }

        // Rotation matrices
        private static double[,] RotX(double angle)
        {
            double c = Math.Cos(angle), s = Math.Sin(angle);
            return new double[,] {
                {1,0,0,0},
                {0,c,-s,0},
                {0,s,c,0},
                {0,0,0,1}
            };
        }
        private static double[,] RotY(double angle)
        {
            double c = Math.Cos(angle), s = Math.Sin(angle);
            return new double[,] {
                {c,0,s,0},
                {0,1,0,0},
                {-s,0,c,0},
                {0,0,0,1}
            };
        }
        private static double[,] RotZ(double angle)
        {
            double c = Math.Cos(angle), s = Math.Sin(angle);
            return new double[,] {
                {c,-s,0,0},
                {s,c,0,0},
                {0,0,1,0},
                {0,0,0,1}
            };
        }

        // Matrix multiplication (4x4)
        private static double[,] MatMul(double[,] A, double[,] B)
        {
            int n = 4;
            double[,] C = new double[n, n];
            for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++)
                {
                    double s = 0.0;
                    for (int k = 0; k < n; k++)
                        s += A[i, k] * B[k, j];
                    C[i, j] = s;
                }
            return C;
        }

        // Matrix-vector multiplication (4x4 * 4x1)
        private static double[] MatVecMul(double[,] M, double[] v)
        {
            double[] r = new double[4];
            for (int i = 0; i < 4; i++)
            {
                double s = 0.0;
                for (int j = 0; j < 4; j++)
                    s += M[i, j] * v[j];
                r[i] = s;
            }
            return r;
        }

        // Identity matrix (4x4)
        private static double[,] Identity4()
        {
            double[,] I = new double[4, 4];
            for (int i = 0; i < 4; i++)
                I[i, i] = 1.0;
            return I;
        }

        // Matrix inverse (for 4x4 homogeneous, only rotation+translation)
        private static double[,] MatInv(double[,] T)
        {
            double[,] R = new double[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    R[i, j] = T[i, j];
            double[,] Rt = new double[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    Rt[i, j] = R[j, i];
            double[] p = { T[0, 3], T[1, 3], T[2, 3] };
            double[] pt = new double[3];
            for (int i = 0; i < 3; i++)
            {
                pt[i] = 0;
                for (int j = 0; j < 3; j++)
                    pt[i] -= Rt[i, j] * p[j];
            }
            double[,] Tinv = KinematicsSolver.Identity4();
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    Tinv[i, j] = Rt[i, j];
            for (int i = 0; i < 3; i++)
                Tinv[i, 3] = pt[i];
            return Tinv;
        }

        // Print matrix (for verbose)
        private static void PrintMatrix(double[,] M)
        {
            for (int i = 0; i < M.GetLength(0); i++)
            {
                for (int j = 0; j < M.GetLength(1); j++)
                    Console.Write($"{M[i, j]:F4}\t");
                Console.WriteLine();
            }
        }

        // Pose error between two transforms (position+simple orientation)
        private static double[] PoseError(double[,] T1, double[,] T2)
        {
            double[] err = new double[6];
            for (int i = 0; i < 3; i++)
                err[i] = T2[i, 3] - T1[i, 3];
            for (int i = 0; i < 3; i++)
                err[3 + i] = T2[i, 2] - T1[i, 2];
            return err;
        }

        // Jacobian (numerical, position + simple orientation)
        private double[,] Jacobian(double[] q, double[] tool, double[] mount)
        {
            int n = q.Length;
            double[,] J = new double[6, n];
            double eps = 1e-8;
            double[,] T0 = Forward(q, tool ?? new double[6], mount ?? new double[6], 0);
            double[] p0 = { T0[0, 3], T0[1, 3], T0[2, 3] };
            for (int i = 0; i < n; i++)
            {
                double[] qd = (double[])q.Clone();
                qd[i] += eps;
                double[,] Td = Forward(qd, tool ?? new double[6], mount ?? new double[6], 0);
                double[] pd = { Td[0, 3], Td[1, 3], Td[2, 3] };
                for (int j = 0; j < 3; j++)
                    J[j, i] = (pd[j] - p0[j]) / eps;
                for (int j = 0; j < 3; j++)
                    J[3 + j, i] = (Td[j, 2] - T0[j, 2]) / eps;
            }
            return J;
        }

        // Damped pseudo-inverse (for Jacobian)
        private static double[,] DampedPseudoInverse(double[,] J, double damping)
        {
            int m = J.GetLength(0), n = J.GetLength(1);
            double[,] Jt = new double[n, m];
            for (int i = 0; i < n; i++)
                for (int j = 0; j < m; j++)
                    Jt[i, j] = J[j, i];
            double[,] JJt = new double[n, n];
            for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++)
                {
                    JJt[i, j] = 0.0;
                    for (int k = 0; k < m; k++)
                        JJt[i, j] += Jt[i, k] * J[k, j];
                    if (i == j) JJt[i, j] += damping * damping;
                }
            double[,] JJt_inv = MatrixInverse(JJt);
            double[,] J_pinv = new double[n, m];
            for (int i = 0; i < n; i++)
                for (int j = 0; j < m; j++)
                {
                    double s = 0.0;
                    for (int k = 0; k < n; k++)
                        s += JJt_inv[i, k] * Jt[k, j];
                    J_pinv[i, j] = s;
                }
            return J_pinv;
        }

        // Matrix inverse (Gauss-Jordan; small matrices)
        private static double[,] MatrixInverse(double[,] M)
        {
            int n = M.GetLength(0);
            double[,] A = new double[n, n];
            Array.Copy(M, A, M.Length);
            double[,] I = new double[n, n];
            for (int i = 0; i < n; i++) I[i, i] = 1.0;
            for (int i = 0; i < n; i++)
            {
                double diag = A[i, i];
                if (Math.Abs(diag) < 1e-12) throw new InvalidOperationException("Matrix is singular or ill-conditioned.");
                for (int j = 0; j < n; j++)
                {
                    A[i, j] /= diag;
                    I[i, j] /= diag;
                }
                for (int k = 0; k < n; k++)
                {
                    if (k == i) continue;
                    double factor = A[k, i];
                    for (int j = 0; j < n; j++)
                    {
                        A[k, j] -= factor * A[i, j];
                        I[k, j] -= factor * I[i, j];
                    }
                }
            }
            return I;
        }

        // --- Restored helper methods required by InverseGeometric ---

        // Build transform from frame0 to frame_{jointCount} (robot frame, no base/tool).
        // jointCount: number of joints to include (1..6). For jointCount >=4 this will include the fixed 'e' row.
        private static double[,] TransformUpToJointCount(double[] q, int jointCount)
        {
            double[,] T = KinematicsSolver.Identity4();
            int processed = 0;
            for (int t = 0; t < alpha.Length; t++)
            {
                int rowJointIndex = (t == 3) ? -1 : (t < 3 ? t : t - 1);
                bool apply = false;
                if (rowJointIndex == -1)
                {
                    // fixed 'e' row: include only if we are building beyond joint3 (i.e., want joint4..6)
                    if (jointCount > 3) apply = true;
                }
                else
                {
                    if (rowJointIndex < jointCount) apply = true;
                }

                if (!apply) continue;

                double theta;
                if (rowJointIndex == -1) theta = theta_offset[t];
                else theta = theta_offset[t] + q[rowJointIndex];

                double[,] Ti = KinematicsSolver.MDHTransform(alpha[t], a[t], theta, d[t]);
                T = KinematicsSolver.MatMul(T, Ti);

                if (rowJointIndex != -1) processed++;
            }
            return T;
        }

        // Normalize angle to range (-PI, PI]
        private static double NormalizeAngle(double angle)
        {
            double a = angle;
            while (a <= -Math.PI) a += 2.0 * Math.PI;
            while (a > Math.PI) a -= 2.0 * Math.PI;
            return a;
        }

        // Try to extract angles for R = Rz(gamma)*Ry(beta)*Rx(alpha)
        // Returns rx(alpha), ry(beta), rz(gamma)
        private static bool TryExtractZYX(double[,] R, out double rx, out double ry, out double rz)
        {
            double r00 = R[0, 0], r10 = R[1, 0], r20 = R[2, 0];
            double r21 = R[2, 1], r22 = R[2, 2];

            double cy = Math.Sqrt(r00 * r00 + r10 * r10);
            ry = Math.Atan2(-r20, cy);

            if (Math.Abs(cy) < 1e-12)
            {
                // near gimbal: set rx = 0, estimate rz
                rx = 0.0;
                rz = Math.Atan2(-R[0, 1], R[1, 1]);
                return true;
            }
            else
            {
                rx = Math.Atan2(r21, r22);
                rz = Math.Atan2(r10, r00);
                return true;
            }
        }

        // Helper to format q array for printing (diagnostics)
        private static string FormatQ(double[] q)
        {
            if (q == null) return "(null)";
            var s = "";
            for (int i = 0; i < q.Length; i++)
                s += $"{q[i]:F6}" + (i + 1 < q.Length ? ", " : "");
            return s;
        }
    }
}
