using System;
using System.Linq;
using KinematicsRM2;

class Program
{
    private static void Main()
    {
        Console.WriteLine("--- Kinematics Final Check ---");

        var robot = ManipulatorFactory.CreateManipulator("MiRobot","TOOL0");

        // Show tool (flange -> TCP) local homogeneous transform (built from ToolSpecification)
        var tool = robot.Tool;
        var T_tool = new double[4,4];
        for (var r = 0; r < 3; r++)
        {
            T_tool[r, 0] = tool.Rotation[r, 0];
            T_tool[r, 1] = tool.Rotation[r, 1];
            T_tool[r, 2] = tool.Rotation[r, 2];
        }
        T_tool[0, 3] = tool.PositionLocal[0];
        T_tool[1, 3] = tool.PositionLocal[1];
        T_tool[2, 3] = tool.PositionLocal[2];
        T_tool[3, 3] = 1.0;

        Console.WriteLine("\nTool local matrix (flange -> TCP):");
        PrintMatrix(T_tool);

        var q_target = new[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        Console.WriteLine($"\n1. Target Joint Angles (deg): [{string.Join(", ", q_target.Select(r => r * 180 / Math.PI).Select(d => d.ToString("F4")))}]");

        var T_target = robot.Forward(q_target, verbose:2);
        Console.WriteLine("\n2. Target Cartesian Pose (matrix): ");
        PrintMatrix(T_target); // position already F3

        Console.WriteLine("\n3. Solving with InverseJacobian()...");
        var q_guess = new double[] { 0, 0, 0, 0, 0, 0 };
        var ik_sol = robot.InverseJacobian(T_target, q_guess, out var success, verbose:1);

        if (!success || ik_sol == null)
        {
            Console.WriteLine("   Jacobian method failed to find a solution.");
            Console.WriteLine("\nCHECK FAILED");
            return;
        }
        Console.WriteLine($"   Found solution (deg): [{string.Join(", ", ik_sol.Select(r => r * 180 / Math.PI).Select(d => d.ToString("F4")))}]");

        Console.WriteLine("\n4. Verifying by feeding IK solution back into FK...");
        var T_check = robot.Forward(ik_sol);
        Console.WriteLine("   Resulting Cartesian Pose from IK solution:");
        PrintMatrix(T_check);

        double pos_error = 0;
        for (var i = 0; i < 3; i++)
        {
            var diff = T_target[i, 3] - T_check[i, 3];
            pos_error += diff * diff;
        }
        pos_error = Math.Sqrt(pos_error);

        double orientation_error = 0;
        for (var i = 0; i < 3; i++)
            for (var j = 0; j < 3; j++)
            {
                var diff = T_target[i,j] - T_check[i,j];
                orientation_error += diff * diff;
            }
        orientation_error = Math.Sqrt(orientation_error);

        Console.WriteLine($"\n   Pose Error (Position Norm): {pos_error:F3}");
        Console.WriteLine($"   Pose Error (Orientation Matrix Norm): {orientation_error:F6}");

        var tolerance = 1e-3;
        if (pos_error < tolerance && orientation_error < tolerance)
            Console.WriteLine("\nForward() - InverseJacobian(): CHECK PASSED");
        else
            Console.WriteLine("\nForward() - InverseJacobian(): CHECK FAILED");

        Console.WriteLine("\n5. Solving with InverseGeometric() (enumerating all solutions)...");
        var geo_solutions = robot.InverseGeometric(T_target, verbose:1);
        Console.WriteLine($"   Number of geometric IK solutions: {geo_solutions.Length}");

        for (var i = 0; i < geo_solutions.Length; i++)
        {
            var qs = geo_solutions[i];
            var degStr = string.Join(", ", qs.Select(r => (r * 180 / Math.PI).ToString("F4")));
            Console.WriteLine($"   [{i}] (deg): [{degStr}]");
        }

        // --- 5.a Forward() consistency check across geometric solutions ---
        if (geo_solutions.Length > 0)
        {
            Console.WriteLine("\n5.a Forward() consistency check across geometric solutions...");
            const double posTol = 1e-3;        // position norm tolerance
            const double orientTolFrob = 1e-3; // orientation Frobenius norm tolerance (internal)
            var forwardPosErrors = new double[geo_solutions.Length];
            var forwardOrientNorms = new double[geo_solutions.Length];
            var forwardOrientAnglesDeg = new double[geo_solutions.Length];
            var matrices = new double[geo_solutions.Length][,];

            // Compute FK per geometric solution and error vs T_target
            for (var i = 0; i < geo_solutions.Length; i++)
            {
                var T_sol = robot.Forward(geo_solutions[i]);
                matrices[i] = T_sol;

                double pe = 0;
                for (var r = 0; r < 3; r++)
                {
                    var d = T_target[r, 3] - T_sol[r, 3];
                    pe += d * d;
                }
                pe = Math.Sqrt(pe);

                double frob = 0;
                for (var r = 0; r < 3; r++)
                    for (var c = 0; c < 3; c++)
                    {
                        var d = T_target[r, c] - T_sol[r, c];
                        frob += d * d;
                    }
                frob = Math.Sqrt(frob);

                var angleRad = RotationAngle(T_target, T_sol);
                var angleDeg = angleRad * 180.0 / Math.PI;

                forwardPosErrors[i] = pe;
                forwardOrientNorms[i] = frob;
                forwardOrientAnglesDeg[i] = angleDeg;

                Console.WriteLine($"   FK[{i}] position error: {pe:E3}, orient angle: {angleDeg:F4} deg");
            }

            // Pairwise pose discrepancies (position & orientation angle)
            if (geo_solutions.Length > 1)
            {
                Console.WriteLine("   Pairwise pose discrepancies (position / orientation angle deg):");
                for (var i = 0; i < geo_solutions.Length; i++)
                {
                    for (var j = i + 1; j < geo_solutions.Length; j++)
                    {
                        double pErr = 0;
                        for (var r = 0; r < 3; r++)
                        {
                            var d = matrices[i][r, 3] - matrices[j][r, 3];
                            pErr += d * d;
                        }
                        pErr = Math.Sqrt(pErr);

                        var angRad = RotationAngle(matrices[i], matrices[j]);
                        var angDeg = angRad * 180.0 / Math.PI;

                        Console.WriteLine($"      [{i},{j}] pos: {pErr:E3}, orient angle: {angDeg:F4} deg");
                    }
                }
            }

            var allOk = forwardPosErrors.All(e => e <= posTol) &&
                        forwardOrientNorms.All(e => e <= orientTolFrob);
            if (allOk)
                Console.WriteLine("   Forward() outputs for all geometric solutions: CONSISTENT (within tolerance)");
            else
            {
                Console.WriteLine("   Forward() outputs for geometric solutions: INCONSISTENT (exceed tolerance)");
                for (var i = 0; i < geo_solutions.Length; i++)
                {
                    if (forwardPosErrors[i] > posTol || forwardOrientNorms[i] > orientTolFrob)
                        Console.WriteLine($"      -> Solution [{i}] exceeds tolerance (pos {forwardPosErrors[i]:E3}, orient angle {forwardOrientAnglesDeg[i]:F4} deg)");
                }
            }
        }
        // --- End added block ---

        // --- 5.b Ranking geometric IK solutions ---
        if (geo_solutions.Length > 0)
        {
            Console.WriteLine("\n5.b Ranking geometric IK solutions (continuity vs Jacobian solution, elbow-down preference, flip penalty)...");

            // Rank using previous configuration = ik_sol (Jacobian result) for continuity
            var ranked = robot.InverseGeometricRanked(
                T_target,
                previousQ: ik_sol,
                preferElbowDown: true,
                continuityWeight: 1.0,
                wristSingularityWeight: 0.3,
                elbowPreferenceWeight: 0.3,
                flipPenaltyWeight: 20.0, // strong penalty pushes flips to bottom
                continuityJointWeights: new[] { 1.0, 1.0, 1.0, 0.5, 0.5, 0.2 },
                verbose: 0);

            // Map ranked solutions back to original geo indices (if present)
            for (var i = 0; i < ranked.Length; i++)
            {
                var rs = ranked[i];
                var idxOrig = FindMatchingSolution(rs.Q, geo_solutions, 1e-6);
                var degStr = string.Join(", ", rs.Q.Select(r => (r * 180 / Math.PI).ToString("F4")));
                Console.WriteLine(
                    $"   Rank[{i}] origIdx={(idxOrig >= 0 ? idxOrig.ToString() : "-")} score={rs.TotalScore:F6} cont={rs.ContinuityCost:F6} " +
                    $"wrist={rs.WristSingularityScore:F4} elbow={rs.Branch} flipAng={rs.FlipAngleDeg:F4} flipAxis={rs.FlipAxisAligned} => [{degStr}]");
            }

            var best = ranked.First();
            var bestDegStr = string.Join(", ", best.Q.Select(r => (r * 180 / Math.PI).ToString("F4")));
            Console.WriteLine($"\n   Best Ranked Solution (deg): [{bestDegStr}]");
            Console.WriteLine($"     TotalScore={best.TotalScore:F6} Continuity={best.ContinuityCost:F6} WristSing={best.WristSingularityScore:F4} Elbow={best.Branch} FlipAngle={best.FlipAngleDeg:F4} FlipAxis={best.FlipAxisAligned}");

            // Compare best ranked solution to Jacobian solution
            Console.WriteLine("   Diff (best ranked - Jacobian) (rad): " +
                string.Join(", ", ik_sol.Zip(best.Q, (a, b) => NormalizeAngle(b - a).ToString("F6"))));
        }
        // --- End ranking block ---

        var jacobianInGeoIndex = FindMatchingSolution(ik_sol, geo_solutions, 1e-4);
        if (jacobianInGeoIndex >= 0)
        {
            Console.WriteLine($"\nInverseJacobian() - InverseGeometric(): CHECK PASSED (index {jacobianInGeoIndex})");
            ReportJointDiff("   Jacobian vs Geometric match diff (rad)", ik_sol, geo_solutions[jacobianInGeoIndex]);
        }
        else
        {
            Console.WriteLine("\nInverseJacobian() - InverseGeometric(): CHECK FAILED (Jacobian solution absent)");
            var (closestIdx, maxErr) = FindClosestSolution(ik_sol, geo_solutions);
            if (closestIdx >= 0)
            {
                Console.WriteLine($"   Closest geometric solution index {closestIdx}, max joint error = {maxErr:E3} rad");
                ReportJointDiff("   Jacobian vs Closest (rad)", ik_sol, geo_solutions[closestIdx]);
            }
        }

        var forwardMatchIndex = FindMatchingSolution(q_target, geo_solutions, 1e-4);
        if (forwardMatchIndex >= 0)
        {
            Console.WriteLine($"\nForward() - InverseGeometric(): CHECK PASSED (index {forwardMatchIndex})");
            ReportJointDiff("   Forward vs Geometric match diff (rad)", q_target, geo_solutions[forwardMatchIndex]);
        }
        else
        {
            Console.WriteLine("\nForward() - InverseGeometric(): CHECK FAILED (original joint vector absent)");
            var (closestIdx, maxErr) = FindClosestSolution(q_target, geo_solutions);
            if (closestIdx >= 0)
            {
                Console.WriteLine($"   Closest geometric solution index {closestIdx}, max joint error = {maxErr:E3} rad");
                ReportJointDiff("   Forward vs Closest (rad)", q_target, geo_solutions[closestIdx]);
            }
        }

        for (var i = 0; i < geo_solutions.Length; i++)
        {
            var T_g = robot.Forward(geo_solutions[i]);
            double pe = 0;
            for (var r = 0; r < 3; r++)
            {
                var d = T_target[r, 3] - T_g[r, 3];
                pe += d * d;
            }
            pe = Math.Sqrt(pe);
            if (pe > 1e-3)
                Console.WriteLine($"   Warning: solution [{i}] position error {pe:E3} exceeds tolerance.");
        }
    }

    private static int FindMatchingSolution(double[] target, double[][] candidates, double angleTolRad)
    {
        if (candidates == null) return -1;
        for (var i = 0; i < candidates.Length; i++)
        {
            var all = true;
            for (var j = 0; j < 6; j++)
            {
                if (Math.Abs(NormalizeAngle(candidates[i][j] - target[j])) > angleTolRad)
                {
                    all = false;
                    break;
                }
            }
            if (all) return i;
        }
        return -1;
    }

    private static (int idx, double maxErr) FindClosestSolution(double[] target, double[][] candidates)
    {
        var best = -1;
        var bestMax = double.PositiveInfinity;
        if (candidates == null) return (best, bestMax);
        for (var i = 0; i < candidates.Length; i++)
        {
            double mx = 0;
            for (var j = 0; j < 6; j++)
            {
                var e = Math.Abs(NormalizeAngle(candidates[i][j] - target[j]));
                if (e > mx) mx = e;
                if (mx >= bestMax) break;
            }
            if (mx < bestMax)
            {
                bestMax = mx;
                best = i;
            }
        }
        return (best, bestMax);
    }

    private static void ReportJointDiff(string header, double[] refQ, double[] testQ)
    {
        Console.WriteLine(header + ": " +
            string.Join(", ", refQ.Zip(testQ, (a, b) => NormalizeAngle(b - a).ToString("F6"))));
    }

    private static double NormalizeAngle(double a)
    {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private static void PrintMatrix(double[,] M)
    {
        for (var r = 0; r < 4; r++)
            Console.WriteLine($"  [{M[r, 0],8:F3},{M[r, 1],8:F3},{M[r, 2],8:F3},{M[r, 3],8:F3}]");
    }

    // Rotation angle between target and solution (radians)
    private static double RotationAngle(double[,] R_target, double[,] R_sol)
    {
        double tr = 0;
        for (var r = 0; r < 3; r++)
            for (var c = 0; c < 3; c++)
                tr += R_target[r, c] * R_sol[r, c]; // equals trace(R_target^T * R_sol)
        var cosTheta = Math.Clamp((tr - 1.0) / 2.0, -1.0, 1.0);
        return Math.Acos(cosTheta);
    }
}

