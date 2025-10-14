using System;
using System.Linq;
using KinematicsRM2;

class Program
{
    static void Main()
    {
        Console.WriteLine("--- Kinematics Final Check ---");

        var robot = ManipulatorFactory.CreateManipulator("MiRobot");

        var q_target = new[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        Console.WriteLine($"\n1. Target Joint Angles (deg): [{string.Join(", ", q_target.Select(r => r * 180 / Math.PI).Select(d => d.ToString("F3")))}]");

        var T_target = robot.Forward(q_target, verbose:2);
        Console.WriteLine("\n2. Target Cartesian Pose (matrix): ");
        PrintMatrix(T_target);

        Console.WriteLine("\n3. Solving with InverseJacobian()...");
        var q_guess = new double[] { 0, 0, 0, 0, 0, 0 };
        // verbose=1 to show Pw
        var ik_sol = robot.InverseJacobian(T_target, q_guess, out var success, verbose:1);

        if (!success || ik_sol == null)
        {
            Console.WriteLine("   Jacobian method failed to find a solution.");
            Console.WriteLine("\nCHECK FAILED");
            return;
        }
        Console.WriteLine($"   Found solution (deg): [{string.Join(", ", ik_sol.Select(r => r * 180 / Math.PI).Select(d => d.ToString("F3")))}]");


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

        Console.WriteLine($"\n   Pose Error (Position Norm): {pos_error:F6}");
        Console.WriteLine($"   Pose Error (Orientation Matrix Norm): {orientation_error:F6}");

        var tolerance = 1e-3;
        if (pos_error < tolerance && orientation_error < tolerance)
        {
            Console.WriteLine("\nForward() - InverseJacobian(): CHECK PASSED");
        }
        else
        {
            Console.WriteLine("\nForward() - InverseJacobian(): CHECK FAILED");
        }

        // ------------------------------------------------------------------
        // 5. Geometric IK: enumerate all feasible solutions for same T_target
        // ------------------------------------------------------------------
        Console.WriteLine("\n5. Solving with InverseGeometric() (enumerating all solutions)...");
        // verbose=1 to show Pw
        var geo_solutions = robot.InverseGeometric(T_target, verbose:1);
        Console.WriteLine($"   Number of geometric IK solutions: {geo_solutions.Length}");

        for (int i = 0; i < geo_solutions.Length; i++)
        {
            var qs = geo_solutions[i];
            var degStr = string.Join(", ", qs.Select(r => (r * 180 / Math.PI).ToString("F3")));
            Console.WriteLine($"   [{i}] (deg): [{degStr}]");
        }

        int matchedIndex = -1;
        if (geo_solutions.Length > 0)
        {
            for (int i = 0; i < geo_solutions.Length && matchedIndex < 0; i++)
            {
                var g = geo_solutions[i];
                bool allClose = true;
                for (int k = 0; k < 6; k++)
                {
                    if (Math.Abs(NormalizeAngle(g[k] - ik_sol[k])) > 1e-4) { allClose = false; break; }
                }
                if (allClose) matchedIndex = i;
            }
        }

        if (matchedIndex >= 0)
            Console.WriteLine($"   Jacobian solution matches geometric solution index [{matchedIndex}].");
        else
            Console.WriteLine("   Jacobian solution NOT found in geometric solution set.");

        for (int i = 0; i < geo_solutions.Length; i++)
        {
            var T_g = robot.Forward(geo_solutions[i]);
            double pe = 0;
            for (int r = 0; r < 3; r++)
            {
                var d = T_target[r, 3] - T_g[r, 3];
                pe += d * d;
            }
            pe = Math.Sqrt(pe);
            if (pe > 1e-3)
            {
                Console.WriteLine($"   Warning: solution [{i}] position error {pe:E3} exceeds tolerance.");
            }
        }
    }

    static double NormalizeAngle(double a)
    {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }

    static void PrintMatrix(double[,] M)
    {
        for (var r = 0; r < 4; r++)
            Console.WriteLine($"  [{M[r, 0],8:F3},{M[r, 1],8:F3},{M[r, 2],8:F3},{M[r, 3],8:F3}]");
    }
}

