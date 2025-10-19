using System;
using System.Linq;
using KinematicsRM2;

class Program
{
    // Main test routine
    private static void Main()
    {
        Console.WriteLine("--- Kinematics Final Check ---\n");

        // Run the original hard-coded test case first
        Console.WriteLine("--- Running Original Hard-coded Test Case ---");
        var halfPi = Math.PI / 2;
        var q_target_orig = new[] { 0.1 * halfPi, 0.0, 0.0, 0.0, 0.0, 0.0 };
        var q_guess_orig = new double[] { 0, 0, 0, halfPi * 0.8, 0, 0 };
        bool original_test_passed = RunSingleTest(q_target_orig, q_guess_orig, 1);
        Console.WriteLine($"--- Original Test Result: {(original_test_passed ? "PASSED" : "FAILED")} ---\n");


        // Now run multiple random test cases
        Console.WriteLine("--- Running 5 Random Test Cases ---");
        var rand = new Random();
        int successCount = 0;
        int testCount = 5;

        // Joint limits in degrees (from KinematicsRM2.cs)
        var minAnglesDeg = new double[] { -179, -179, -89, -179, -179, -359 };
        var maxAnglesDeg = new double[] { 180, 90, 180, 180, 90, 360 };

        for (int i = 0; i < testCount; i++)
        {
            Console.WriteLine($"\n--- Test Case {i + 1}/{testCount} ---");
            var q_target_rand = new double[6];
            var q_guess_rand = new double[6];

            for (int j = 0; j < 6; j++)
            {
                // Generate random angles within limits
                q_target_rand[j] = (minAnglesDeg[j] + rand.NextDouble() * (maxAnglesDeg[j] - minAnglesDeg[j])) * Math.PI / 180.0;
                q_guess_rand[j] = (minAnglesDeg[j] + rand.NextDouble() * (maxAnglesDeg[j] - minAnglesDeg[j])) * Math.PI / 180.0;
            }

            if (RunSingleTest(q_target_rand, q_guess_rand, i + 2))
            {
                successCount++;
            }
        }

        Console.WriteLine($"\n--- Random Test Summary ---");
        Console.WriteLine($"{successCount} out of {testCount} random tests passed.");
    }

    // Extracted test logic to run for a single target/guess pair
    private static bool RunSingleTest(double[] q_target, double[] q_guess, int testId)
    {
        var robot = ManipulatorFactory.CreateManipulator("MiRobot", "TOOL0");

        Console.WriteLine($"1. Target Joint Angles (deg): [{string.Join(", ", q_target.Select(r => r * 180 / Math.PI).Select(d => d.ToString("F4")))}]");

        var T_target = robot.Forward(q_target, verbose: 0); // verbose set to 0 for cleaner random test logs
        Console.WriteLine("\n2. Target Cartesian Pose (matrix): ");
        PrintMatrix(T_target);

        Console.WriteLine("\n3. Solving with InverseJacobian()...");
        Console.WriteLine($"   Initial guess (deg): [{string.Join(", ", q_guess.Select(r => r * 180 / Math.PI).Select(d => d.ToString("F4")))}]");
        var ik_sol = robot.InverseJacobian(T_target, q_guess, out var success, maxIterations: 300, verbose: 0); // verbose set to 0 for less noise

        if (!success || ik_sol == null)
        {
            Console.WriteLine("   Jacobian method failed to find a solution.");
            Console.WriteLine("\nCHECK FAILED");
            return false;
        }
        Console.WriteLine($"   Found solution (deg): [{string.Join(", ", ik_sol.Select(r => r * 180 / Math.PI).Select(d => d.ToString("F4")))}]");

        Console.WriteLine("\n4. Verifying by feeding IK solution back into FK...");
        var T_check = robot.Forward(ik_sol);
        
        double pos_error = 0;
        for (var i = 0; i < 3; i++)
        {
            var diff = T_target[i, 3] - T_check[i, 3];
            pos_error += diff * diff;
        }
        pos_error = Math.Sqrt(pos_error);

        double orientation_error = RotationAngle(T_target, T_check); // Use more robust angle error

        Console.WriteLine($"\n   Pose Error (Position Norm): {pos_error:F6} mm");
        Console.WriteLine($"   Pose Error (Orientation Angle): {orientation_error * 180.0 / Math.PI:F6} deg");

        var pos_tolerance = 1e-3;
        var ori_tolerance_rad = 1e-3;
        if (pos_error < pos_tolerance && orientation_error < ori_tolerance_rad)
        {
            Console.WriteLine("\nCHECK PASSED");
            return true;
        }
        else
        {
            Console.WriteLine("\nCHECK FAILED");
            return false;
        }
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