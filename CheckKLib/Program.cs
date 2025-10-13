using System;
using System.Linq;
using KinematicsRM4;

class Program
{
    static void Main()
    {
        Console.WriteLine("--- Kinematics Final Check ---");

        var robot = ManipulatorFactory.CreateManipulator("MiRobot");

        var q_target = new[] { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6 };
        Console.WriteLine($"\n1. Target Joint Angles (deg): [{string.Join(", ", q_target.Select(r => r * 180 / Math.PI).Select(d => d.ToString("F3")))}]");

        var T_target = robot.Forward(q_target);
        Console.WriteLine("\n2. Target Cartesian Pose (matrix): ");
        PrintMatrix(T_target);

        Console.WriteLine("\n3. Solving with InverseJacobian()...");
        var q_guess = new double[] { 0, 0, 0, 0, 0, 0 };
        var ik_sol = robot.InverseJacobian(T_target, q_guess, out bool success);

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
        for (int i = 0; i < 3; i++)
        {
            double diff = T_target[i, 3] - T_check[i, 3];
            pos_error += diff * diff;
        }
        pos_error = Math.Sqrt(pos_error);

        double orientation_error = 0;
        for(int i=0; i<3; i++) for(int j=0; j<3; j++)
        {
            double diff = T_target[i,j] - T_check[i,j];
            orientation_error += diff*diff;
        }
        orientation_error = Math.Sqrt(orientation_error);

        Console.WriteLine($"\n   Pose Error (Position Norm): {pos_error:F6}");
        Console.WriteLine($"   Pose Error (Orientation Matrix Norm): {orientation_error:F6}");

        double tolerance = 1e-3;
        if (pos_error < tolerance && orientation_error < tolerance)
        {
            Console.WriteLine("\nCHECK PASSED");
        }
        else
        {
            Console.WriteLine("\nCHECK FAILED");
        }
    }

    static void PrintMatrix(double[,] M) { for (int r=0;r<4;r++) Console.WriteLine($"  [{M[r,0],8:F3},{M[r,1],8:F3},{M[r,2],8:F3},{M[r,3],8:F3}]"); }
}
