using System;
using System.Linq;
using System.Numerics;
using KinematicsCM2;

class Program
{
    private static void Main()
    {
        Console.WriteLine("--- Kinematics CM2 ---");
        var r = new Kinematics();
        var angles = new double[] { 0, 0, 0, 0, 0, 0 };
        var T = r.Forward(angles);
        PrintMatrix(T);
    }
    private static void PrintMatrix(Matrix4x4 M)
    {
        for (var r = 0; r < 4; r++)
            Console.WriteLine($"  [{M[r, 0],8:F3},{M[r, 1],8:F3},{M[r, 2],8:F3},{M[r, 3],8:F3}]");
    }
}
