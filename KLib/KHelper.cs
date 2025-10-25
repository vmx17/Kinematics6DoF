using System;
using System.Numerics;

namespace KHelper;
public struct MdhParameter
{
    public double Alpha
    {
        get; set;
    }
    public double A
    {
        get; set;
    }
    public double D
    {
        get; set;
    }
    public double ThetaOffset
    {
        get; set;
    } // 物理角度とMDH角度のオフセット
}
// MDH変換行列を生成する拡張メソッド 列優先（Column-Major）
public static class Matrix4x4Extension
{
    // RotX(alpha)
    public static Matrix4x4 CreateRotationX(double alpha)
    {
        var cos = (float)Math.Cos(alpha);
        var sin = (float)Math.Sin(alpha);
        return new Matrix4x4(
            1, 0, 0, 0,
            0, cos, -sin, 0,
            0, sin, cos, 0,
            0, 0, 0, 1
        );
    }

    // TransX(a)
    public static Matrix4x4 CreateTranslationX(double a)
    {
        return new Matrix4x4(
            1, 0, 0, (float)a,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
        );
    }

    // RotZ(theta)
    public static Matrix4x4 CreateRotationZ(double theta)
    {
        var cos = (float)Math.Cos(theta);
        var sin = (float)Math.Sin(theta);
        return new Matrix4x4(
            cos, -sin, 0, 0,
            sin, cos, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
        );
    }

    // TransZ(d)
    public static Matrix4x4 CreateTranslationZ(double d)
    {
        return new Matrix4x4(
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, (float)d,
            0, 0, 0, 1
        );
    }
}
