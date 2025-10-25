using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using KHelper;

namespace KinematicsCM2;
public class Kinematics
{
    private MdhParameter[] _mdhParameters;

    public Kinematics()
    {
        // 幾何学的に簡潔なMDHパラメータを定義
        _mdhParameters =
        [
            // i-1, alpha, a, d, thetaOffset
            new MdhParameter { Alpha = 0,           A = 0,      D = 127.0,  ThetaOffset = 0 }, // Joint 1
            new MdhParameter { Alpha = Math.PI / 2, A = 29.69,  D = 0,      ThetaOffset = Math.PI / 2 }, // Joint 2
            new MdhParameter { Alpha = 0,           A = 108.0,  D = 0,      ThetaOffset = -Math.PI / 2 }, // Joint 3
            new MdhParameter { Alpha = Math.PI / 2, A = 0,      D = -20.0,  ThetaOffset = 0 }, // Joint 4 (90degree crank)
            new MdhParameter { Alpha = -Math.PI / 2,A = 168.98, D = 0,      ThetaOffset = 0 }, // Joint 5
            new MdhParameter { Alpha = Math.PI / 2, A = 0,      D = 0,      ThetaOffset = 0 }, // Joint 6
            new MdhParameter { Alpha = 0,           A = 0,      D = 24.29,  ThetaOffset = 0 }  // End Effector (Fixed)
        ];
    }

    // FK
    // jointAngles: (0, 0, 0, 0, 0, 0)
    public Matrix4x4 Forward(double[] jointAngles)
    {
        if (jointAngles.Length != 6)
        {
            throw new ArgumentException("Forward kinematics need six angles");
        }

        var T = Matrix4x4.Identity;

        // adopt joint angles from 0(base) to 5
        for (var i = 0; i < 6; i++)
        {
            // add angles to offset
            var mdhTheta = jointAngles[i] + _mdhParameters[i].ThetaOffset;

            // 変換行列の積を計算 (RotX -> TransX -> RotZ -> TransZ の順)
            var Ti = Matrix4x4Extension.CreateRotationX(_mdhParameters[i].Alpha) *
                           Matrix4x4Extension.CreateTranslationX(_mdhParameters[i].A) *
                           Matrix4x4Extension.CreateRotationZ(mdhTheta) *
                           Matrix4x4Extension.CreateTranslationZ(_mdhParameters[i].D);

            T *= Ti;
        }

        // 最後にツール（エンドエフェクタ）の固定変換を適用
        // 90度クランクの仮想ジョイントはMDH表のi=3に統合されている
        var T_tool = Matrix4x4Extension.CreateRotationX(_mdhParameters[6].Alpha) *
                           Matrix4x4Extension.CreateTranslationX(_mdhParameters[6].A) *
                           Matrix4x4Extension.CreateRotationZ(jointAngles[5] + _mdhParameters[6].ThetaOffset) *
                           Matrix4x4Extension.CreateTranslationZ(_mdhParameters[6].D);
        T *= T_tool;

        return T;
    }

    public MdhParameter[] GetMdhParameters() => _mdhParameters;
}

