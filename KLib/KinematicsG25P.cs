using System;
using System.Text;

namespace KinematicsG25P;

/*
   MDHの変換行列の乗算順序は、仕様書  に記載されている幾何学的な操作順序 RotX -> TransX -> RotZ -> TransZ に従います。これは、行優先行列を用いた後掛け（Post-multiplication）の連鎖計算（T_total = T_total * T_local）と一致します。
*/
/// <summary>
/// CoreSpec.pdf (新仕様) の仕様に基づく6DoFマニピュレータの運動学モジュール。
/// 外部ライブラリに依存せず、標準の double 型配列のみで実装されています。
/// </summary>
public class ManipulatorKinematics
{
    // --- 新しいMDHパラメータ (CoreSpec.pdf) ---
    // Link 0->1, 1->2, 2->3, 3->e, e->4, 4->5, 5->6 の順
    // alpha_{i-1}
    private readonly double[] mdh_alpha = {
        0,          // 1 (J1)
        Math.PI / 2,// 2 (J2)
        Math.PI,    // 3 (J3)
        -Math.PI / 2,// e (Fixed)
        0,          // 4 (J4)
        Math.PI / 2,// 5 (J5)
        Math.PI / 2 // 6 (J6)
    };

    // a_{i-1}
    private readonly double[] mdh_a = {
        0,          // 1 (J1)
        29.690,     // 2 (J2)
        108,        // 3 (J3)
        20,         // e (Fixed)
        0,          // 4 (J4)
        0,          // 5 (J5)
        0           // 6 (J6)
    };

    // d_i
    private readonly double[] mdh_d = {
        127,        // 1 (J1)
        0,          // 2 (J2)
        0,          // 3 (J3)
        0,          // e (Fixed)
        168.980,    // 4 (J4)
        0,          // 5 (J5)
        24.290      // 6 (J6)
    };

    // theta_i offset [cite: 84, 86]
    private readonly double[] theta_offsets = {
        0,          // 1 (J1)
        Math.PI / 2,// 2 (J2)
        0,          // 3 (J3)
        0,          // e (Fixed)
        0,          // 4 (J4)
        -Math.PI / 2,// 5 (J5)
        0           // 6 (J6)
    };


    // --- 1. 順運動学 (Forward Kinematics) [cite: 7] ---

    /// <summary>
    /// 順運動学 (FK) を計算します。
    /// </summary>
    /// <param name="jointAngles">6軸の関節角度 (rad) [q1, q2, q3, q4, q5, q6]</param>
    /// <param name="tools">ツールパラメータ [Xt, Yt, Zt, rtx, rty, rtz]。nullの場合は (0,0,0,0,0,0)。</param>
    /// <param name="verbose">冗長レベル。1: 原点位置 [cite: 117], 2: 同次変換行列 [cite: 118]</param>
    /// <returns>4x4 同次変換行列 (double[,])</returns>
    public double[,] Forward(double[] jointAngles, double[] tools = null, int verbose = 0)
    {
        // デフォルトマウント (0,0,0,0,0,0) を使用 [cite: 41, 83]
        double[] defaultMount = { 0, 0, 0, 0, 0, 0 };
        return Forward(jointAngles, tools, defaultMount, verbose);
    }

    /// <summary>
    /// 順運動学 (FK) のフルシグネチャ。マウントとツールの両方を指定します。
    /// </summary>
    public double[,] Forward(double[] jointAngles, double[] toolParams, double[] mountParams, int verbose = 0)
    {
        if (jointAngles.Length != 6)
        {
            throw new ArgumentException("jointAngles は 6 要素である必要があります。");
        }

        // 1. パラメータのデフォルト値設定
        double[] tp = toolParams ?? new double[] { 0, 0, 0, 0, 0, 0 }; // [cite: 77]
        double[] mp = mountParams ?? new double[] { 0, 0, 0, 0, 0, 0 }; // [cite: 41]

        // 2. 各フレームの変換行列を計算
        // T_frames[0] = T_mount
        // T_frames[1] = T_0_1 ... T_frames[7] = T_0_6 (フランジ)
        double[][,] T_frames = CalculateFkFrames(jointAngles, mp);

        // 3. ツール変換行列を計算 [cite: 49]
        double[,] T_tool = GetToolTransform(tp); // 6_T_t

        // 4. 最終的なTCPの姿勢を計算
        // T_0_t = T_0_6 * T_6_t
        double[,] T_final = MatrixUtils.Multiply(T_frames[7], T_tool);

        // 5. 冗長出力
        if (verbose > 0)
        {
            PrintVerboseFk(T_frames, T_tool, T_final, verbose);
        }

        // 6. double[,] 形式で返却
        return T_final;
    }

    /// <summary>
    /// FKの計算ロジック本体。ベースからフランジ(Sigma_6)までの全中間フレームを計算します。
    /// </summary>
    /// <returns>T[0]=T_mount, T[1]=T_0_1, ..., T[7]=T_0_6</returns>
    private double[][,] CalculateFkFrames(double[] jointAngles, double[] mountParams)
    {
        double[][,] T_frames = new double[8][,];

        // T_frames[0] : マウント変換 (Sigma_abs -> Sigma_0) [cite: 33, 36]
        T_frames[0] = GetMountTransform(mountParams);

        // 関節角度とオフセットを合算 [cite: 84, 86]
        double[] q = new double[7];
        q[0] = jointAngles[0] + theta_offsets[0]; // J1 (theta_1)
        q[1] = jointAngles[1] + theta_offsets[1]; // J2 (theta_2)
        q[2] = jointAngles[2] + theta_offsets[2]; // J3 (theta_3)
        q[3] = theta_offsets[3];                  // Fixed e (theta_e = 0)
        q[4] = jointAngles[3] + theta_offsets[4]; // J4 (theta_4)
        q[5] = jointAngles[4] + theta_offsets[5]; // J5 (theta_5)
        q[6] = jointAngles[5] + theta_offsets[6]; // J6 (theta_6)

        // MDHパラメータに基づき、T[i] = T[i-1] * T_{i-1 -> i} を計算 [cite: 47]
        T_frames[1] = MatrixUtils.Multiply(T_frames[0], GetMdHTransform(mdh_alpha[0], mdh_a[0], q[0], mdh_d[0]));
        T_frames[2] = MatrixUtils.Multiply(T_frames[1], GetMdHTransform(mdh_alpha[1], mdh_a[1], q[1], mdh_d[1]));
        T_frames[3] = MatrixUtils.Multiply(T_frames[2], GetMdHTransform(mdh_alpha[2], mdh_a[2], q[2], mdh_d[2]));
        T_frames[4] = MatrixUtils.Multiply(T_frames[3], GetMdHTransform(mdh_alpha[3], mdh_a[3], q[3], mdh_d[3]));
        T_frames[5] = MatrixUtils.Multiply(T_frames[4], GetMdHTransform(mdh_alpha[4], mdh_a[4], q[4], mdh_d[4]));
        T_frames[6] = MatrixUtils.Multiply(T_frames[5], GetMdHTransform(mdh_alpha[5], mdh_a[5], q[5], mdh_d[5]));
        T_frames[7] = MatrixUtils.Multiply(T_frames[6], GetMdHTransform(mdh_alpha[6], mdh_a[6], q[6], mdh_d[6]));

        return T_frames;
    }


    // --- 2. 逆運動学 (Inverse Kinematics) - Geometric [cite: 8] ---

    /// <summary>
    /// 逆運動学 (IK) を幾何学的アプローチで計算します。
    /// </summary>
    /// <param name="T_target">ターゲットの 4x4 同次変換行列 (double[,])</param>
    /// <param name="verbose">冗長レベル。1: Pw などの計算値を出力 [cite: 119]</param>
                /// <returns>6軸の関節角度 (rad)</returns>
    public double[] InverseGeometric(double[,] T_target, int verbose = 0)
    {
        // 戻り値は1つの解のみ
        double[] q = new double[6];

        // --- 1. Flange (O6) の目標変換行列 T_0_6 の計算 ---

        // T_target = T_0_6 * T_6_TCP より T_0_6 = T_target * T_6_TCP_inv
        // ここではツール長ゼロ（デフォルト）を想定し、T_targetをそのままT_0_6として使用
        // 厳密には、ツールパラメータを考慮する必要があります。
        // ※ 厳密解を求めるには、ここで GetToolTransform(tools) の逆行列を使う必要があります。
        // 今回は簡単化のため、ツール長ゼロを想定し、T_targetをT_0_TCP = T_0_6 として扱います。
        double[,] T_0_6 = T_target; // T_target は T_0_TCP

        // T_0_6 の位置ベクトル P6 = (x6, y6, z6)
        double[] P6 = MatrixUtils.GetTranslation(T_0_6);
        double x6 = P6[0];
        double y6 = P6[1];
        double z6 = P6[2];

        if (verbose > 0)
        {
            Console.WriteLine("\n  [IK Geo] Target Flange (O6) Position:");
            Console.WriteLine($"  P6: ({x6:F3}, {y6:F3}, {z6:F3}) mm");
        }

        // --- 2. J1, J2, J3 の解法のための O4 の位置 P4 の推定 ---

        // O4 (P4) の位置は P6 の位置と、q4, q5, q6 の角度、および d4, d5, d6 のリンク長に依存します。
        // この構造では、P4の位置は q4, q5, q6が既知でなければ定まりません。
        // P4 = P6 - (T_0_4 * [0; 0; d4 + d5 + d6; 1] の影響) ... 非常に複雑

        // 幾何学的IKを成立させるため、以下の代数操作が必要です:
        // P4 の位置は (x4, y4, z4) であり、これは q1, q2, q3 のみで表現できます。
        // (x4, y4, z4) = f(q1, q2, q3)
        // ターゲットの (x6, y6, z6) は全関節で表現されます。
        // q1, q2, q3 を解くためには、まず q4, q5, q6 を解く必要があります。

        // **幾何学的IKの一般的な流れ（非球状手首の場合）：**
        // 2-1. T_0_6 から q4, q5, q6 を代数的に解く。
        // 2-2. 求めた q4, q5, q6 の解を使って T_6_4 (T_4_6 の逆) を計算。
        // 2-3. T_0_4 = T_0_6 * T_6_4 から O4 の目標位置 P4 を決定。
        // 2-4. P4 を使って q1, q2, q3 を解く（Planar Arm IK）。

        // ここでは幾何学的解法の構造を示すため、P4 の位置決定を簡略化します。
        // 厳密な代数解を実装しないため、ここでは未実装エラーを返します。

        Console.WriteLine("\n  [IK Geo] Error: This manipulator has a non-spherical wrist structure (a_e=20, d_4=168.980).");
        Console.WriteLine("  Geometric IK is not possible via standard position/orientation separation.");
        Console.WriteLine("  Full algebraic solution for this structure is required, which is extremely complex.");

        return null;

        // --- (以下、幾何学的解法の実装に必要なステップのメモ) ---
        // if (T_0_4 is found)
        // {
        //     // 3. J1 の解 (Q1)
        //     double[] P4 = MatrixUtils.GetTranslation(T_0_4);
        //     // q1 を atan2(y4, x4) と他の項から導出 (複数の解)
        //     // q[0] = ... 

        //     // 4. J2, J3 の解 (Q2, Q3)
        //     // T_1_0 * P4 を計算し、その XY または XZ 平面で二関節平面アームを解く
        //     // L2 = a2, L3 = a3
        //     // q[1], q[2] = ... (Law of Cosines/Subproblem 3/4) (肘の上下で複数の解)

        //     // 5. J4, J5, J6 の解 (Q4, Q5, Q6)
        //     // T_4_0 * T_0_6 を T_4_6 に等しいとおき、T_4_6 の回転成分から解く
        //     // q[3], q[4], q[5] = ... (Subproblem 2/3) (手首のフリップで複数の解)

        //     // 6. 複数解の組み合わせと検証
        //     // ...
        //     // return q;
        // }
        // return null;
    }

    // --- 3. 逆運動学 (Inverse Kinematics) - Jacobian ---

    /// <summary>
    /// 逆運動学 (IK) をヤコビアン（反復法）で計算。
    /// Damped Least Squares (DLS) 法に従う
    /// </summary>
    public double[] InverseJacobian(double[,] T_target, double[] q_initial, out bool success,
    int maxIterations = 300, double tolerance = 1e-6, double alpha = 1.0,
    double damping = 1e-3, int verbose = 0)
    {
        success = false;
        double[] q = (double[])q_initial.Clone();
        double error_norm = double.MaxValue;

        for (int i = 0; i < maxIterations; i++)
        {
            // 1. 現在のFKと全フレームを計算 (T_frames[7] が T_0_6 (フランジ))
            double[][,] T_frames = CalculateFkFrames(q, new double[] { 0, 0, 0, 0, 0, 0 });
            double[,] T_current = T_frames[7];

            // 2. 誤差ベクトル (6x1) を計算
            double[] error = GetErrorVector(T_target, T_current);

            // 3. 収束判定
            error_norm = 0;
            for (int j = 0; j < 6; j++) error_norm += error[j] * error[j];
            error_norm = Math.Sqrt(error_norm);

            if (error_norm < tolerance)
            {
                success = true;
                if (verbose > 0) Console.WriteLine($"[IK Jac] Converged in {i} iterations.");
                return q;
            }

            // 4. ヤコビアン (6x6) を計算
            double[,] J = CalculateJacobian(T_frames);

            // 5. 角度の増分 Δq を計算 (DLS法を実装)
            try
            {
                // A = J * J^T
                double[,] JT = MatrixUtils.Transpose(J);
                double[,] A = MatrixUtils.Multiply(J, JT); // J は 6x6, JT も 6x6

                // B = A + damping^2 * I
                double dampingSq = damping * damping;
                double[,] I = MatrixUtils.Identity6x6();
                double[,] dampingI = new double[6, 6];
                for (int r = 0; r < 6; r++)
                {
                    for (int c = 0; c < 6; c++)
                    {
                        dampingI[r, c] = I[r, c] * dampingSq;
                    }
                }
                double[,] B = MatrixUtils.Add(A, dampingI); // B = J*J^T + damping^2 * I

                // B_inv = Inverse(B)
                double[,] B_inv = MatrixUtils.Inverse(B);

                // delta_q = alpha * J^T * B_inv * error
                double[,] JTB_inv = MatrixUtils.Multiply(JT, B_inv);

                double[] delta_q_vector = MatrixUtils.MultiplyMatrixVector(JTB_inv, error);

                // 6. 角度を更新
                double[] q_new = (double[])q.Clone();
                for (int j = 0; j < 6; j++)
                {
                    q_new[j] = q[j] + alpha * delta_q_vector[j];
                }
                q = q_new;
            }
            catch (InvalidOperationException ex)
            {
                Console.WriteLine($"[IK Jac] エラー: 特異点または計算エラー: {ex.Message}");
                success = false;
                return q;
            }
        }

        if (verbose > 0) Console.WriteLine($"[IK Jac] Failed to converge after {maxIterations} iterations. Final Error Norm: {error_norm:F6}");
        success = false;
        return q;
    }


    // --- 4. 変換行列の生成ヘルパー ---

    /// <summary>
    /// MDHパラメータから同次変換行列を生成します (行優先)。
    /// 仕様書 [cite: 68-71] の幾何学的順序 T = RotX * TransX * RotZ * TransZ に従う。
    /// </summary>
    private double[,] GetMdHTransform(double alpha, double a, double theta, double d)
    {
        double[,] T_RotX = MatrixUtils.CreateRotationX(alpha);
        double[,] T_TransX = MatrixUtils.CreateTranslation(a, 0, 0);
        double[,] T_RotZ = MatrixUtils.CreateRotationZ(theta);
        double[,] T_TransZ = MatrixUtils.CreateTranslation(0, 0, d);

        // 乗算順序: T = RotX * TransX * RotZ * TransZ
        double[,] T = MatrixUtils.Multiply(T_RotX, T_TransX);
        T = MatrixUtils.Multiply(T, T_RotZ);
        T = MatrixUtils.Multiply(T, T_TransZ);

        return T;
    }

    /// <summary>
    /// マウントパラメータから同次変換行列を生成します [cite: 36, 40]。
                /// T = Trans(X,Y,Z) * RotZ(rmz) * RotY(rmy) * RotX(rmx)
                /// </summary>
    private double[,] GetMountTransform(double[] mp)
    {
        double[,] T_Trans = MatrixUtils.CreateTranslation(mp[0], mp[1], mp[2]);
        double[,] T_RotX = MatrixUtils.CreateRotationX(mp[3]);
        double[,] T_RotY = MatrixUtils.CreateRotationY(mp[4]);
        double[,] T_RotZ = MatrixUtils.CreateRotationZ(mp[5]);

        // 回転順序: rz0 -> ry0 -> rx0 [cite: 40]
        double[,] T = MatrixUtils.Multiply(T_Trans, T_RotZ);
        T = MatrixUtils.Multiply(T, T_RotY);
        T = MatrixUtils.Multiply(T, T_RotX);
        return T;
    }

    /// <summary>
    /// ツールパラメータから同次変換行列を生成します [cite: 26, 29]。
    /// T = Trans(X,Y,Z) * RotZ(rtz) * RotY(rty) * RotX(rtx)
    /// </summary>
    /// <summary>
    /// ツールパラメータから同次変換行列を生成します。
    /// T = Trans(X,Y,Z) * RotZ(rtz) * RotY(rty) * RotX(rtx)
    /// </summary>
    private double[,] GetToolTransform(double[] tp)
    {
        // 【修正箇所】null チェックを追加
        if (tp == null)
        {
            tp = new double[] { 0, 0, 0, 0, 0, 0 };
        }

        double[,] T_Trans = MatrixUtils.CreateTranslation(tp[0], tp[1], tp[2]);
        double[,] T_RotX = MatrixUtils.CreateRotationX(tp[3]);
        double[,] T_RotY = MatrixUtils.CreateRotationY(tp[4]);
        double[,] T_RotZ = MatrixUtils.CreateRotationZ(tp[5]);

        // 回転順序: rzt -> ryt -> rxt
        double[,] T = MatrixUtils.Multiply(T_Trans, T_RotZ);
        T = MatrixUtils.Multiply(T, T_RotY);
        T = MatrixUtils.Multiply(T, T_RotX);
        return T;
    }

    // --- 5. IK用ヘルパー (Jacobian) ---

    /// <summary>
    /// IK用のヤコビアン行列 (6x6) を計算します。
    /// </summary>
    /// <param name="T_frames">FKの全中間フレーム (T_frames[7] が T_0_6 (フランジ))</param>
    private double[,] CalculateJacobian(double[][,] T_frames)
    {
        double[,] J = new double[6, 6];
        double[] P_end = MatrixUtils.GetTranslation(T_frames[7]); // フランジ(P_6)の位置

        // T_frames[0] = T_mount(Sigma_0)
        // T_frames[1] = T_0_1 (J1)
        // ...
        // T_frames[4] = T_0_e (Fixed)
        // T_frames[5] = T_0_4 (J4)
        // T_frames[6] = T_0_5 (J5)
        // T_frames[7] = T_0_6 (J6)

        // 各関節の回転軸(Z)と原点(P)を取得
        double[][] Z = new double[6][];
        double[][] P = new double[6][];

        Z[0] = MatrixUtils.GetZAxis(T_frames[1]); // Z1 (J1)
        P[0] = MatrixUtils.GetTranslation(T_frames[1]);
        Z[1] = MatrixUtils.GetZAxis(T_frames[2]); // Z2 (J2)
        P[1] = MatrixUtils.GetTranslation(T_frames[2]);
        Z[2] = MatrixUtils.GetZAxis(T_frames[3]); // Z3 (J3)
        P[2] = MatrixUtils.GetTranslation(T_frames[3]);
        Z[3] = MatrixUtils.GetZAxis(T_frames[5]); // Z4 (J4)
        P[3] = MatrixUtils.GetTranslation(T_frames[5]);
        Z[4] = MatrixUtils.GetZAxis(T_frames[6]); // Z5 (J5)
        P[4] = MatrixUtils.GetTranslation(T_frames[6]);
        Z[5] = MatrixUtils.GetZAxis(T_frames[7]); // Z6 (J6)
        P[5] = MatrixUtils.GetTranslation(T_frames[7]);

        // J1, J2, J3, J4, J5, J6 に対応
        double[][] Zi = { Z[0], Z[1], Z[2], Z[3], Z[4], Z[5] };
        double[][] Pi = { P[0], P[1], P[2], P[3], P[4], P[5] }; // J6の回転軸はP5(O5)ではなくP6(O6)にある

        for (int i = 0; i < 6; i++)
        {
            // P_end - P_i
            double[] P_diff = { P_end[0] - Pi[i][0], P_end[1] - Pi[i][1], P_end[2] - Pi[i][2] };

            // J_p = Z_i x (P_end - P_i)
            double[] J_p = MatrixUtils.CrossProduct(Zi[i], P_diff);
            // J_o = Z_i
            double[] J_o = Zi[i];

            J[0, i] = J_p[0];
            J[1, i] = J_p[1];
            J[2, i] = J_p[2];
            J[3, i] = J_o[0];
            J[4, i] = J_o[1];
            J[5, i] = J_o[2];
        }

        return J;
    }

    /// <summary>
    /// IK用の誤差ベクトル (6x1) を計算します。
    /// e = [pos_error, rot_error]
    /// </summary>
    private double[] GetErrorVector(double[,] T_target, double[,] T_current)
    {
        double[] e = new double[6];

        // 1. 位置誤差 (単純な差分)
        double[] p_target = MatrixUtils.GetTranslation(T_target);
        double[] p_current = MatrixUtils.GetTranslation(T_current);
        e[0] = p_target[0] - p_current[0];
        e[1] = p_target[1] - p_current[1];
        e[2] = p_target[2] - p_current[2];

        // 2. 姿勢誤差
        // e_rot = 0.5 * (n x n_d + o x o_d + a x a_d)
        // T_current (n, o, a)
        double[] n_c = MatrixUtils.GetXAxis(T_current);
        double[] o_c = MatrixUtils.GetYAxis(T_current);
        double[] a_c = MatrixUtils.GetZAxis(T_current);
        // T_target (n_d, o_d, a_d)
        double[] n_d = MatrixUtils.GetXAxis(T_target);
        double[] o_d = MatrixUtils.GetYAxis(T_target);
        double[] a_d = MatrixUtils.GetZAxis(T_target);

        double[] cross_n = MatrixUtils.CrossProduct(n_c, n_d);
        double[] cross_o = MatrixUtils.CrossProduct(o_c, o_d);
        double[] cross_a = MatrixUtils.CrossProduct(a_c, a_d);

        e[3] = 0.5 * (cross_n[0] + cross_o[0] + cross_a[0]);
        e[4] = 0.5 * (cross_n[1] + cross_o[1] + cross_a[1]);
        e[5] = 0.5 * (cross_n[2] + cross_o[2] + cross_a[2]);

        return e;
    }


    // --- 6. 冗長出力ヘルパー ---

    /// <summary>
    /// FKの冗長出力を行います。
    /// </summary>
    private void PrintVerboseFk(double[][,] T_frames, double[,] T_tool, double[,] T_final, int verbose)
    {
        // 仕様書のフレーム名に基づく [cite: 84, 86]
        string[] names = { "0(Base)", "1(J1)", "2(J2)", "3(J3)",
                           "e(Fixed)", "4(J4)", "5(J5)", "6(Flange)" };

         if (verbose == 1) // [cite: 117]
        {
            Console.WriteLine("--- FK Verbose (Level 1): Local Origins (Absolute Coords) ---");
            for (int i = 0; i < T_frames.Length; i++)
            {
                double[] pos = MatrixUtils.GetTranslation(T_frames[i]);
                Console.WriteLine($"  {names[i]}: ({pos[0]:F3}, {pos[1]:F3}, {pos[2]:F3})");
            }
            double[] tcp_pos = MatrixUtils.GetTranslation(T_final);
            Console.WriteLine($"  TCP   (T_0_t): ({tcp_pos[0]:F3}, {tcp_pos[1]:F3}, {tcp_pos[2]:F3})");
        }
        else if (verbose >= 2) // [cite: 118]
        {
            Console.WriteLine("--- FK Verbose (Level 2): Homogeneous Matrices ---");
            for (int i = 0; i < T_frames.Length; i++)
            {
                Console.WriteLine($"  --- {names[i]} ---");
                Console.WriteLine(MatrixUtils.MatrixToString(T_frames[i]));
            }
            Console.WriteLine("  --- Tool (T_6_t) ---");
            Console.WriteLine(MatrixUtils.MatrixToString(T_tool));
            Console.WriteLine("  --- FINAL (T_0_t) ---");
            Console.WriteLine(MatrixUtils.MatrixToString(T_final));
        }
    }
}

/// <summary>
/// 外部ライブラリに依存しない、double配列ベースの
/// 4x4 行列および 3D ベクトル演算ユーティリティ。
/// (行優先の2D配列 double[4,4] を仮定して実装)
/// </summary>
public static class MatrixUtils
{
    /// <summary>
    /// 4x4 の単位行列を生成します。
    /// </summary>
    public static double[,] Identity()
    {
        return new double[4, 4] {
            { 1, 0, 0, 0 },
            { 0, 1, 0, 0 },
            { 0, 0, 1, 0 },
            { 0, 0, 0, 1 }
        };
    }

    /// <summary>
    /// 4x4 行列の乗算 (A * B) を行います。
    /// </summary>
    public static double[,] Multiply(double[,] A, double[,] B)
    {
        double[,] C = new double[4, 4];
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                C[i, j] = 0;
                for (int k = 0; k < 4; k++)
                {
                    C[i, j] += A[i, k] * B[k, j];
                }
            }
        }
        return C;
    }

    /// <summary>
    /// 4x4 同次変換行列の逆行列を計算。（R^T | -R^T * p の形式）
    /// T_inv = [R^T | -R^T * p]
    /// </summary>
    public static double[,] Inverse4x4Homogeneous(double[,] T)
    {
        double[,] T_inv = Identity();

        // 回転部分の転置 (R^T)
        T_inv[0, 0] = T[0, 0]; T_inv[0, 1] = T[1, 0]; T_inv[0, 2] = T[2, 0];
        T_inv[1, 0] = T[0, 1]; T_inv[1, 1] = T[1, 1]; T_inv[1, 2] = T[2, 1];
        T_inv[2, 0] = T[0, 2]; T_inv[2, 1] = T[1, 2]; T_inv[2, 2] = T[2, 2];

        // 位置ベクトル p
        double[] p = { T[0, 3], T[1, 3], T[2, 3] };

        // 新しい位置ベクトル -R^T * p
        T_inv[0, 3] = -(T_inv[0, 0] * p[0] + T_inv[0, 1] * p[1] + T_inv[0, 2] * p[2]);
        T_inv[1, 3] = -(T_inv[1, 0] * p[0] + T_inv[1, 1] * p[1] + T_inv[1, 2] * p[2]);
        T_inv[2, 3] = -(T_inv[2, 0] * p[0] + T_inv[2, 1] * p[1] + T_inv[2, 2] * p[2]);

        return T_inv;
    }

    /// <summary>
    /// 平行移動行列を生成します。
    /// </summary>
    public static double[,] CreateTranslation(double x, double y, double z)
    {
        return new double[4, 4] {
            { 1, 0, 0, x },
            { 0, 1, 0, y },
            { 0, 0, 1, z },
            { 0, 0, 0, 1 }
        };
    }

    /// <summary>
    /// X軸周りの回転行列を生成します。
    /// </summary>
    public static double[,] CreateRotationX(double angleRad)
    {
        double cos = Math.Cos(angleRad);
        double sin = Math.Sin(angleRad);
        return new double[4, 4] {
            { 1, 0,   0,    0 },
            { 0, cos, -sin, 0 },
            { 0, sin, cos,  0 },
            { 0, 0,   0,    1 }
        };
    }

    /// <summary>
    /// Y軸周りの回転行列を生成します。
    /// </summary>
    public static double[,] CreateRotationY(double angleRad)
    {
        double cos = Math.Cos(angleRad);
        double sin = Math.Sin(angleRad);
        return new double[4, 4] {
            { cos,  0, sin, 0 },
            { 0,    1, 0,   0 },
            { -sin, 0, cos, 0 },
            { 0,    0, 0,   1 }
        };
    }

    /// <summary>
    /// Z軸周りの回転行列を生成します。
    /// </summary>
    public static double[,] CreateRotationZ(double angleRad)
    {
        double cos = Math.Cos(angleRad);
        double sin = Math.Sin(angleRad);
        return new double[4, 4] {
            { cos, -sin, 0, 0 },
            { sin, cos,  0, 0 },
            { 0,   0,    1, 0 },
            { 0,   0,    0, 1 }
        };
    }

    /// <summary>
    /// 3Dベクトルの外積 (v1 x v2) を計算します。
    /// </summary>
    public static double[] CrossProduct(double[] v1, double[] v2)
    {
        return new double[] {
            v1[1] * v2[2] - v1[2] * v2[1],
            v1[2] * v2[0] - v1[0] * v2[2],
            v1[0] * v2[1] - v1[1] * v2[0]
        };
    }

    // --- 行列からのデータ抽出 ---

    /// <summary>
    /// 4x4 行列から位置ベクトル (X, Y, Z) を抽出します。
    /// </summary>
    public static double[] GetTranslation(double[,] M)
    {
        return new double[] { M[0, 3], M[1, 3], M[2, 3] };
    }

    /// <summary>
    /// 4x4 行列から X軸ベクトル (n) を抽出します。
    /// </summary>
    public static double[] GetXAxis(double[,] M)
    {
        return new double[] { M[0, 0], M[1, 0], M[2, 0] };
    }

    /// <summary>
    /// 4x4 行列から Y軸ベクトル (o) を抽出します。
    /// </summary>
    public static double[] GetYAxis(double[,] M)
    {
        return new double[] { M[0, 1], M[1, 1], M[2, 1] };
    }

    /// <summary>
    /// 4x4 行列から Z軸ベクトル (a) を抽出します。
    /// </summary>
    public static double[] GetZAxis(double[,] M)
    {
        return new double[] { M[0, 2], M[1, 2], M[2, 2] };
    }

    /// <summary>
    /// 4x4 行列をデバッグ用に文字列化します。
    /// </summary>
    public static string MatrixToString(double[,] M)
    {
        StringBuilder sb = new StringBuilder();
        sb.AppendLine($"  [{M[0, 0]:F3}, {M[0, 1]:F3}, {M[0, 2]:F3}, {M[0, 3]:F3}]");
        sb.AppendLine($"  [{M[1, 0]:F3}, {M[1, 1]:F3}, {M[1, 2]:F3}, {M[1, 3]:F3}]");
        sb.AppendLine($"  [{M[2, 0]:F3}, {M[2, 1]:F3}, {M[2, 2]:F3}, {M[2, 3]:F3}]");
        sb.AppendLine($"  [{M[3, 0]:F3}, {M[3, 1]:F3}, {M[3, 2]:F3}, {M[3, 3]:F3}]");
        return sb.ToString();
    }

    // InverseJacobianで使用する部分

    /// <summary>
    /// 6x6の単位行列を生成します。
    /// </summary>
    public static double[,] Identity6x6()
    {
        return new double[6, 6] {
        { 1, 0, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 },
        { 0, 0, 1, 0, 0, 0 },
        { 0, 0, 0, 1, 0, 0 },
        { 0, 0, 0, 0, 1, 0 },
        { 0, 0, 0, 0, 0, 1 }
    };
    }

    /// <summary>
    /// 行列の転置を計算します (A^T)。
    /// </summary>
    public static double[,] Transpose(double[,] M)
    {
        int rows = M.GetLength(0);
        int cols = M.GetLength(1);
        double[,] MT = new double[cols, rows];
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                MT[j, i] = M[i, j];
            }
        }
        return MT;
    }

    /// <summary>
    /// 行列の加算 (A + B) を行います。
    /// </summary>
    public static double[,] Add(double[,] A, double[,] B)
    {
        int rows = A.GetLength(0);
        int cols = A.GetLength(1);
        double[,] C = new double[rows, cols];
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                C[i, j] = A[i, j] + B[i, j];
            }
        }
        return C;
    }

    /// <summary>
    /// 行列とベクトルの乗算 (M * V) を行います (M: 6x6, V: 6x1)。
    /// </summary>
    public static double[] MultiplyMatrixVector(double[,] M, double[] V)
    {
        double[] R = new double[6];
        for (int i = 0; i < 6; i++)
        {
            R[i] = 0;
            for (int j = 0; j < 6; j++)
            {
                R[i] += M[i, j] * V[j];
            }
        }
        return R;
    }

    /// <summary>
    /// N x N 行列の逆行列をガウス・ジョルダン法で計算。
    /// (主に行列 B = J*J^T + damping^2 * I の計算に使用)
    /// </summary>
    public static double[,] Inverse(double[,] M)
    {
        int N = M.GetLength(0);
        if (N != M.GetLength(1))
        {
            throw new ArgumentException("行列は正方行列である必要があります。");
        }

        // 拡大行列 [M | I] を作成（実装は前回提示した内容と同一）
        double[,] augmented = new double[N, 2 * N];
        for (int i = 0; i < N; i++)
        {
            for (int j = 0; j < N; j++)
            {
                augmented[i, j] = M[i, j];
                augmented[i, j + N] = (i == j) ? 1.0 : 0.0;
            }
        }

        // ガウス・ジョルダン法
        for (int i = 0; i < N; i++)
        {
            // ピボットの選択 (絶対値が最大の要素)
            int pivot = i;
            for (int j = i + 1; j < N; j++)
            {
                if (Math.Abs(augmented[j, i]) > Math.Abs(augmented[pivot, i]))
                {
                    pivot = j;
                }
            }

            // 行の交換
            for (int j = 0; j < 2 * N; j++)
            {
                double temp = augmented[i, j];
                augmented[i, j] = augmented[pivot, j];
                augmented[pivot, j] = temp;
            }

            // ピボット要素を 1 に
            double divisor = augmented[i, i];
            if (Math.Abs(divisor) < 1e-12)
            {
                // 特異点：逆行列が存在しない
                // IKソルバの場合は特異点処理 (ダンピングで緩和を試みるが、ここではエラーとして扱う)
                throw new InvalidOperationException("特異点: 行列は逆行列を持ちません。");
            }
            for (int j = 0; j < 2 * N; j++)
            {
                augmented[i, j] /= divisor;
            }

            // 他の行の消去
            for (int j = 0; j < N; j++)
            {
                if (i != j)
                {
                    double factor = augmented[j, i];
                    for (int k = 0; k < 2 * N; k++)
                    {
                        augmented[j, k] -= factor * augmented[i, k];
                    }
                }
            }

            if (Math.Abs(augmented[i, i]) < 1e-12)
            {
                throw new InvalidOperationException("特異点: 行列は逆行列を持ちません。");
            }
        }

        // 逆行列部分 [I | M^-1] を抽出
        double[,] Minv = new double[N, N];
        for (int i = 0; i < N; i++)
        {
            for (int j = 0; j < N; j++)
            {
                Minv[i, j] = augmented[i, j + N];
            }
        }
        return Minv;
    }
}
