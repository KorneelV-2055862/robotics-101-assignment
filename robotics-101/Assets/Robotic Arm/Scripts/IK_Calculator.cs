using System;
using Unity.VisualScripting;
using UnityEngine;

public class IK_Calculator
{
    private float L1, L2, L3;

    public IK_Calculator(float L1, float L2, float L3) { 
        this.L1 = L1;
        this.L2 = L2;
        this.L3 = L3;
    }

    public Vector3 Calculate(Vector3 targetPoint3D)
    {
        if (targetPoint3D == Vector3.zero) return Vector3.zero;

        float hypotenuse = GetHypotenuse(targetPoint3D);
        Vector2 targetPoint2D = new Vector2(hypotenuse, targetPoint3D.y);
        // log the target point

        // cos, tan, atan, acos and atan2 return results in radians
        float theta = GetTheta(targetPoint3D); // Joint 1 Angle (Base Rotation)
        float gamma = GetGamma(targetPoint2D);// Joint 2 Angle
        float alpha = GetAlpha(targetPoint2D);// Joint 3 Angle


        var targetAngles = new Vector3(theta, gamma, alpha);

        return targetAngles;
    }

    private float GetAlpha(Vector2 targetPoint2D)
    {
        // cos(alpha) = (b^2 + c^2 - a^2) / (2 b c)
        // cos(alpha) = (L2^2 + L3^2 - a^2) / (2 L2 L3)
        // a^2 = b^2 + c^2
        // b = x, c = y - L1
        float asqd = Mathf.Pow(targetPoint2D.x, 2) + Mathf.Pow(targetPoint2D.y - L1, 2);
        float cosAlpha = (Mathf.Pow(L2, 2) + Mathf.Pow(L3, 2) - asqd) / (2 * L2 * L3);

        // reverse cos to get the correct angle
        float alpha = Mathf.Acos(cosAlpha) *  Mathf.Rad2Deg;
        return 180 - alpha;
    }

    private float GetGamma(Vector2 targetPoint2D)
    {
        // divided into 3 angles
        // first angle is 90 degrees
        // second angle with tangens (x and y - L1)
        // third angle with cosine rule
        float gamma = 90;

        float gamma2 = Mathf.Atan((targetPoint2D.y - L1) / targetPoint2D.x);

        // cos(gamma3) = (a^2 + L2^2 - L3^2) / (2 a L2)
        // a = sqrt(x^2 + (y - L1)^2) (see GetAlpha)

        float a = Mathf.Sqrt(Mathf.Pow(targetPoint2D.x, 2) + Mathf.Pow(targetPoint2D.y - L1, 2));
        
        float cosGamma3 = (Mathf.Pow(a, 2) + Mathf.Pow(L2, 2) - Mathf.Pow(L3, 2)) / (2 * a * L2);
        float gamma3 = Mathf.Acos(cosGamma3);

        // actual gamma angle is the sum of the 3 angles, but unity needs 180 - our calculated angle
        gamma += gamma2 * Mathf.Rad2Deg + gamma3 * Mathf.Rad2Deg;

        return 180 - gamma;
    }

    private float GetTheta(Vector3 targetPoint3D)
    {
        // theta = atan2(z/x)
        return Mathf.Atan2(targetPoint3D.x, targetPoint3D.z) * Mathf.Rad2Deg;
    }

    private float GetHypotenuse(Vector3 targetPoint3D)
    {
        // hypotenuse^2 = x^2 + z^2
        return Mathf.Sqrt(Mathf.Pow(targetPoint3D.x, 2) + Mathf.Pow(targetPoint3D.z, 2));
    }
}
