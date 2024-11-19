using UnityEngine;

public static class Utilities
{
    // ”правление плавным изменением
    public static float MoveTo(float value, float target, float speed, float deltaTime, float min = 0, float max = 1)
    {
        var diff = target - value;

        var delta = Mathf.Clamp(diff, -speed * deltaTime, speed * deltaTime);

        return Mathf.Clamp(value + delta, min, max);
    }

    public static Vector3 Scale6(
    Vector3 value,
    float positiveX, float negativeX,
    float positiveY, float negativeY,
    float positiveZ, float negativeZ
)
    {
        var result = value;

        switch (result.x)
        {
            case > 0:
                result.x *= positiveX;
                break;
            case < 0:
                result.x *= negativeX;
                break;
        }

        switch (result.y)
        {
            case > 0:
                result.y *= positiveY;
                break;
            case < 0:
                result.y *= negativeY;
                break;
        }

        switch (result.z)
        {
            case > 0:
                result.z *= positiveZ;
                break;
            case < 0:
                result.z *= negativeZ;
                break;
        }

        return result;
    }
}