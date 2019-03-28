using System.Collections;
using System.Collections.Generic;
using UnityEngine;


//TODO
// 1. plane rotation fix

public class BezierCurve : MonoBehaviour
{
    public GameObject[] Point_Objects;
    public GameObject Airplane;
    public LineRenderer lineRenderer;

    // # of points = 100
    public Vector3[] BezierPoints;

    Vector3[] ControlPoints = new Vector3[4];

    [Range(0.0f, 1.0f)]
    public float parameter_t; // parameter


    void Start()
    {
        //Airplane.transform.rotation = Quaternion.Euler(-24.468f, -42.204f, 23.543f);
    }
    // Update is called once per frame
    void Update()
    {
        GetControlPoints();
        DrawBezierCurve();

        //////////////////////////
        /// #14                ///
        /// Implement Update() ///
        //////////////////////////

        // Move the airplane to the point of Bezier Curve at parameter_t (use Airplane.transform.position)
        Airplane.transform.position = ComputeBezierPoint(parameter_t);

        // Compute 3 orthogonal vector3 of frenet frame
        Vector3 t_t = ComputeBezierTangent(parameter_t);
        Vector3 n_t = ComputeBezier2ndDerivative(parameter_t);
        n_t = n_t - Vector3.Dot(t_t, n_t) * t_t;
        Vector3 b_t = ComputeFrenetNormal(t_t, n_t);

        // Visualize the axes of frenet frame (use Debug.DrawLine())
        //Debug.Log("airplane position: " + Airplane.transform.position);
        //Debug.Log("airplane tangent: " + (Airplane.transform.position + t_t));

        Debug.DrawLine(Airplane.transform.position, Airplane.transform.position + 20 * t_t, Color.blue);
        Debug.DrawLine(Airplane.transform.position, Airplane.transform.position + 20 * n_t, Color.red);
        Debug.DrawLine(Airplane.transform.position, Airplane.transform.position + 20 * b_t, Color.green);

        // Generate rotation matrix with basis vectors of franet frame and convert it to a quaternion (use Matrix4x4, RotToQuat())
        Matrix4x4 rot = new Matrix4x4();
        rot.SetColumn(0, new Vector4(n_t.x, n_t.y, n_t.z, 0));
        rot.SetColumn(1, new Vector4(b_t.x, b_t.y, b_t.z, 0));
        rot.SetColumn(2, new Vector4(t_t.x, t_t.y, t_t.z, 0));
        rot.SetColumn(3, new Vector4(0,0,0,1));
        Quaternion q = RotToQuat(rot);

        // Set airplane's rotation (use Airplane.transform.rotation)
        Airplane.transform.rotation = q;
    }

    void GetControlPoints() {
        for (int i = 0; i < 4; i++)
        {
            ControlPoints[i].x = Point_Objects[i].transform.position.x;
            ControlPoints[i].y = Point_Objects[i].transform.position.y;
            ControlPoints[i].z = Point_Objects[i].transform.position.z;
        }
    }

    Vector3 ComputeBezierPoint(float t)
    {
        Vector3 b_t = Vector3.zero;

        ///////////////////////////////////////////
        /// #8                                  ///
        /// find the point on Bezier Curve at t ///
        ///////////////////////////////////////////
        ///
        Vector3 P_0 = ControlPoints[0];
        Vector3 P_1 = ControlPoints[1];
        Vector3 P_2 = ControlPoints[2];
        Vector3 P_3 = ControlPoints[3];

        b_t = Mathf.Pow(1 - t, 3) * P_0 + 3 * Mathf.Pow(1 - t, 2) * t * P_1 + 3 * (1 - t) * Mathf.Pow(t, 2) * P_2 + Mathf.Pow(t, 3) * P_3;
        return b_t;
    }

    void DrawBezierCurve() {

        /////////////////////////////////////////////////////////
        /// #9                                                ///
        /// Draw a Bezier curve with points in BezierPoints[] ///
        /// (use ComputeBezierPoint(), LineRenderer)          ///
        /////////////////////////////////////////////////////////

        //int index = 0;
        //Vector3[] b_points = new Vector3[length_line];
        int length_line = 100;
        lineRenderer.positionCount = length_line;
        //lineRenderer = GetComponent<LineRenderer>();
        for (int iter = 0; iter < 100; iter ++)
        {
            BezierPoints[iter] = ComputeBezierPoint((float)iter / 100);
            lineRenderer.SetPosition(iter, BezierPoints[iter]);
        }
        

    }

    Vector3 ComputeBezierTangent(float t) {
        Vector3 tangent = Vector3.zero;

        /////////////////////////////////////////////////////
        /// #10                                           ///
        /// compute tangent vector of a Bezier Curve at t ///
        /////////////////////////////////////////////////////

        //dP(t) / dt = -3(1 - t) ^ 2 * P0 + 3(1 - t) ^ 2 * P1 - 6t(1 - t) * P1 - 3t ^ 2 * P2 + 6t(1 - t) * P2 + 3t ^ 2 * P3
        Vector3 P_0 = ControlPoints[0];
        Vector3 P_1 = ControlPoints[1];
        Vector3 P_2 = ControlPoints[2];
        Vector3 P_3 = ControlPoints[3];

        tangent = -3 * Mathf.Pow(1 - t, 2) * P_0 + 3 * Mathf.Pow(1 - t, 2) * P_1 - 6 *t* (1-t)*P_1 - 3* Mathf.Pow(t,2)*P_2 + 6*t*(1-t)*P_2 + 3 * Mathf.Pow(t,2) *P_3 ;

        return tangent.normalized;
    }

    Vector3 ComputeBezier2ndDerivative(float t) {
        Vector3 second_d = Vector3.zero;

        /////////////////////////////////////////////////////
        /// #11                                           ///
        /// compute 2nd derivative of a Bezier Curve at t ///
        /////////////////////////////////////////////////////
        //dP(t) / dt = 6(1 - t) * P0 - 6 (2 - 3t) * P1 + 6(1 - 3t) * P2 + 6t * P3
        Vector3 P_0 = ControlPoints[0];
        Vector3 P_1 = ControlPoints[1];
        Vector3 P_2 = ControlPoints[2];
        Vector3 P_3 = ControlPoints[3];
        second_d = 6 * (1 - t) * P_0 - 6 * (2 - 3 * t) * P_1 + 6 * (1 - 3 * t) * P_2 + 6 * t * P_3;
        
        return second_d.normalized;
    }

    Vector3 Vec3Cross(Vector3 a, Vector3 b)
    {
        Vector3 v = Vector3.zero;

        /////////////////////////////////////////////////////
        /// #12                                           ///
        /// compute cross product of two vectors a and b  ///
        /////////////////////////////////////////////////////
        Debug.Log(a);
        Debug.Log(b);

        v = Vector3.Cross(a, b);
        //v.x =  a.y * b.z - a.z * b.y;
        //v.y =  a.z * b.x - a.x * b.z;
        //v.z =  a.x * b.y - a.y * b.x;

        return v.normalized;
    }

    Vector3 ComputeFrenetNormal(Vector3 first_derivative, Vector3 second_derivative) {
        Vector3 N = Vector3.zero;

        /////////////////////////////////////////////////////
        /// #13                                           ///
        /// compute Normal Vector of Frenet frame         ///
        /////////////////////////////////////////////////////
        //binoral vectdor of the vurve at t
        N = Vec3Cross(first_derivative, second_derivative);
        return N.normalized;
    }

    float MatTrace(Matrix4x4 M)
    {
        float trace = 0.0f;
        for (int i=0; i<3 ;i++)
        {
            trace += M[i, i];
        }
        return trace;
    }
    public Quaternion RotToQuat(Matrix4x4 R)
    {
        Quaternion q = new Quaternion();

        /////////////////////////////////////////////////////////////
        /// #2                                                    ///
        /// copy function implemented at #2 of ComputeRotation.cs ///
        /////////////////////////////////////////////////////////////
        ///
        Debug.Log("rotation matrix: \n"+R);
        float trace = MatTrace(R);
        float S;
        if (trace > 0)
        {
            q.w = Mathf.Sqrt(Mathf.Max(0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2;
            q.x = (R[2, 1] - R[1, 2]) / q.w / 4.0f;
            q.y = (R[0, 2] - R[2, 0]) / q.w / 4.0f;
            q.z = (R[1, 0] - R[0, 1]) / q.w / 4.0f;

        }
        else
        {
            Debug.Log("Warning: Trace is smaller or equal to zero");
            if ((R.m00 > R.m11) & (R.m00 > R.m22))
            {
                S = Mathf.Sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2;
                q.w = (R.m21 - R.m12) / S;
                q.x = 0.25f * S;
                q.y = (R.m01 + R.m10) / S;
                q.z = (R.m02 + R.m20) / S;
            }
            else if (R.m11 > R.m22)
            {
                S = Mathf.Sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2;
                q.w = (R.m02 - R.m20) / S;
                q.x = (R.m01 + R.m10) / S;
                q.y = 0.25f * S;
                q.z = (R.m12 + R.m21) / S;
            }
            else
            {
                S = Mathf.Sqrt(1 + R[2, 2] - R[1, 1] - R[0, 0]) * 2;
                q.w = (R.m10 - R.m01) / S;
                q.x = (R.m02 + R.m20) / S;
                q.y = (R.m12 + R.m21) / S;
                q.z = 0.25f * S;
            }

        }
        return q;
    }
}
