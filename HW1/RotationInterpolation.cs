using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

//TODO
// 1. quat and exponential debug
// 2. number visualization check
// 3. rot2quat check

public class RotationInterpolation : MonoBehaviour
{
    float ang2rad(float input)
    {
        return (Mathf.PI / 180.0f) * input;
    }

    float rad2ang(float input)
    {
        return 180.0f / Mathf.PI * input;
    }

    Matrix4x4 EulerToRot(Vector3 v)
    {
        Matrix4x4 R = new Matrix4x4();
        Matrix4x4 Rx = new Matrix4x4();
        Matrix4x4 Ry = new Matrix4x4();
        Matrix4x4 Rz = new Matrix4x4();

        ///////////////////////////////////////////////////////////////////////
        /// #1                                                              ///
        /// Implement conversion from euler angles input to rotation matrix ///
        /// !!!!!Unity3D engine uses YXZ intrinsic rotation!!!!!            ///
        ///////////////////////////////////////////////////////////////////////


        float ang_x = v.x;  // x
        float ang_y = v.y;  // y
        float ang_z = v.z;  // z


        float rad_x = ang2rad(ang_x);
        float rad_y = ang2rad(ang_y);
        float rad_z = ang2rad(ang_z);

        float cosx   = Mathf.Cos(rad_x);
        float sinx   = Mathf.Sin(rad_x);
        float cosy = Mathf.Cos(rad_y);
        float siny = Mathf.Sin(rad_y);
        float cosz   = Mathf.Cos(rad_z);
        float sinz   = Mathf.Sin(rad_z);


        //x = [1 0 0; 0 cos -sin; 0 sin cos]
        //y = [cos 0 sin; 0 1 0; -sin 0 cos]
        //z = [cos -sin 0 ; sin cos 0 ; 0 0 1]

        Rx.SetRow(0, new Vector4(1, 0, 0, 0));
        Rx.SetRow(1, new Vector4(0, cosx, -sinx, 0));
        Rx.SetRow(2, new Vector4(0, sinx, cosx, 0));
        Rx.SetRow(3, new Vector4(0, 0, 0, 1));

        Ry.SetRow(0, new Vector4(cosy, 0, siny, 0));
        Ry.SetRow(1, new Vector4(0, 1, 0, 0));
        Ry.SetRow(2, new Vector4(-siny, 0, cosy, 0));
        Ry.SetRow(3, new Vector4(0, 0, 0, 1));

        Rz.SetRow(0, new Vector4(cosz, -sinz, 0, 0));
        Rz.SetRow(1, new Vector4(sinz, cosz, 0, 0));
        Rz.SetRow(2, new Vector4(0, 0, 1, 0));
        Rz.SetRow(3, new Vector4(0, 0, 0, 1));

        R = Ry * Rx * Rz;
        //R = Rz * Ry * Rx;
        //Debug.Log(R);
        return R;
    }

    Quaternion RotToQuat(Matrix4x4 R)
    {
        Quaternion q = new Quaternion();

        ///////////////////////////////////////////////////////////////
        /// #2                                                      ///
        /// Implement conversion from rotation matrix to quaternion ///
        ///////////////////////////////////////////////////////////////
        ///
        q.w = Mathf.Sqrt(Mathf.Max(0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2;
        q.x = (R[2, 1] - R[1, 2]) / q.w / 4.0f ;
        q.y = (R[0, 2] - R[2, 0]) / q.w / 4.0f;
        q.z = (R[1, 0] - R[0, 1]) / q.w / 4.0f;
        //Debug.Log(R);
        
        return q;
    }

    Vector3 RotToExp(Matrix4x4 R)
    {
        float theta = 0f;
        Vector3 w_unit = Vector3.zero;
        double EPS = 1E-10;
        /////////////////////////////////////////////////////////////////////////////
        /// #3                                                                    ///
        /// Implement conversion from rotation matrix to angle-axis (exponential) ///
        /////////////////////////////////////////////////////////////////////////////

        bool identity_flag = true;
        float trace = 0.0f;
        for(int iter = 0; iter < 4; iter++)
        {
            trace += R[iter, iter];

            if ( (R[iter,iter] > 1 - EPS) && 
                 (R[iter,iter] < 1 + EPS)) 
            {
                // identity
                continue;
            }
            else
            {
                // not identity
                identity_flag = false;
            }            
        }

        if(identity_flag)
        {
            //Debug.Log("rotation: identity");
            theta = 0;
            w_unit.x = 1;
            w_unit.y = 1;
            w_unit.z = 1;
            w_unit /= w_unit.magnitude;
            //Debug.Log(w_unit.magnitude);
        }

        else
        {
            //Debug.Log("trace: "  + trace);
            theta = Mathf.Acos((R[0, 0] + R[1, 1] + R[2, 2] - 1) / 2.0f);
            w_unit.x = R.m21 - R.transpose.m21;
            w_unit.y = R.m02 - R.transpose.m02;
            w_unit.z = R.m10 - R.transpose.m10;
            w_unit /= w_unit.magnitude;
        }
        //Debug.Log("w_unit: " + w_unit);
        //Debug.Log("w_unit mag: " + w_unit.magnitude);
        //Debug.Log("theta: " + theta);
        return w_unit * theta;
    }

    Quaternion ExpToQuat(Vector3 w)
    {
        Quaternion q = new Quaternion();

        ///////////////////////////////////////////////////////////
        /// #4                                                  ///
        /// Implement conversion from exponential to quaternion ///
        ///////////////////////////////////////////////////////////

        // q = <cos (theta / 2) , omega sin (theta / 2)>

        float theta = w.magnitude;
        Vector3 w_unit = w / theta;
        float sin = Mathf.Sin(theta / 2.0f);
        
        q.w = Mathf.Cos(theta / 2.0f);
        q.x = w_unit.x * sin;
        q.y = w_unit.y * sin;
        q.z = w_unit.z * sin;
        return q;
    }

    public enum InterpolationMethod { EulerAngles, ExpCoord, Quat }
    public InterpolationMethod interpmethod;

    [Range(0.0f, 1.0f)]
    public float interp; // interpolation parameter

    // Input
    public Vector3 start_EulerAngles; // initial orientation in Euler Angles
    public Vector3 end_EulerAngles; // final orientation in Euler Angles

    // Euler Angles interp
    private Vector3 interp_v;
    
    // Rotation Matrix
    private Matrix4x4 m_start, m_end; 

    // Exponential Coordinate
    private Vector3 w_start, w_end; 
    private Vector3 interp_w;

    //Quaternion
    private Quaternion q_start, q_end;
    private Quaternion interp_q;

    // GUI text
    public Text TextEuler;
    public Text TextExpCoord;
    public Text TextQuat;
    public Text TextInterpVal;
    // Use this for initialization
    void Start()
    {
        TextEuler.text = "EulerAngles: (0.0, 0.0, 0.0)";
        TextExpCoord.text = "Exp. Coords: (0.0, 0.0, 0.0)";
        TextQuat.text = "Quaternion: (0.0, 0.0, 0.0, 1.0)";
        //TextInterpVal.text = "0";

        end_EulerAngles.Set(140.0f, 40.0f, 70.0f);
        transform.position = new Vector3(0.0f, 0.0f, 0.0f);
        //transform.rotation = Quaternion.Euler(-39.297f, 8.484f, -46.218f);
        transform.localScale = new Vector3(0.60573f, 0.25846f, 1.0f);

        //Camera.main.transform.position = new Vector3(0.0f, 0.0f, -2.46f);

        Display.displays[0].Activate();
    }

   

    // Update is called once per frame
    void Update()
    {
        //////////////////////////////////////////////////////////////////
        /// #5                                                         ///
        /// Implement linear interpolation of euler angles             ///
        //////////////////////////////////////////////////////////////////
        float ex = (1 - interp) * start_EulerAngles.x + interp * end_EulerAngles.x;
        float ey = (1 - interp) * start_EulerAngles.y + interp * end_EulerAngles.y;
        float ez = (1 - interp) * start_EulerAngles.z + interp * end_EulerAngles.z;
        interp_v.Set(ex, ey, ez);
        interp_w = RotToExp(EulerToRot(interp_v));
        interp_q = RotToQuat(EulerToRot(interp_v));
        //////////////////////////////////////////////////////////////////
        /// #6                                                         ///
        /// Calculate interpolated rotations and apply it to the cube  ///
        /// 1. Euler Angles (use rotation matrix)                      ///
        /// 2. Exponential Coordinate                                  ///
        /// 3. Quaternion                                              ///
        ///                                                            ///
        /// use transform.rotation = ~ /                               ///
        /// use Quaternion.Slerp() for quaternion interpolation        ///
        //////////////////////////////////////////////////////////////////

        // apply
        switch (interpmethod)
        {
            case InterpolationMethod.EulerAngles:
                transform.rotation = RotToQuat(EulerToRot(interp_v));
                break;

            case InterpolationMethod.ExpCoord:
                w_start = RotToExp(EulerToRot(start_EulerAngles));
                w_end = RotToExp(EulerToRot(end_EulerAngles));
                float wx = (1 - interp) * w_start.x + interp * w_end.x;
                float wy = (1 - interp) * w_start.y + interp * w_end.y;
                float wz = (1 - interp) * w_start.z + interp * w_end.z;
                //interp_q = ExpToQuat(interp_w);
                interp_w.Set(wx, wy, wz);
                //interp_w.x = wx;
                //interp_w.y = wy;
                //interp_w.z = wz;
                transform.rotation = ExpToQuat(interp_w);
                break;

            case InterpolationMethod.Quat:
                q_start = RotToQuat(EulerToRot(start_EulerAngles));
                q_end = RotToQuat(EulerToRot(end_EulerAngles));
                interp_q = Quaternion.Slerp(q_start, q_end, interp);
                transform.rotation = interp_q;
                break;
        }

        //////////////////////////////////////////////////////////////////////////////
        /// #7                                                                     ///
        /// Visualize the rotation in euler angle, angle-axis (exp) and quaternion ///
        /// ex) TextEuler.text = "EulerAngles: (" + value + ")";                   /// 
        //////////////////////////////////////////////////////////////////////////////
        TextEuler.text     = string.Format("EulerAngles: ({0}, {1}, {2})"    , interp_v.x.ToString("F1"), interp_v.y.ToString("F1"), interp_v.z.ToString("F1"));
        TextExpCoord.text  = string.Format("Exp. Coords: ({0}, {1}, {2})"    , interp_w.x.ToString("F1"), interp_w.y.ToString("F1"), interp_w.z.ToString("F1"));
        TextQuat.text      = string.Format("Quaternion: ({0}, {1}, {2}, {3})", interp_q.x.ToString("F1"), interp_q.y.ToString("F1"), interp_q.z.ToString("F1"), interp_q.w.ToString("F1"));
        //TextInterpVal.text = string.Format("Interp: {0}"                     , interp.ToString("F1"));

    }
}