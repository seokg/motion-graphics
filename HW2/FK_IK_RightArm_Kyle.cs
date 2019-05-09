using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.Assertions;

public class FK_IK_RightArm_Kyle : MonoBehaviour {

	//
    public enum Options { FK, CCDIK, JacobianIK }
    public Options option;

	//
	Transform[] Ts;
	Transform EE, Target;

	//
	int totalDoF = 0;
	Joint[] joints;
	Joint endEffector;
	List<Vector3> axes = new List<Vector3>();

    //
	double[,] J; //body Jacobian matrix
	int CCDorder;
    
    // Adjoint transformation
    // This function implements V_out = Ad_g(V_in), where g = (R,p), V_out = (w_out,v_out),  V_in = (w_in, v_in)
    void Adjoint(Vector3 p, Quaternion R, Vector3 w_in, Vector3 v_in, out Vector3 w_out, out Vector3 v_out)
    {
        w_out = R * w_in;
        v_out = Vector3.Cross(p, R * w_in) + R * v_in;
    }

    //input: exponential coordinates (randian)
    //output: quaternion corresponding to the exponential coordinates
    Quaternion exp(Vector3 expCoord)
    {
        float angle = expCoord.magnitude * 180 / Mathf.PI;
        return Quaternion.AngleAxis(angle, expCoord.normalized);
    }

    //input: quaternion 
    //output: exponential coordinates (radian)    
    Vector3 log(Quaternion quat)
    {
        float angleDeg;
        Vector3 w;
        quat.ToAngleAxis(out angleDeg, out w);
        w *= angleDeg * Mathf.PI / 180; //convert to radian
        return w;
    }

	// Use this for initialization
	void Start () {
        //init
        Ts = new Transform[3];
        joints = new Joint[3];
        
        //initialize option as FK
        option = Options.FK;

        //initialize robot body parts
        Ts[0] = GameObject.Find("Right_Upper_Arm_01").transform;
		Ts[1] = GameObject.Find("Right_Forearm_01").transform;
		Ts[2] = GameObject.Find("Right_Wrist_01").transform;

        //initialize end effector as "Right_Hand_01" and target as "TargetCube"
        EE = GameObject.Find("Right_Hand_01").transform;
		Target = GameObject.Find("TargetCube").transform;

		//initialize joint axis parameters
		joints[0] = GameObject.Find("Right_Upper_Arm_01").GetComponent<Joint>();
		joints[1] = GameObject.Find("Right_Forearm_01").GetComponent<Joint>();
		joints[2] = GameObject.Find("Right_Wrist_01").GetComponent<Joint>();
  
        //get total degree of freedom
        foreach (Joint joint in joints) {
			totalDoF += joint.getDoF();
		}
        print("Total number of dof: " + totalDoF);

		//add axis
		foreach (Joint joint in joints) {
            for (int index = 0; index != joint.getDoF(); ++index) {
				axes.Add(joint.getAxis(index));
			}
		} 

		//
		J = new double[6, totalDoF]; //6x7 matrix
        CCDorder = 2; //from the terminal link
    }

    // Update is called once per frame
    void Update()
    {
		if (option == Options.JacobianIK)
			IK_Numerial ();
		else if (option == Options.CCDIK)
			IK_CCD ();
		else
			FK ();
    }

	void FK()
	{		
		joints[2].manualUpdate();
	}

    //perform CCD
    void IK_CCD()
    {
        //perform cyclic coordinate descent IK
        IK_CCD_Link(CCDorder);

        //set CCDoder to the next (proximal) one
        CCDorder -= 1;
        if (CCDorder == -1) CCDorder = 2;        
    }

    //perform CCD to the link(linkIdx)
    void IK_CCD_Link(int linkIdx)
    {
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //#5
        //TODO: rotate joint to make EE approach Target
        //You consider "TargetCube" as point at CCD method(== don't have to consider rotation of target)

        //minimize the position of EE and target
        
        Vector3 error = EE.position - Target.position;
        print("error: " + error.magnitude);

        if (error.magnitude > 0.0001)
        {

            Vector3 u = EE.position - Ts[linkIdx].position;
            Vector3 v = Target.position - Ts[linkIdx].position;
            u = u.normalized;
            v = v.normalized;
            Vector3 w = Vector3.Cross(u, v) / Vector3.Cross(u, v).magnitude;
            float dot = Vector3.Dot(u, v);
            float theta = Mathf.Atan2(Vector3.Dot(w, Vector3.Cross(u, v)), dot);
            //Quaternion omega = exp(w*theta);
            print("1. angle to rotate: " + theta);
            print("2. axis to rotate: " + w);

            Ts[linkIdx].transform.Rotate(w, 180/Mathf.PI*(theta), Space.World);
            //Ts[linkIdx].localRotation *= exp(Ts[linkIdx].transform.rotation*w*theta);
        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }

    //Jacobian-based IK
    void IK_Numerial()
    {
        //Target seen from EE
        Vector3 p = EE.InverseTransformPoint(Target.position); //Target's position seen from EE
        Quaternion R = Quaternion.Inverse(EE.rotation) * Target.rotation; //Target's rotation seen from EE
        Vector3 w = log(R); // log: translate into expoenential rotation form

        //print("target position: " + Target.position);
        //print("target rotation(quaternion): " + R);
        //print("target omega(exp): " + w);

        //return if error is small enough
        if (p.magnitude < 0.001f && w.magnitude < 0.001f) return;

        //
        //otherwise, do inverse kinematics
        //

        double[] b = new double[6];
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //#3
        //TODO: set b[] as the desired body velocity. 
        b = new double[] { w.x, w.y, w.z, p.x, p.y, p.z };

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //update body Jacobian
        UpdateBodyJacobian();
                
        //solve J x = b for x
		double[] x = new double[totalDoF];

        int info; alglib.densesolverlsreport rep;
		alglib.rmatrixsolvels(J, 6, totalDoF, b, 1e-3, out info, out rep, out x);
        
        //rotate each joint by x        
		//T0.localRotation = T0.localRotation * exp(axes[0] * (float)(x[0]) + axes[1] * (float)(x[1]) + axes[2] * (float)(x[2]));
		//T1.localRotation = T1.localRotation * exp(axes[3] * (float)(x[3]));
		//T2.localRotation = T2.localRotation * exp(axes[4] * (float)(x[4]) + axes[5] * (float)(x[5]) + axes[6] * (float)(x[6]));
		//
		int totalDoFIndex = 0;
		//
		for (int transfIndex=0; transfIndex!=3; ++transfIndex) {			
			//
			Vector3 expCoord = new Vector3(0,0,0);
			for (int axisIndex = 0; axisIndex != joints[transfIndex].getDoF(); ++axisIndex) {
				expCoord += joints[transfIndex].getAxis(axisIndex) * (float)(x[totalDoFIndex++]);
			}
			//
			Ts[transfIndex].localRotation *= exp(expCoord);
		}
    }

    // Update Body Jacobian matrix
    void UpdateBodyJacobian()
    {
		Vector3[] Jw = new Vector3[totalDoF]; //top three rows (for rotation) of Jacobian
		Vector3[] Jv = new Vector3[totalDoF]; //bottom three rows (for translation) of Jacobian

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //#4
        //TODO: Computer Jw[] and Jv[]. 
        //you can also use ' Adjoint(`) ' for computing elements of J
        int dof_idx = 0;
        int joint_idx = 0;
        foreach (Joint joint in joints)
        {   

            for (int index = 0; index != joint.getDoF(); ++index)
            {
                Vector3 p = EE.InverseTransformPoint(Ts[joint_idx].position); //joint's position seen from EE
                Quaternion R_quat = Quaternion.Inverse(EE.rotation) * Ts[joint_idx].rotation; //joint's rotation seen from EE
                Vector3 w_in = joint.getAxis(index);
                Vector3 v_in = Vector3.zero;
                Adjoint(p, R_quat, w_in, v_in, out Jw[dof_idx], out Jv[dof_idx]);
                dof_idx++;
            }
            joint_idx++;
        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // assert if total_idx does not match with totalDOF
        Assert.AreEqual(totalDoF, dof_idx);

        //Pack Jw[] and Jv[] into J
        for (int i = 0; i < totalDoF; ++i)
        {
            J[0, i] = Jw[i].x;
            J[1, i] = Jw[i].y;
            J[2, i] = Jw[i].z;
            J[3, i] = Jv[i].x;
            J[4, i] = Jv[i].y;
            J[5, i] = Jv[i].z;
        }        

    }
}
