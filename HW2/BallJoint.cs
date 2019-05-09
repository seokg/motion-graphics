using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BallJoint : Joint {

	//
	protected Vector3[] axes = {new Vector3(1,0,0), new Vector3(0,1,0), new Vector3(0,0,1)};

	//shoulder: -130.0f~150.0f, wrist: -90.0f~90.0f
	public float angleXLowerLimit, angleXUpperLimit;
	[Range(-180, 180)]
	public float angleX;

	//shoulder: -135.0f~135.0f, wrist: -45.0f~20.0f
	public float angleYLowerLimit, angleYUpperLimit;
	[Range(-180, 180)]
	public float angleY;

	//shoulder: -130.0f~180.0f, wrist: -80.0f~80.0f
	public float angleZLowerLimit, angleZUpperLimit;
	[Range(-180, 180)]
	public float angleZ;

	//
	//public Quaternion quat;//Quaternion.identity (x:0, y:0, z:0, w:1)

	//
	protected override void localTransformUpdate () {		
		//
		float boundedAngleX = limitAngle(angleX, angleXLowerLimit, angleXUpperLimit);
		float boundedAngleY = limitAngle(angleY, angleYLowerLimit, angleYUpperLimit);
		float boundedAngleZ = limitAngle(angleZ, angleZLowerLimit, angleZUpperLimit);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //#1
        //TODO: update local rotation
        //you can also use init object transformation info. using ' td '
        //Quaternion.Euler q;
        Vector3 rotationVector = new Vector3(boundedAngleX, boundedAngleY, boundedAngleZ);
        print("rot vector" + rotationVector);
        print(string.Format("x y z euler angle:{0}, {1}, {2}", boundedAngleX, boundedAngleY, boundedAngleZ));
        this.transform.localRotation = Quaternion.Euler(rotationVector);
        print("position of the ball joint" + td.localPosition);
        print("rotation of the ball joint" +  td.localRotation);
        //You are allowed to use Unity’s global transform updating module by only fixing
        //“transform.localRotation”.
        //Robot Kyle’s initial transform matrix cloning code is provided as below.
        //It can be used in inherited BallJoints and HingeJoints.
        //“TransformData td = transform.Clone()”


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }

    //
    public override int getDoF() {
		//
		return 3;
	}

	//
	public override Vector3 getAxis(int index) {
		//
		if (index < 0 || 3 <= index) System.Environment.Exit(0);
		//
		return axes[index];
	}

	// Use this for initialization
	void Start () {	
		base.init();
	}

	// Update is called once per frame
	void Update () {	

	}
}
