using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HingeJoint : Joint {

	//lower arms: 0, 0, -1
	public Vector3 axis;
	//lower arms: 0.0f ~ 160.0f
	public float angleLowerLimit, angleUpperLimit;
	[Range(-180, 180)]
	public float angle;

	//
	protected override void localTransformUpdate () {		
		//
		float boundedAngle = limitAngle(angle, angleLowerLimit, angleUpperLimit);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //#2
        //TODO: update local rotation
        //you can also use init object transformation info. using ' td '

        Vector3 rotationVector = new Vector3(0, 0, boundedAngle);
        this.transform.localRotation = Quaternion.Euler(rotationVector);
        print("position of the hinge joint" + td.localPosition);
        print("rotation of the hinge joint" + td.localRotation); 
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }

    //
    public override int getDoF() {
		//
		return 1;
	}

	//
	public override Vector3 getAxis(int index=0) {
		//
		if (index != 0) System.Environment.Exit(0);
		//
		return axis;
	}

	// Use this for initialization
	void Start () {	
		base.init();
	}

	// Update is called once per frame
	void Update () {	

	}
}
