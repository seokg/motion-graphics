using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EndEffector : Joint {
	
	//
	protected override void localTransformUpdate () {		
		
	}

	//
	public override int getDoF() {
		//
		return 0;
	}

	//
	public override Vector3 getAxis(int index) {
		//
		System.Environment.Exit(0);
		//
		return new Vector3();
	}

	// Use this for initialization
	void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {
		
	}
}
