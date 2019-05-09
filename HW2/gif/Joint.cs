using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public abstract class Joint : MonoBehaviour {

	public GameObject currentObject = null;
	public GameObject parentObject = null;
	//protected Joint parent = null;

	//private Transform G;
	protected TransformData td;

	//
	public void init() {
		//
		td = currentObject.transform.Clone();

        if (currentObject != null)
        {
            Debug.Log("initiailize " + currentObject.name);
            Debug.Log("td:  " + td);
        }
    }

	//
	public void manualUpdate() {		
		//
		if (parentObject != null) {
			Joint parent = parentObject.GetComponent<Joint>();			
			parent.manualUpdate();
		}
		//
		localTransformUpdate();
		//
		if (currentObject != null) {
			Debug.Log("manualUpdate " + currentObject.name);
		} 
	}

	//
	protected abstract void localTransformUpdate();

	//
	public abstract int getDoF();

	//
	public abstract Vector3 getAxis(int index);

	//
	public float limitAngle(float angle, float lowerLimit, float upperLimit) {
		//
		return Mathf.Max(Mathf.Min(angle, upperLimit), lowerLimit);
	}

	// Use this for initialization
	void Start () {	
		
	}

	// Update is called once per frame
	void Update () {	
		
	}
}
