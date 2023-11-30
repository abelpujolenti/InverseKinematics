using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;




namespace OctopusController
{

    
    internal class MyTentacleController

    //MAINTAIN THIS CLASS AS INTERNAL
    {

        TentacleMode tentacleMode;
        Transform[] _bones;
        Transform _endEffectorSphere;

        public Transform[] Bones { get => _bones; }
        public Transform EndEffector => _endEffectorSphere;

        //Exercise 1.
        public Transform[] LoadTentacleJoints(Transform root, TentacleMode mode)
        {
            
            //TODO: add here whatever is needed to find the bones forming the tentacle for all modes
            //you may want to use a list, and then convert it to an array and save it into _bones
            tentacleMode = mode;

            switch (tentacleMode){
                case TentacleMode.LEG:

                    LoadJoints(2, root);                    
                    break;
                
                case TentacleMode.TAIL:

                    LoadJoints(4, root);
                    break;
                
                case TentacleMode.TENTACLE:
                    //TODO: in _endEffectorphere you  keep a reference to the sphere with a collider attached to the endEffector
                    break;
            }
            return Bones;
        }

        private void LoadJoints(int jointsLength, Transform root)
        {
            List<Transform> transformList = new List<Transform> { root };
            
            for (int i = 0; i < jointsLength; i++)
            {
                transformList.Add(transformList[i].GetChild(1));
                Debug.Log(transformList[i]);
            }
            Debug.Log(transformList[transformList.Count - 1]);
            _endEffectorSphere = transformList[transformList.Count - 1].GetChild(1);
            Debug.Log(_endEffectorSphere);
                    
            _bones = transformList.ToArray();
        }
    }
}
