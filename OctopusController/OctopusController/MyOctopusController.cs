using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using static UnityEngine.GraphicsBuffer;


namespace OctopusController
{
    public enum TentacleMode { LEG, TAIL, TENTACLE };

    public class MyOctopusController
    {

        MyTentacleController[] _tentacles = new MyTentacleController[4];

        Transform _currentRegion;
        Transform _target;

        Transform[] _randomTargets;// = new Transform[4];
        Vector3[] _targetOffsets;

        float _twistMin, _twistMax;
        float _swingMin, _swingMax;

        #region public methods
        //DO NOT CHANGE THE PUBLIC METHODS!!

        public float TwistMin { set => _twistMin = value; }
        public float TwistMax { set => _twistMax = value; }
        public float SwingMin { set => _swingMin = value; }
        public float SwingMax { set => _swingMax = value; }


        public void TestLogging(string objectName)
        {


            Debug.Log("hello, I am initializing my Octopus Controller in object " + objectName);


        }

        public void Init(Transform[] tentacleRoots, Transform[] randomTargets)
        {
            Transform[] actualTentacleRoots = new Transform[tentacleRoots.Length];
            for (int i = 0; i < tentacleRoots.Length; i++)
            {
                actualTentacleRoots[i] = tentacleRoots[i].GetChild(0).GetChild(0);
            }
            tentacleRoots = actualTentacleRoots;
            _tentacles = new MyTentacleController[tentacleRoots.Length];
            _targetOffsets = new Vector3[tentacleRoots.Length];

            _randomTargets = randomTargets;

            // foreach (Transform t in tentacleRoots)
            for (int i = 0; i < tentacleRoots.Length; i++)
            {

                _tentacles[i] = new MyTentacleController();
                _tentacles[i].LoadTentacleJoints(tentacleRoots[i], TentacleMode.TENTACLE);
                _targetOffsets[i] = _tentacles[i].Bones[_tentacles[i].Bones.Length - 1].position - randomTargets[i].position;
                //TODO: initialize any variables needed in ccd
            }
            //TODO: use the regions however you need to make sure each tentacle stays in its region

        }


        public void NotifyTarget(Transform target, Transform region)
        {
            _currentRegion = region;
            _target = target;
        }

        public void NotifyShoot()
        {
            //TODO. what happens here?
            Debug.Log("Shoot");
        }


        public void UpdateTentacles()
        {
            //TODO: implement logic for the correct tentacle arm to stop the ball and implement CCD method
            update_ccd();
        }

        #endregion


        #region private and internal methods
        //todo: add here anything that you need

        void update_ccd()
        {

            for (int i = 0; i < _tentacles.Length; i++)
            {
                for (int t = _tentacles[i].Bones.Length - 2; t >= 0; t--)
                {
                    Transform currentBone = _tentacles[i].Bones[t];
                    Transform tipBone = _tentacles[i].Bones[_tentacles[i].Bones.Length - 1];
                    Vector3 target = _randomTargets[i].position + _targetOffsets[i];


                    Vector3 currentToTip = (tipBone.position - currentBone.position).normalized;
                    Vector3 currentToTarget = (target - currentBone.position).normalized;

                    Quaternion q = Quaternion.FromToRotation(currentToTip, currentToTarget);
                    currentBone.rotation = q * currentBone.rotation;
                }
            }
        }

        #endregion
    }
}
