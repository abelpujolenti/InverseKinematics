using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;


namespace OctopusController
{
  
    public class MyScorpionController
    {
        //TAIL CONSTS
        private const float DELTA = 0.05f;
        private const float SPEED = 10;
        private const float DISTANCE_TO_TARGET_THRESHOLD = 0.05f;
        
        //LEGS CONSTS
        private const float DISTANCE_FROM_SHOLUDER_TO_CURRENT_BASE_THRESHOLD = 5;
        private const float DISTANCE_FROM_FOOT_TO_CURRENT_BASE_THRESHOLD = 0.05f;
        private const float DISTANCE_FROM_SHOULDER_TO_TARGET_THRESHOLD = 0.05f;
        
        
        //TAIL
        MyTentacleController _tail;
        
        Transform _tailTarget;
        Transform _tailEndEffector;
        
        Vector3[] _tailJointsAxisRotation;
        
        float[] _tailCurrentJointRotations;
        float[] _tailVirtualJointRotations;
        
        float _tailSize;
        
        Vector3[] _tailJointsRelativePositions;
        
        Vector3 _tailCurrentEndEffectorPosition;
        
        bool _startTailAnimation;

        //LEGS
        MyTentacleController[] _legs = new MyTentacleController[6];
        
        Transform[] _legTargets;
        Transform[] _legFutureBases;

        float[][] _distanceBetweenJoints;
        float[] _legsLength;

        
        bool _startLegsAnimation;


        //TODO: create the apropiate animations and update the IK from the legs and tail
        public void UpdateIK()
        {
            if (_startLegsAnimation)
            {
                UpdateLegs();    
            }
            if (!_startTailAnimation)
            {
                return;
            }   
            //UpdateTail();
        }

        #region Legs
        
        public void InitLegs(Transform[] LegRoots,Transform[] LegFutureBases, Transform[] LegTargets)
        {
            _legs = new MyTentacleController[LegRoots.Length];
            _legFutureBases = new Transform[LegFutureBases.Length];
            _legTargets = new Transform[LegTargets.Length];
            _legsLength = new float[LegTargets.Length];
            _distanceBetweenJoints = new float[LegTargets.Length][];
            //Legs init
            
            for (int i = 0; i < LegRoots.Length; i++)
            {
                Debug.Log(i);
                InitializeLegsValues(LegRoots, LegFutureBases, LegTargets,i);
            }
        }

        private void InitializeLegsValues(Transform[] LegRoots , Transform[] LegFutureBases, Transform[] LegTargets, int index)
        {
            _legs[index] = new MyTentacleController();
            _legs[index].LoadTentacleJoints(LegRoots[index].GetChild(0), TentacleMode.LEG);                
            _legFutureBases[index] = LegFutureBases[index];
            _legTargets[index] = LegTargets[index];

            Transform[] bones = _legs[index].Bones;
            
            _distanceBetweenJoints[index] = new float[bones.Length];
            
            for (int i = 0; i < bones.Length - 2; i++)
            {
                _distanceBetweenJoints[index][i] = (bones[i].position - bones[i + 1].position).magnitude;
                _legsLength[index] += _distanceBetweenJoints[index][i];
            }

            int bonesLastIndex = bones.Length - 1;
            
            _distanceBetweenJoints[index][bonesLastIndex] = (bones[bonesLastIndex].position - _legs[index].EndEffector.position).magnitude;
            _legsLength[index] += _distanceBetweenJoints[index][bonesLastIndex];
        }

        //TODO: Notifies the start of the walking animation
        public void NotifyStartWalk()
        {
            _startLegsAnimation = true;
        }
        
        //TODO: implement fabrik method to move legs 
        private void UpdateLegs()
        {
            for (int i = 0; i < 1; i++)
            {
                Debug.Log(_legsLength[i]);
                if ((_legTargets[i].position - _legFutureBases[i].position).magnitude <= _legsLength[i])
                {
                    Debug.Log("Short");
                    return;
                }
                UpdateLegPos(_legs[i].Bones, _legs[i].EndEffector, i);
            }
        }

        private bool DidFootReachDestination(Transform foot, int index)
        {
            return (foot.position - _legFutureBases[index].position).magnitude <
                   DISTANCE_FROM_FOOT_TO_CURRENT_BASE_THRESHOLD;
        }

        private bool DidShoulderReachDestination(Transform shoulder, int index)
        {
            return (shoulder.position - _legTargets[index].position).magnitude < 
                   DISTANCE_FROM_SHOULDER_TO_TARGET_THRESHOLD;
        }

        //TODO: Implement the leg base animations and logic
        private void UpdateLegPos(Transform[] joints, Transform endEffector, int index)
        {
            while (!DidFootReachDestination(joints[0], index) || !DidShoulderReachDestination(endEffector, index))
            {
                FABRIK(joints, endEffector, index);    
            }
            
            //check for the distance to the futureBase, then if it's too far away start moving the leg towards the future base position
            //
        }

        private void FABRIK(Transform[] joints, Transform endEffector, int index)
        {
            MoveBonesForwards(joints, endEffector, index);
            
            MoveBonesBackwards(joints, endEffector, index);
        }

        private void MoveBonesForwards(Transform[] joints, Transform endEffector, int index)
        {
            joints[0].position = _legFutureBases[index].position;

            Vector3 vectorToNextJoint;
            Vector3 jointPosition;

            for (int i = 0; i < joints.Length - 2; i++)
            {
                jointPosition = joints[i].position;
                vectorToNextJoint = (joints[i + 1].position - jointPosition).normalized;
                joints[i + 1].position = jointPosition + vectorToNextJoint * _distanceBetweenJoints[index][i];
            }

            jointPosition = joints[joints.Length - 1].position;
            
            vectorToNextJoint = (endEffector.position - jointPosition).normalized;
            endEffector.position = jointPosition + vectorToNextJoint *
                _distanceBetweenJoints[index][_distanceBetweenJoints[index].Length - 1];
        }

        private void MoveBonesBackwards(Transform[] joints, Transform endEffector, int index)
        {
            endEffector.position = _legTargets[index].position;
            
            Vector3 jointPosition = joints[joints.Length - 1].position;
            Vector3 vectorToPreviousJoint = (jointPosition - endEffector.position).normalized;
            joints[joints.Length - 1].position = endEffector.position + vectorToPreviousJoint *
                _distanceBetweenJoints[index][_distanceBetweenJoints[index].Length - 1];

            for (int i = joints.Length - 1; i > 1; i--)
            {
                jointPosition = joints[i].position;
                vectorToPreviousJoint = (joints[i - 1].position - jointPosition).normalized;
                joints[i - 1].position = jointPosition + vectorToPreviousJoint * _distanceBetweenJoints[index][i];
            }

        }

        #endregion
        
        #region Tail
        public void InitTail(Transform TailBase)
        {
            _tail = new MyTentacleController();
            _tail.LoadTentacleJoints(TailBase, TentacleMode.TAIL);
            
            Transform[] bones = _tail.Bones;
            
            _tailJointsAxisRotation = new Vector3[bones.Length];
            _tailCurrentJointRotations = new float[bones.Length];
            _tailVirtualJointRotations = new float[bones.Length];
            _tailJointsRelativePositions = new Vector3[bones.Length];
            
            _tailEndEffector = _tail.EndEffector;
            
            InitializeTailValues(bones);
        }
        
        private void InitializeTailValues(Transform[] bones)
        {
            _tailJointsAxisRotation[0] = RotationZ;
            
            //// DANGER ZONE
            
            //_tailCurrentJointRotations[0] = bones[0].localEulerAngles.z;
            _tail.Bones[0].rotation = Quaternion.identity;
            Debug.Log(_tailCurrentJointRotations[0]);
            
            /////
            for (int i = 1; i < bones.Length; i++)
            {
                _tailJointsAxisRotation[i] = RotationX;
                
                /// DANGER ZONE
                
                //_tailCurrentJointRotations[i] = bones[i].localEulerAngles.x;
                _tail.Bones[i].rotation = Quaternion.identity;
                Debug.Log(_tailCurrentJointRotations[i]);
                
                /////
                _tailJointsRelativePositions[i - 1] = bones[i].position - bones[i - 1].position;
            }
            
            _tailCurrentEndEffectorPosition = _tailEndEffector.position;
            _tailJointsRelativePositions[_tailJointsRelativePositions.Length - 1] = _tailEndEffector.position - bones[bones.Length - 1].position;

            for (int i = 0; i < bones.Length; i++)
            {
                _tailSize += _tailJointsRelativePositions[i].magnitude;
            }
        }

        //TODO: Check when to start the animation towards target and implement Gradient Descent method to move the joints.
        public void NotifyTailTarget(Transform target)
        {
            if (Vector3.Distance(_tail.Bones[0].position, target.position) < _tailSize)
            {
                _startTailAnimation = true;
                _tailTarget = target;
                return;
            }
            _startTailAnimation = false;
        }

        //TODO: implement Gradient Descent method to move tail if necessary
        private void UpdateTail()
        {
            for (int i = 0; i < _tail.Bones.Length; i++)
            {
                if ((_tailTarget.position - _tailCurrentEndEffectorPosition).magnitude <= DISTANCE_TO_TARGET_THRESHOLD)
                {
                    continue;
                }
                
                CalculateTailJointRotations();
                ApplyTailJointRotations();
            }
            UpdateJoints();
        }

        private void CalculateTailJointRotations()
        {
            for (int i = 0; i < _tail.Bones.Length; i++)
            { 
                GradientDescent(i);
            }
        }

        private void ApplyTailJointRotations()
        {
            for (int i = 0; i < _tail.Bones.Length; i++)
            {
                _tailCurrentJointRotations[i] -= SPEED * _tailVirtualJointRotations[i];
            }
        }

        private void UpdateJoints()
        {
            //// DANGER ZONE
            
            //Quaternion rotation = Quaternion.identity;
            Quaternion rotation = _tail.Bones[0].rotation;
            
            ////
            for (int i = 0; i < _tail.Bones.Length; i++)
            {
                //// DANGER ZONE
                
                //rotation = _tail.Bones[i].localRotation;
                rotation *= Quaternion.AngleAxis(_tailCurrentJointRotations[i], _tailJointsAxisRotation[i]);
                _tail.Bones[i].rotation = rotation;
                
                /////
            }
        }

        private void GradientDescent(int index)
        {
            float currentDistanceBetweenEndEffectorAndTarget = ErrorFunction();
            float currentAngle = _tailCurrentJointRotations[index];
            _tailCurrentJointRotations[index] += DELTA;
            float nextDistanceBetweenEndEffectorAndTarget = ErrorFunction();
            _tailCurrentJointRotations[index] = currentAngle;
            
            float gradient = (nextDistanceBetweenEndEffectorAndTarget - currentDistanceBetweenEndEffectorAndTarget) / DELTA;

            _tailVirtualJointRotations[index] = gradient;
        }

        private float ErrorFunction()
        {
            ForwardKinematics();
            return (_tailCurrentEndEffectorPosition - _tailTarget.position).magnitude;
        }

        private void ForwardKinematics()
        {
            _tailCurrentEndEffectorPosition = _tail.Bones[0].position;
            
            //// DANGER ZONE
            
            //Quaternion rotation = Quaternion.identity;
            Quaternion rotation = _tail.Bones[0].rotation;
            
            /////
            for (int i = 1; i < _tail.Bones.Length; i++)
            {
                rotation *= Quaternion.AngleAxis(_tailCurrentJointRotations[i - 1], _tailJointsAxisRotation[i - 1]);
                _tailCurrentEndEffectorPosition += rotation * _tailJointsRelativePositions[i];
                //_tail.Bones[i].position = _tailCurrentEndEffectorPosition;
            }
            
            //_tail.EndEffector.position = _tailCurrentEndEffectorPosition;
        }

        #endregion
        
        private static Vector3 RotationX
        {
            get
            {
                Vector3 vector;
                vector.x = 1;
                vector.y = 0;
                vector.z = 0;
                return vector;
            }
        }
        
        private static Vector3 RotationZ
        {
            get
            {
                Vector3 vector;
                vector.x = 0;
                vector.y = 0;
                vector.z = 1;
                return vector;
            }
        }
    }
}