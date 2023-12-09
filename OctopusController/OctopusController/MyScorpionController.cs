using System;
using System.Collections.Generic;
using System.Diagnostics.SymbolStore;
using System.Linq;
using System.Reflection;
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
        private const float MAXIMUM_DISTANCE_BETWEEN_CURRENT_BASE_TO_FUTURE_BASE_THRESHOLD = 0.2f;
        private const float MINIMUM_DISTANCE_BETWEEN_CURRENT_BASE_TO_FUTURE_BASE_THRESHOLD = 0.01f;
        private const float DISTANCE_FROM_FOOT_TO_CURRENT_BASE_THRESHOLD = 0.001f;
        private const float DISTANCE_FROM_SHOULDER_TO_TARGET_THRESHOLD = 0.001f;
        private const float TIME_TO_MOVE_LEG = 0.2f;
        private const float MAXIMUM_HEIGHT_TO_FOOT = 0.3f;
        private const int MAXIMUM_ITERATIONS_FABRIK = 20;


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

        Vector3[] _currentFeetBases;
        Vector3[] _lastFeetBases;

        float[][] _distanceBetweenJoints;
        float[] _legsLength;
        float[] _legsPositionZLerp;
        float[] _legsPositionYLerp;

        bool[] _legsReachCeil;
        bool[] _moveLegs;
        bool _startLegsAnimation;

        bool active = true;

        public bool UpdateIK()
        {
            if (!active)
            {
                return active;
            }

            if (_startLegsAnimation)
            {
                UpdateLegs();
            }
            if (!_startTailAnimation)
            {
                return active;
            }
            UpdateTail();
            return active;
        }

        #region Legs

        public void InitLegs(Transform[] LegRoots, Transform[] LegFutureBases, Transform[] LegTargets)
        {
            _legs = new MyTentacleController[LegRoots.Length];
            _legFutureBases = new Transform[LegFutureBases.Length];
            _legTargets = new Transform[LegTargets.Length];
            _legsLength = new float[LegRoots.Length];
            _distanceBetweenJoints = new float[LegRoots.Length][];
            _currentFeetBases = new Vector3[LegFutureBases.Length];
            _lastFeetBases = new Vector3[LegFutureBases.Length];
            _legsPositionZLerp = new float[LegRoots.Length];
            _legsPositionYLerp = new float[LegRoots.Length];
            _legsReachCeil = new bool[LegRoots.Length];
            _moveLegs = new bool[LegRoots.Length];
            //Legs init

            for (int i = 0; i < LegRoots.Length; i++)
            {
                InitializeLegsValues(LegRoots, LegFutureBases, LegTargets, i);
                if (i % 2 == 0)
                {
                    continue;
                }
                THIS_FUCKING_SCENE_IS_A_SHIT(_legs[i]);
            }
        }

        private void InitializeLegsValues(Transform[] LegRoots, Transform[] LegFutureBases, Transform[] LegTargets, int index)
        {
            _legs[index] = new MyTentacleController();
            _legs[index].LoadTentacleJoints(LegRoots[index].GetChild(0), TentacleMode.LEG);
            _legFutureBases[index] = LegFutureBases[index];
            _legTargets[index] = LegTargets[index];
            _distanceBetweenJoints[index] = new float[_legs[index].Bones.Length];
            _legsPositionZLerp[index] = 0;
            _legsPositionYLerp[index] = 0;
            _currentFeetBases[index] = _legs[index].Bones[0].position;

            Transform[] bones = _legs[index].Bones;

            for (int i = 0; i < bones.Length - 1; i++)
            {
                _distanceBetweenJoints[index][i] = (bones[i].position - bones[i + 1].position).magnitude;
                _legsLength[index] += _distanceBetweenJoints[index][i];
            }

            int bonesLastIndex = bones.Length - 1;

            _distanceBetweenJoints[index][bonesLastIndex] = (bones[bonesLastIndex].position - _legs[index].EndEffector.position).magnitude;
            _legsLength[index] += _distanceBetweenJoints[index][bonesLastIndex];
            
        }

        private void THIS_FUCKING_SCENE_IS_A_SHIT(MyTentacleController leg)
        {
            leg.Bones[0].rotation = Rotate(leg.Bones[0].rotation, RotationY, 180);
                
            leg.Bones[1].rotation = Rotate(leg.Bones[1].rotation, RotationY, 180);
            leg.Bones[1].rotation = Rotate(leg.Bones[1].rotation, RotationZ, 60);
        }

        //TODO: Notifies the start of the walking animation
        public void NotifyStartWalk()
        {
            _startLegsAnimation = true;
        }

        //TODO: implement fabrik method to move legs 
        private void UpdateLegs()
        {
            int legToCheckMovement;
            int auxInt;
            
            for (int i = 0; i < _legs.Length; i++)
            {
                auxInt = i % 2;
                if (auxInt == 0)
                {
                    auxInt = 1;
                }
                else
                {
                    auxInt = -1;
                }

                legToCheckMovement = i + auxInt;

                if (!_moveLegs[legToCheckMovement])
                {
                    if ((_legFutureBases[i].position - _currentFeetBases[i]).magnitude > MAXIMUM_DISTANCE_BETWEEN_CURRENT_BASE_TO_FUTURE_BASE_THRESHOLD && 
                        !_moveLegs[i])
                    {
                        _lastFeetBases[i] = _currentFeetBases[i];
                        _moveLegs[i] = true;    
                    }
                
                    if (_moveLegs[i])
                    {
                        _legsPositionZLerp[i] += Time.deltaTime;

                        if (!_legsReachCeil[i])
                        {
                            _legsReachCeil[i] = _legsPositionYLerp[i] >= TIME_TO_MOVE_LEG;
                        }

                        if (_legsReachCeil[i])
                        {
                            _legsPositionYLerp[i] -= Time.deltaTime * 2;   
                        }
                        else
                        {
                            _legsPositionYLerp[i] += Time.deltaTime * 2;
                        }
                    
                        _currentFeetBases[i].z = Mathf.Lerp(_lastFeetBases[i].z, _legFutureBases[i].position.z, _legsPositionZLerp[i] / TIME_TO_MOVE_LEG);
                        _currentFeetBases[i].y = Mathf.Lerp(_lastFeetBases[i].y,  _lastFeetBases[i].y + MAXIMUM_HEIGHT_TO_FOOT, _legsPositionYLerp[i] / TIME_TO_MOVE_LEG);

                        if ((_legFutureBases[i].position - _currentFeetBases[i]).magnitude < MINIMUM_DISTANCE_BETWEEN_CURRENT_BASE_TO_FUTURE_BASE_THRESHOLD)
                        {
                            _currentFeetBases[i] = _legFutureBases[i].position;
                            _legsPositionZLerp[i] = 0;
                            _legsPositionYLerp[i] = 0;
                            _legsReachCeil[i] = false;
                            _moveLegs[i] = false;
                        }
                    }
                }
                
                UpdateLegPos(_legs[i].Bones, _legs[i].EndEffector, i);
            }
        }

        private bool DidFootReachDestination(Transform foot, int index)
        {
            return (foot.position - _currentFeetBases[index]).magnitude <
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
            for (int i = 0; i < MAXIMUM_ITERATIONS_FABRIK; i++)
            {
                FABRIK(joints, index);
                if (!DidFootReachDestination(joints[0], index) || !DidShoulderReachDestination(endEffector, index))
                {
                    continue;
                }
                break;
            }
        }

        private void FABRIK(Transform[] joints, int index)
        {
            Vector3[] virtualPositions = new Vector3[joints.Length + 1];

            CalculateBonesForward(joints, virtualPositions, index);

            CalculateBonesBackwards(virtualPositions, index);

            MoveBones(joints, virtualPositions, index);
        }

        private void CalculateBonesForward(Transform[] joints, Vector3[] virtualPositions, int index)
        {
            virtualPositions[0] = _currentFeetBases[index];

            Vector3 vectorToNextJoint;
            Vector3 auxJointPosition;

            for (int i = 0; i < joints.Length - 1; i++)
            {
                auxJointPosition = virtualPositions[i];
                vectorToNextJoint = (joints[i + 1].position - auxJointPosition).normalized;
                virtualPositions[i + 1] = auxJointPosition + vectorToNextJoint * _distanceBetweenJoints[index][i];
            }
        }

        private void CalculateBonesBackwards(Vector3[] virtualPositions, int index)
        {
            virtualPositions[virtualPositions.Length - 1] = _legTargets[index].position;

            Vector3 vectorToPreviousJoint;
            Vector3 auxJointPosition;

            for (int i = virtualPositions.Length - 1; i > 0; i--)
            {
                auxJointPosition = virtualPositions[i];
                vectorToPreviousJoint = (virtualPositions[i - 1] - auxJointPosition).normalized;
                virtualPositions[i - 1] = auxJointPosition + vectorToPreviousJoint * _distanceBetweenJoints[index][i - 1];
            }
        }

        private void MoveBones(Transform[] joints, Vector3[] virtualPositions, int index)
        {
            joints[0].position = virtualPositions[0];

            Vector3 vectorBetweenVirtualPositions;
            Vector3 vectorBetweenVirtualPositionsNormalized;
            Vector3 crossVector;
            
            float cosine;
            float angle;

            for (int i = 0; i < virtualPositions.Length - 1; i++)
            {
                vectorBetweenVirtualPositions = virtualPositions[i + 1] - virtualPositions[i];
                vectorBetweenVirtualPositionsNormalized = vectorBetweenVirtualPositions.normalized;
                cosine = Vector3.Dot(joints[i].up, vectorBetweenVirtualPositionsNormalized);
                angle = Rad2Deg(Mathf.Acos(cosine));
                crossVector = Vector3.Cross(joints[i].up, vectorBetweenVirtualPositionsNormalized).normalized;

                joints[i].rotation = Quaternion.AngleAxis(angle, crossVector) * joints[i].rotation;
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
            Quaternion[] auxRotations = new Quaternion[bones.Length];

            for (int i = 0; i < bones.Length; i++)
            {
                auxRotations[i] = bones[i].rotation;
            }

            _tailJointsAxisRotation[0] = RotationZ;
            _tailCurrentJointRotations[0] = bones[0].localEulerAngles.z;
            _tail.Bones[0].rotation = Quaternion.identity;

            for (int i = 1; i < bones.Length; i++)
            {
                _tailJointsAxisRotation[i] = RotationX;
                _tailCurrentJointRotations[i] = bones[i].localEulerAngles.x;
                _tail.Bones[i].rotation = Quaternion.identity;
                _tailJointsRelativePositions[i - 1] = bones[i].position - bones[i - 1].position;
            }

            _tailCurrentEndEffectorPosition = _tailEndEffector.position;
            _tailJointsRelativePositions[_tailJointsRelativePositions.Length - 1] = _tailEndEffector.position - bones[bones.Length - 1].position;

            for (int i = 0; i < bones.Length; i++)
            {
                bones[i].rotation = auxRotations[i];
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
                    return;
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
            Quaternion rotation = _tail.Bones[0].rotation;

            for (int i = 0; i < _tail.Bones.Length; i++)
            {
                rotation *= Quaternion.AngleAxis(_tailCurrentJointRotations[i], _tailJointsAxisRotation[i]);
                _tail.Bones[i].rotation = rotation;
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
            Quaternion rotation = _tail.Bones[0].rotation;

            for (int i = 1; i < _tail.Bones.Length; i++)
            {
                rotation *= Quaternion.AngleAxis(_tailCurrentJointRotations[i - 1], _tailJointsAxisRotation[i - 1]);
                _tailCurrentEndEffectorPosition += rotation * _tailJointsRelativePositions[i];
            }
        }

        #endregion

        #region Utils

        public float Map(float value, float originalMin, float originalMax, float newMin, float newMax)
        {
            return newMin + (value - originalMin) * (newMax - newMin) / (originalMax - originalMin);
        }

        internal float Deg2Rad(float angle)
        {
            return angle * ((float)Math.PI / 180f);
        }

        internal float Rad2Deg(float angle)
        {
            return angle * (180f / (float)Math.PI);
        }

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
        
        private static Vector3 RotationY
        {
            get
            {
                Vector3 vector;
                vector.x = 0;
                vector.y = 1;
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
        
        internal Quaternion Rotate(Quaternion currentRotation, Vector3 axis, float angle)
        {
            angle /= 2;

            Quaternion quaternionRotationZ = Quaternion.identity;
            quaternionRotationZ.w = (float)Math.Cos(Deg2Rad(angle) * axis.z);
            quaternionRotationZ.z = (float)Math.Sin(Deg2Rad(angle) * axis.z);

            Quaternion quaternionRotationY = Quaternion.identity;
            quaternionRotationY.w = (float)Math.Cos(Deg2Rad(angle) * axis.y);
            quaternionRotationY.y = (float)Math.Sin(Deg2Rad(angle) * axis.y);

            Quaternion quaternionRotationX = Quaternion.identity;
            quaternionRotationX.w = (float)Math.Cos(Deg2Rad(angle) * axis.x);
            quaternionRotationX.x = (float)Math.Sin(Deg2Rad(angle) * axis.x);

            Quaternion result = quaternionRotationZ * quaternionRotationX * quaternionRotationY;

            return currentRotation * result;
        }

        #endregion
    }
}