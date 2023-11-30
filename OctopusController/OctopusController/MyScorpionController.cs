using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;


namespace OctopusController
{
    public struct MyVec
    {

        public float x;
        public float y;
        public float z;
    }
    
    public struct MyQuat
    {
        public float w;
        public float x;
        public float y;
        public float z;
        public static MyQuat operator* (MyQuat q1, MyQuat q2) 
        {
            MyQuat result;
            result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
            result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
            result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
            result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
            
            return result;
        }
    }
  
    public class MyScorpionController
    {
        private const float DELTA = 0.05f;
        private const float STEP = 100;
        private const float DISTANCE_TO_TARGET_THRESHOLD = 1; 
        
        //TAIL
        MyTentacleController _tail;
        
        Transform _tailTarget;
        Transform _tailEndEffector;
        
        Vector3[] _tailJointsAxisRotation;
        
        float[] _tailCurrentJointRotation;
        float[] _tailVirtualJointRotation;
        
        float _tailSize;
        
        Vector3[] _tailJointsRelativePositions;
        
        Vector3 _tailCurrentEndEffectorPosition;

        //LEGS
        Transform[] _legTargets;
        Transform[] _legFutureBases;
        MyTentacleController[] _legs = new MyTentacleController[6];

        private bool enter;
        private bool _startLegsAnimation;
        private bool _startTailAnimation;

        
        #region public
        public void InitLegs(Transform[] LegRoots,Transform[] LegFutureBases, Transform[] LegTargets)
        {
            _legs = new MyTentacleController[LegRoots.Length];
            _legFutureBases = new Transform[LegFutureBases.Length];
            _legTargets = new Transform[LegTargets.Length];
            //Legs init
            for (int i = 0; i < LegRoots.Length; i++)
            {
                _legs[i] = new MyTentacleController();
                _legs[i].LoadTentacleJoints(LegRoots[i].GetChild(0), TentacleMode.LEG);                
                _legFutureBases[i] = LegFutureBases[i];
                _legTargets[i] = LegTargets[i];
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
            _startLegsAnimation = false;
        }

        //TODO: Notifies the start of the walking animation
        public void NotifyStartWalk()
        {
            _startLegsAnimation = true;
        }

        //TODO: create the apropiate animations and update the IK from the legs and tail
        public void UpdateIK()
        {            
            if (_startLegsAnimation)
            {
                UpdateLegs();
            }
            if (_startTailAnimation)
            {
                UpdateTail();
            }            
        }
        #endregion
        
        //TODO: implement fabrik method to move legs 
        private void UpdateLegs()
        {
            UpdateLegPos();
        }

        //TODO: Implement the leg base animations and logic
        private void UpdateLegPos()
        {
            //check for the distance to the futureBase, then if it's too far away start moving the leg towards the future base position
            //
        }
        
        public void InitTail(Transform TailBase)
        {
            _tail = new MyTentacleController();
            _tail.LoadTentacleJoints(TailBase, TentacleMode.TAIL);
            
            Transform[] bones = _tail.Bones;
            
            _tailJointsAxisRotation = new Vector3[bones.Length];
            _tailCurrentJointRotation = new float[bones.Length];
            _tailVirtualJointRotation = new float[bones.Length];
            _tailJointsRelativePositions = new Vector3[bones.Length];

            _tailJointsAxisRotation[0] = RotationZ;
            _tailCurrentJointRotation[0] = bones[0].localEulerAngles.z;
            
            for (int i = 1; i < bones.Length; i++)
            {
                _tailSize += (bones[i].position - bones[i - 1].position).magnitude;
                _tailJointsAxisRotation[i] = RotationX;
                _tailCurrentJointRotation[i] = bones[i].localEulerAngles.x;
                _tailJointsRelativePositions[i - 1] = bones[i].position - bones[i - 1].position;
            }

            _tailEndEffector = _tail.EndEffector;
            _tailJointsRelativePositions[_tailJointsRelativePositions.Length - 1] = 
                _tailEndEffector.position - bones[bones.Length - 1].position;
            _tailCurrentEndEffectorPosition = _tailEndEffector.position;
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
                _tailCurrentJointRotation[i] -= STEP * _tailVirtualJointRotation[i];
            }
        }

        private void UpdateJoints()
        {
            Quaternion rotation = Quaternion.identity;
            for (int i = 0; i < _tail.Bones.Length; i++)
            {
                rotation = Rotate(rotation, _tailJointsAxisRotation[i], _tailCurrentJointRotation[i]);
                _tail.Bones[i].rotation = rotation;
            }
        }

        private void GradientDescent(int index)
        {
            float currentDistanceBetweenEndEffectorAndTarget = ErrorFunction();
            
            float currentAngle = _tailCurrentJointRotation[index];
            _tailCurrentJointRotation[index] += DELTA;
            
            float nextDistanceBetweenEndEffectorAndTarget = ErrorFunction();

            _tailCurrentJointRotation[index] = currentAngle;
            
            float gradient = (currentDistanceBetweenEndEffectorAndTarget - nextDistanceBetweenEndEffectorAndTarget) /
                             DELTA;

            _tailVirtualJointRotation[index] = gradient;

        }

        private float ErrorFunction()
        {
            ForwardKinematics();
            return (_tailCurrentEndEffectorPosition - _tailTarget.position).magnitude;
        }

        private void ForwardKinematics()
        {
            Transform rootBone = _tail.Bones[0];
            _tailCurrentEndEffectorPosition = rootBone.position;
            Quaternion rotation = rootBone.rotation;

            for (int i = 0; i < _tail.Bones.Length - 1; i++)
            {
                rotation *= Quaternion.AngleAxis(_tailCurrentJointRotation[i], _tailJointsAxisRotation[i]);
                _tailCurrentEndEffectorPosition += rotation * _tailJointsRelativePositions[i];
            }
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
        

        internal float Deg2Rad(float angle)
        {
            return angle * ((float)Math.PI / 180f);
        }

        internal float Rad2Deg(float angle)
        {
            return angle * (180f / (float)Math.PI);
        }
    
        internal static MyQuat InverseQuaternion(MyQuat quaternion)
        {
            MyQuat result;

            result.w = quaternion.w;
            result.x = -quaternion.x;
            result.y = -quaternion.y;
            result.z = -quaternion.z;

            return result;
        }

        internal static MyQuat NormalizeQuaternion(MyQuat quaternion)
        {
            float length = (float)Math.Sqrt(Math.Pow(quaternion.w, 2f) + Math.Pow(quaternion.x, 2f) + Math.Pow(quaternion.y, 2f) + Math.Pow(quaternion.z, 2f));

            quaternion.w /= length;
            quaternion.x /= length;
            quaternion.y /= length;
            quaternion.z /= length;

            return quaternion;
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

        internal Vector3 AxisAnglesFromQuaternion(Quaternion quaternion)
        {
            Vector3 axisAngles;

            if (quaternion.w > 1)
            {
                quaternion.Normalize();
            }
            float angle = (float)(2 * Math.Acos(quaternion.w));
            double s = Math.Sqrt(1 - quaternion.w * quaternion.w);
            if (s < 0.001)
            {
                axisAngles.x = quaternion.x;
                axisAngles.y = quaternion.y;
                axisAngles.z = quaternion.z;
            }
            else
            {
                axisAngles.x = (float)(quaternion.x / s);
                axisAngles.y = (float)(quaternion.y / s);
                axisAngles.z = (float)(quaternion.z / s);
            }
            return axisAngles;
        }
    }
}