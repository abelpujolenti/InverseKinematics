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
        //TAIL
        Transform _tailTarget;
        Transform _tailEndEffector;
        MyTentacleController _tail;
        MyVec[] _jointsAxisRotation;
        float[] _initialJointRotation;
        float[] _currentJointRotation;
        float _tailSize;
        float _animationRange;
        Vector3 _currentEndEffectorPosition;

        //LEGS
        Transform[] _legTargets ;
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

        public void InitTail(Transform TailBase)
        {
            _tail = new MyTentacleController();
            _tail.LoadTentacleJoints(TailBase, TentacleMode.TAIL);
            Transform[] bones = _tail.Bones;
            _jointsAxisRotation = new MyVec[bones.Length];
            _initialJointRotation = new float[bones.Length];
            _currentJointRotation = new float[_initialJointRotation.Length];

            _jointsAxisRotation[0] = RotationZ;
            _initialJointRotation[0] = bones[0].localEulerAngles.z;
            _currentJointRotation[0] = _initialJointRotation[0];
            for (int i = 1; i < bones.Length; i++)
            {
                _tailSize += (bones[i].position - bones[i - 1].position).magnitude;
                _jointsAxisRotation[i] = RotationX;
                _initialJointRotation[i] = bones[i].localEulerAngles.x;
                _currentJointRotation[i] = _initialJointRotation[i];
            }
            _tailEndEffector = bones[bones.Length - 1];
            _currentEndEffectorPosition = _tailEndEffector.position;
        }

        //TODO: Check when to start the animation towards target and implement Gradient Descent method to move the joints.
        public void NotifyTailTarget(Transform target)
        {
            if (Vector3.Distance(_tail.Bones[0].position, target.position) < _tailSize)
            {
                _startTailAnimation = true;
                _tailTarget = target;    
            }
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
                updateLegs();
            }
            if (_startTailAnimation)
            {
                updateTail();
            }            
        }
        #endregion

        #region private
        //TODO: Implement the leg base animations and logic
        private void updateLegPos()
        {
            //check for the distance to the futureBase, then if it's too far away start moving the leg towards the future base position
            //
        }
        
        //TODO: implement Gradient Descent method to move tail if necessary
        private void updateTail()
        {
            Transform[] bones = _tail.Bones;

            for (int i = 0; i < bones.Length; i++)
            {
                float initialStep = 1;

                bool closer;

                do
                {

                    GradientDescent(bones[i], _jointsAxisRotation[i], _currentJointRotation[i] + initialStep);
                    /*for (int j = i + 1; j < bones.Length; j++)
                    {
                        GradientDescent(bones[j], _jointsAxisRotation[j], _currentJointRotation[j]);
                    }*/

                    closer = (_tailTarget.position - _tailEndEffector.position).magnitude < (_tailTarget.position - _currentEndEffectorPosition).magnitude;

                    if (!closer)
                    {
                        initialStep *= -1;
                    }                    

                } while (!closer);
            }

            //Debug.Log("Out");

            /*bones[0].localRotation = Rotate(Quaternion.identity, RotationZ, _currentJointRotation[0] + 1);
            bones[1].localRotation = Rotate(Quaternion.identity, RotationX, _currentJointRotation[1] + 2);
            bones[2].localRotation = Rotate(Quaternion.identity, RotationX, _currentJointRotation[2] + 3);
            bones[3].localRotation = Rotate(Quaternion.identity, RotationX, _currentJointRotation[3] + 4);
            bones[4].localRotation = Rotate(Quaternion.identity, RotationX, _currentJointRotation[4] + 5);*/

            _currentJointRotation[0] = bones[0].localEulerAngles.z;
            /*_currentJointRotation[1] = bones[1].localEulerAngles.x;
            _currentJointRotation[2] = bones[2].localEulerAngles.x;
            _currentJointRotation[3] = bones[3].localEulerAngles.x;
            _currentJointRotation[4] = bones[4].localEulerAngles.x;*/
            
        }

        private void GradientDescent(Transform bone, MyVec axis, float currentJointRotation) 
        {
            bone.localRotation = Rotate(Quaternion.identity, axis, currentJointRotation);
        }
        
        //TODO: implement fabrik method to move legs 
        private void updateLegs()
        {
            updateLegPos();
        }
        #endregion        
            
        private static MyVec RotationX
        {
            get
            {
                MyVec a;
                a.x = 1;
                a.y = 0;
                a.z = 0;
                return a;
            }
        }
        
        private static MyVec RotationY
        {
            get
            {
                MyVec a;
                a.x = 0;
                a.y = 1;
                a.z = 0;
                return a;
            }
        }
        
        private static MyVec RotationZ
        {
            get
            {
                MyVec a;
                a.x = 0;
                a.y = 0;
                a.z = 1;
                return a;
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
        
        internal Quaternion Rotate(Quaternion currentRotation, MyVec axis, float angle)
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