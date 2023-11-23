﻿using System;
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
        float _animationRange;

        //LEGS
        Transform[] _legTargets;
        Transform[] _legFutureBases;
        MyTentacleController[] _legs = new MyTentacleController[6];

        private bool enter;

        
        #region public
        public void InitLegs(Transform[] LegRoots,Transform[] LegFutureBases, Transform[] LegTargets)
        {
            _legs = new MyTentacleController[LegRoots.Length];
            //Legs init
            for(int i = 0; i < LegRoots.Length; i++)
            {
                _legs[i] = new MyTentacleController();
                _legs[i].LoadTentacleJoints(LegRoots[i].GetChild(0), TentacleMode.LEG);
                _legTargets[i] = LegTargets[i];
                _legFutureBases[i] = LegFutureBases[i];
            }
        }

        public void InitTail(Transform TailBase)
        {
            _tail = new MyTentacleController();
            _tail.LoadTentacleJoints(TailBase, TentacleMode.TAIL);
            _tailEndEffector = _tail.Bones[_tail.Bones.Length - 1];
        }

        //TODO: Check when to start the animation towards target and implement Gradient Descent method to move the joints.
        public void NotifyTailTarget(Transform target)
        {
            if (Vector3.Distance(target.position, _tailEndEffector.position) < 1)
            {
                _tailTarget = target;    
            }
        }

        //TODO: Notifies the start of the walking animation
        public void NotifyStartWalk()
        {

        }

        //TODO: create the apropiate animations and update the IK from the legs and tail
        public void UpdateIK()
        {
            updateTail();
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
            if (!enter)
            {
                /*_tail.Bones[0].rotation = Rotate(Quaternion.identity, RotationZ, 60);
                _tail.Bones[1].rotation = Rotate(_tail.Bones[0].rotation, RotationX, -60);
                _tail.Bones[2].rotation = Rotate(_tail.Bones[1].rotation, RotationX, -30);
                _tail.Bones[3].rotation = Rotate(_tail.Bones[2].rotation, RotationX, -15);
                _tail.Bones[4].rotation = Rotate(_tail.Bones[3].rotation, RotationX, -10);*/
                enter = true;
            }
        }
        
        //TODO: implement fabrik method to move legs 
        private void updateLegs()
        {

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
    }
}