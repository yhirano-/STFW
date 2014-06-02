using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using StillDesign.PhysX;
using StillDesign.PhysX.MathPrimitives;

namespace SLP
{
    public class SLJoint
    {
        public D6Joint joint;
        public D6JointDescription d6JointDesc;
        public SLAct act1, act2;
        public Vector3 act_anc;

        public void printActor()
        {
            Console.WriteLine("act1 " + act1.actor.GlobalPosition.X + ", " + act1.actor.GlobalPosition.Y + ", " + act1.actor.GlobalPosition.Z);
            Console.WriteLine("act2 " + act2.actor.GlobalPosition.X + ", " + act2.actor.GlobalPosition.Y + ", " + act2.actor.GlobalPosition.Z);
            Console.WriteLine("xyz  " + act_anc.X + ", " + act_anc.Y + ", " + act_anc.Z);
        }

        public SLJoint(
            SLAct actorA, SLAct actorB, float anchor_x, float anchor_y, float anchor_z, float axis_x, float axis_y, float axis_z)
        {
            Vector3 anchor = new Vector3(anchor_x, anchor_y, anchor_z);
            Vector3 axis = new Vector3(axis_x, axis_y, axis_z);
            act_anc = anchor;
            d6JointDesc = new StillDesign.PhysX.D6JointDescription();
            d6JointDesc.SetToDefault();

            d6JointDesc.Actor1 = actorA != null ? actorA.actor : null;
            d6JointDesc.Actor2 = actorB != null ? actorB.actor : null;

            d6JointDesc.DriveLinearVelocity = new Vector3(0, 0, 0);

            // グローバル座標でのアンカーの場所
            d6JointDesc.SetGlobalAnchor(anchor);

            // グローバル座標での回転軸
            d6JointDesc.SetGlobalAxis(axis);

            // 回転方向の制約
            d6JointDesc.TwistMotion = StillDesign.PhysX.D6JointMotion.Locked;
            d6JointDesc.Swing1Motion = StillDesign.PhysX.D6JointMotion.Locked;
            d6JointDesc.Swing2Motion = StillDesign.PhysX.D6JointMotion.Locked;

            // 移動方向の制約
            d6JointDesc.XMotion = StillDesign.PhysX.D6JointMotion.Locked;
            d6JointDesc.YMotion = StillDesign.PhysX.D6JointMotion.Locked;
            d6JointDesc.ZMotion = StillDesign.PhysX.D6JointMotion.Locked;

            d6JointDesc.ProjectionMode = StillDesign.PhysX.JointProjectionMode.None;

            act1 = actorA;
            act2 = actorB;
        }

        /// <summary>
        /// ジョイントに移動・回転制限のパラメータを設定する
        /// </summary>
        /// <param name='joint'>PMX用ジョイントデータ</param>
        /// <param name='conf'>Unity用ジョイント</param>
        public void limitAngular(btVector3 llow, btVector3 lup, btVector3 alow, btVector3 aup, float[] spring)
        {
            float ang_damp = 0;

            // 角度の固定
            if (alow.X == 0.0f && aup.X == 0.0f)
            {
                d6JointDesc.TwistMotion = StillDesign.PhysX.D6JointMotion.Locked;
            }
            else
            {
                d6JointDesc.TwistMotion = StillDesign.PhysX.D6JointMotion.Limited;
                float hlim = Math.Max(-alow.X, -aup.X); //回転方向が逆なので負数
                float llim = Math.Min(-alow.X, -aup.X);
                JointLimitSoftPairDescription xlimit = new StillDesign.PhysX.JointLimitSoftPairDescription();
                JointLimitSoftDescription xhigh = new StillDesign.PhysX.JointLimitSoftDescription();
                //xhigh.Value = Math.Clamp(hlim * Math.Rad2Deg, -180.0f, 180.0f);
                xhigh.Value = hlim;
                xhigh.Damping = ang_damp;
                xhigh.Spring = ang_damp;
                xhigh.Restitution = ang_damp;
                xlimit.High = xhigh;
                JointLimitSoftDescription xlow = new StillDesign.PhysX.JointLimitSoftDescription();
                //jllim.limit = Mathf.Clamp(llim * Mathf.Rad2Deg, -180.0f, 180.0f);
                xlow.Value = llim;
                xlow.Damping = ang_damp;
                xlow.Spring = ang_damp;
                xlow.Restitution = ang_damp;
                xlimit.Low = xlow;
            }

            if (alow.Y == 0.0f && aup.Y == 0.0f)
            {
                d6JointDesc.Swing1Motion = StillDesign.PhysX.D6JointMotion.Locked;
            }
            else
            {
                // 値がマイナスだとエラーが出るので注意
                d6JointDesc.Swing1Motion = StillDesign.PhysX.D6JointMotion.Limited;
                float lim = Math.Min(Math.Abs(alow.Y), Math.Abs(aup.Y));//絶対値の小さい方
                JointLimitSoftDescription ylimit = new StillDesign.PhysX.JointLimitSoftDescription();
                //ylimit.Value = lim * Mathf.Clamp(Mathf.Rad2Deg, 0.0f, 180.0f);
                ylimit.Value = lim;
                ylimit.Damping = ang_damp;
                ylimit.Spring = ang_damp;
                ylimit.Restitution = ang_damp;
            }

            if (alow.Z == 0f && aup.Z == 0f)
            {
                d6JointDesc.Swing2Motion = StillDesign.PhysX.D6JointMotion.Locked;
            }
            else
            {
                d6JointDesc.Swing2Motion = StillDesign.PhysX.D6JointMotion.Limited;
                float lim = Math.Min(Math.Abs(-alow.Z), Math.Abs(-aup.Z));//絶対値の小さい方//回転方向が逆なので負数
                JointLimitSoftDescription zlimit = new StillDesign.PhysX.JointLimitSoftDescription();
                //jlim.limit = Mathf.Clamp(lim * Mathf.Rad2Deg, 0.0f, 180.0f);
                zlimit.Value = lim;
                zlimit.Damping = ang_damp;
                zlimit.Spring = ang_damp;
                zlimit.Restitution = ang_damp;
            }

            float damping = 0;

            // Motionの固定
            JointLimitSoftDescription linearLimit = new StillDesign.PhysX.JointLimitSoftDescription();
            linearLimit.Damping = damping;
            linearLimit.Restitution = damping;
            linearLimit.Spring = damping;
            float val = 0;
            if (llow.X == 0.0f && lup.X == 0.0f)
            {
                linearLimit.Value = 0;
            }
            else
            {
                val = Math.Min(Math.Abs(llow.X), Math.Abs(lup.X));//絶対値の小さい方
            }

            if (llow.Y == 0.0f && lup.Y == 0.0f)
            {
                linearLimit.Value = 0;
            }
            else
            {
                val = Math.Min(Math.Abs(llow.Y), Math.Abs(lup.Y));//絶対値の小さい方
            }

            if (llow.Z == 0.0f && lup.Z == 0.0f)
            {
                linearLimit.Value = 0;
            }
            else
            {
                val = Math.Min(Math.Abs(llow.Z), Math.Abs(lup.Z));//絶対値の小さい方
            }

            if (val != 0)
            {
                linearLimit.Value = val;
            }
            d6JointDesc.LinearLimit = linearLimit;

            /// ジョイントにばねなどのパラメータを設定する
            // Position
            float scale_ = 1;
            damping = 0;

            if (spring[0] != 0.0f)
            {
                JointDriveDescription jointDriveX = new StillDesign.PhysX.JointDriveDescription();
                jointDriveX.DriveType = StillDesign.PhysX.D6JointDriveType.DrivePosition;
                jointDriveX.ForceLimit = 0;
                jointDriveX.Damping = damping;
                jointDriveX.Spring = spring[0] * scale_;
                d6JointDesc.XDrive = jointDriveX;
            }
            if (spring[1] != 0.0f)
            {
                JointDriveDescription jointDriveX = new StillDesign.PhysX.JointDriveDescription();
                jointDriveX.DriveType = StillDesign.PhysX.D6JointDriveType.DrivePosition;
                jointDriveX.ForceLimit = 0;
                jointDriveX.Damping = damping;
                jointDriveX.Spring = spring[1] * scale_;
                d6JointDesc.YDrive = jointDriveX;
            }
            if (spring[2] != 0.0f)
            {
                JointDriveDescription jointDriveX = new StillDesign.PhysX.JointDriveDescription();
                jointDriveX.DriveType = StillDesign.PhysX.D6JointDriveType.DrivePosition;
                jointDriveX.ForceLimit = 0;
                jointDriveX.Damping = damping;
                jointDriveX.Spring = spring[2] * scale_;
                d6JointDesc.ZDrive = jointDriveX;
            }

            // Angular
            if (spring[3] != 0.0f)
            {
                JointDriveDescription jointDriveX = new StillDesign.PhysX.JointDriveDescription();
                jointDriveX.DriveType = StillDesign.PhysX.D6JointDriveType.DrivePosition;
                jointDriveX.ForceLimit = 0;
                jointDriveX.Damping = damping;
                jointDriveX.Spring = spring[3] * scale_;
                d6JointDesc.SwingDrive = jointDriveX;
            }
            if (spring[4] != 0.0f)
            {
                JointDriveDescription jointDriveX = new StillDesign.PhysX.JointDriveDescription();
                jointDriveX.DriveType = StillDesign.PhysX.D6JointDriveType.DrivePosition;
                jointDriveX.ForceLimit = 0;
                jointDriveX.Damping = damping;
                jointDriveX.Spring = spring[4] * scale_;
                d6JointDesc.TwistDrive = jointDriveX;
            }
            if (spring[5] != 0.0f)
            {
                JointDriveDescription jointDriveX = new StillDesign.PhysX.JointDriveDescription();
                jointDriveX.DriveType = StillDesign.PhysX.D6JointDriveType.DrivePosition;
                jointDriveX.ForceLimit = 0;
                jointDriveX.Damping = damping;
                jointDriveX.Spring = spring[5] * scale_;
                d6JointDesc.SLerpDrive = jointDriveX;
            }
        }

        public void setPosition(float x, float y, float z)
        {
            joint.GlobalAnchor = new Vector3(x, y, z);
        }

        public void move(float x, float y, float z)
        {
            joint.GlobalAnchor = new Vector3(joint.GlobalAnchor.X + x, joint.GlobalAnchor.Y + y, joint.GlobalAnchor.Z + z);
        }
    }

    public class Generic6DofSpringConstraint
    {
        public RigidBody mr1, mr2;
        public btTransform mt1, mt2;

        public Generic6DofSpringConstraint(RigidBody r1, RigidBody r2, btTransform t1, btTransform t2, bool b)
        {
            mr1 = r1;
            mr2 = r2;
            mt1 = t1;
            mt2 = t2;
        }

        public SLJoint addJoint()
        {
            float ax, ay, az, xx, xy, xz;
            ax = mt1.Origin.X;
            ay = mt1.Origin.Y;
            az = mt1.Origin.Z;
            xx = 1;
            xy = xz = 0;

            ax = mr1.shape.act.actor.GlobalPose.M41;
            ay = mr1.shape.act.actor.GlobalPose.M42;
            az = mr1.shape.act.actor.GlobalPose.M43;

            SLJoint j = new SLJoint(mr1.shape.act, mr2.shape.act, ax, ay, az, xx, xy, xz);
            j.limitAngular(llow, lup, alow, aup, siff);
            return j;
        }

        public btVector3 llow;
        public btVector3 lup;
        public btVector3 alow;
        public btVector3 aup;
        public float[] siff = new float[6];
        public bool[] spr = new bool[6];

        public void setLinearLowerLimit(btVector3 v)
        {
            llow = v;
        }

        public void setLinearUpperLimit(btVector3 v)
        {
            lup = v;
        }

        public void setAngularLowerLimit(btVector3 v)
        {
            alow = v;
        }

        public void setAngularUpperLimit(btVector3 v)
        {
            aup = v;
        }

        public void setStiffness(int i, float pos)
        {
            siff[i] = pos;
        }

        public void enableSpring(int i, bool b)
        {
            spr[i] = b;
        }

        public void calculateTransforms()
        {
        }

        public void setEquilibriumPoint()
        {
        }
    }
}
