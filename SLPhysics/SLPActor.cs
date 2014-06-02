using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using StillDesign.PhysX;
using StillDesign.PhysX.MathPrimitives;

namespace SLP
{

    /// <summary>
    /// BulletX: 衝突形状の基本クラス
    /// </summary>
    public class CollisionShape
    {
        /// <summary>
        /// BulletX用のアクター
        /// </summary>
        public SLAct act;

        /// <summary>
        /// 慣性を計算する
        /// </summary>
        /// <param name="mass">重さ</param>
        /// <param name="inertia">慣性</param>
        public virtual void calculateLocalInertia(float mass, out btVector3 inertia)
        {
            //act.actorDesc.BodyDescription = new BodyDescription(mass);
            inertia = new btVector3();
        }
    }

    /// <summary>
    /// BulletX: 衝突形状 - 球
    /// </summary>
    public class SphereShape : CollisionShape
    {
        float Margin;

        public SphereShape(float width)
        {
            act = SLAct.createSphere(0, 0, 0, width, 10, false);
            Margin = width;
        }

        public SphereShape(float width, float x, float y, float z)
        {
            act = SLAct.createSphere(x, y, z, width, 10, false);
        }

        public override void calculateLocalInertia(float mass, out btVector3 inertia)
        {
            float elem = 0.4f * mass * Margin * Margin;
            inertia = new btVector3(elem, elem, elem);
        }
    }

    /// <summary>
    /// BulletX: 衝突形状 - 箱
    /// </summary>
    public class BoxShape : CollisionShape
    {
        float Margin;

        public BoxShape(btVector3 vector)
        {
            act = SLAct.createBox(vector.X, vector.Y, vector.Z, 10, false);
            Margin = vector.X;
        }

        public override void calculateLocalInertia(float mass, out btVector3 inertia)
        {
            float elem = 0.4f * mass * Margin * Margin;
            inertia = new btVector3(elem, elem, elem);
        }
    }

    /// <summary>
    /// BulletX: 衝突形状 - カプセル
    /// </summary>
    public class CapsuleShape : CollisionShape
    {
        float Margin;

        public CapsuleShape(float width, float height)
        {
            act = SLAct.createCapsule(0, 0, 0, height, width, 10, false);
            Margin = width;
        }

        public CapsuleShape(float width, float height, float x, float y, float z)
        {
            act = SLAct.createCapsule(x, y, z, height, width, 10, false);
        }

        public override void calculateLocalInertia(float mass, out btVector3 inertia)
        {
            float elem = 0.4f * mass * Margin * Margin;
            inertia = new btVector3(elem, elem, elem);
        }
    }

    public interface IMotionState
    {
        void getWorldTransform(out btTransform worldTrans);
        void setWorldTransform(btTransform worldTrans);
    }

    public class ActivationStateFlags
    {
        static public int DISABLE_DEACTIVATION = 1;
    }

    public class CollisionFlags
    {
        /// <summary>
        /// 物理を適用しない
        /// </summary>
        static public int CF_KINEMATIC_OBJECT = 2;
    }

    /// <summary>
    /// BulletX: 剛体
    /// </summary>
    public class RigidBody
    {
        public RigidBody parent;
        public RigidBody child;
        public float to_parent;
        public float to_child;

        protected float m_friction;
        protected float m_restitution;
        float m_damping, m_angle;

        public float Restitution { get { return m_restitution; } set { m_restitution = value; } }
        public float Friction { get { return m_friction; } set { m_friction = value; } }

        public CollisionShape shape;
        public IMotionState MotionState;
        btVector3 inertia;

        public int ActivationState;
        public int CollisionFlags;

        public RigidBody(float mass, IMotionState motionState, CollisionShape collisionShape, btVector3 localInertia)
        {
            shape = collisionShape;
            MotionState = motionState;
            shape.act.setMass(mass);
            shape.act.setMotionState(motionState);
            inertia = localInertia;
        }

        public void updateAttr()
        {
            shape.act.setAttr(inertia, m_damping, m_angle, m_friction, m_restitution);
        }

        public void setDamping(float linear, float angle)
        {
            m_damping = linear;
            m_angle = angle;
        }

        public void activate(bool a)
        {
        }

        /// <summary>
        /// 未使用？
        /// </summary>
        public btTransform WorldTransform;

        /// <summary>
        /// 未使用？
        /// </summary>
        public btVector3 LinearVelocity;

        /// <summary>
        /// 未使用？
        /// </summary>
        public btVector3 AngularVelocity;
    }

    public class SLAct
    {
        // 属性
        public ActorDescription actorDesc;
        public bool no_gravity;
        public bool kinematic_mode;

        public float mass;

        public float m_friction;
        public float m_restitution;
        public float m_damping, m_angle;

        // アクター
        public Actor actor;
        public IMotionState motionState;

        public static SLAct createCapsule(float x, float y, float z, float height, float radius, float density, bool fix)
        {
            Vector3 pos = new Vector3(x, y, z);
            ActorDescription actorDesc = new ActorDescription();
            BodyDescription bodyDesc = new BodyDescription();
            actorDesc.SetToDefault();
            bodyDesc.SetToDefault();

            CapsuleShapeDescription capsuleDesc = new CapsuleShapeDescription()
            {
                Radius = radius,
                Height = height,
                LocalPose = Matrix.Translation(new Vector3(0, 0, 0)),
                //Name = mName
                // LocalRotation = Matrix.CreateRotationZ(45)

            };

            actorDesc.Shapes.Add(capsuleDesc);
            if (density > 0)
            {
                actorDesc.BodyDescription = bodyDesc;
                actorDesc.Density = density;
            }

            actorDesc.GlobalPose = Matrix.Translation(pos);

            SLAct act = new SLAct(actorDesc);
            act.no_gravity = fix;
            return act;
        }

        public static SLAct createSphere(float x, float y, float z, float radius, float density, bool fix)
        {
            Vector3 pos = new Vector3(x, y, z);
            ActorDescription actorDesc = new ActorDescription();
            BodyDescription bodyDesc = new BodyDescription();
            actorDesc.SetToDefault();
            bodyDesc.SetToDefault();

            SphereShapeDescription capsuleDesc = new SphereShapeDescription()
            {
                Radius = radius,
                LocalPose = Matrix.Translation(new Vector3(0, 0, 0)),
                //Name = mName
                // LocalRotation = Matrix.CreateRotationZ(45)

            };
            actorDesc.Shapes.Add(capsuleDesc);
            if (density > 0)
            {
                actorDesc.BodyDescription = bodyDesc;
                actorDesc.Density = density;
            }
            actorDesc.GlobalPose = Matrix.Translation(pos);

            SLAct act = new SLAct(actorDesc);
            act.no_gravity = fix;
            return act;
        }

        public static SLAct createBox(float x, float y, float z, float density, bool fix)
        {
            Vector3 pos = new Vector3(x, y, z);
            ActorDescription actorDesc = new ActorDescription();
            BodyDescription bodyDesc = new BodyDescription();
            actorDesc.SetToDefault();
            bodyDesc.SetToDefault();

            BoxShapeDescription capsuleDesc = new BoxShapeDescription()
            {
                LocalPose = Matrix.Translation(new Vector3(0, 0, 0)),
                //Name = mName
                // LocalRotation = Matrix.CreateRotationZ(45)

            };

            actorDesc.Shapes.Add(capsuleDesc);
            if (density > 0)
            {
                actorDesc.BodyDescription = bodyDesc;
                actorDesc.Density = density;
            }
            actorDesc.GlobalPose = Matrix.Translation(pos);

            SLAct act = new SLAct(actorDesc);
            act.no_gravity = fix;
            return act;
        }

        /// <summary>
        /// Shapeをつくるときに初期化される
        /// </summary>
        /// <param name="desc"></param>
        public SLAct(ActorDescription desc)
        {
            actorDesc = desc;
            mass = 1;
        }

        public void setMass(float m)
        {
            mass = m;
        }

        /// <summary>
        /// RigidBodyを生成するときに初期化される
        /// </summary>
        /// <param name="ms"></param>
        public void setMotionState(IMotionState ms)
        {
            motionState = ms;

            // MMDXのパーツ座標を取得する
            btTransform world;
            motionState.getWorldTransform(out world);

            // 初期値をセットする
            actorDesc.GlobalPose = world.getMatrix();
        }

        public Vector3 m_inertia;

        public void setAttr(btVector3 inertia, float linear_damping, float angle_damping, float friction, float restriction)
        {
            m_inertia = new Vector3(inertia.X, inertia.Y, inertia.Z);
            m_angle = angle_damping;
            m_damping = linear_damping;
            m_friction = friction;
            m_restitution = restriction;

            //Console.WriteLine("attr " + m_angle + ", " + m_damping + ", " + m_friction + ", " + m_restitution);
        }

        /// <summary>
        /// シーンにaddするときに初期化される
        /// </summary>
        /// <param name="a"></param>
        public void setActor(Actor a)
        {
            actor = a;
        }

        public void setKinematicMode(bool b)
        {
            kinematic_mode = b;
        }

        /// <summary>
        /// ワールドのupdateから呼ばれる
        /// </summary>
        public void updateToPhisics(float timeStep)
        {
            lock (this.actor)
            {
                if (motionState != null)
                {
                    btTransform world;

                    // MMDXのパーツ座標を取得する
                    motionState.getWorldTransform(out world);

                    // 物理にモデルの位置を適用する
                    actor.GlobalPose = world.getMatrix();

                    // Kinamticならそのまま使う
                    if (kinematic_mode)
                    {
                        Matrix pre = world.getMatrix();

                    }

                    // 物理を適用する
                    world.setMatrix(actor.GlobalPose);

                    // 適用結果をパーツ座標に反映する
                    motionState.setWorldTransform(world);
                }
            }
        }

        public void updateToModel(float timeStep)
        {
            lock (this.actor)
            {
                if (motionState != null)
                {
                    btTransform world;

                    // MMDXのパーツ座標を取得する
                    motionState.getWorldTransform(out world);

                    // 物理を適用する
                    world.setMatrix(actor.GlobalPose);

                    // 適用結果をパーツ座標に反映する
                    motionState.setWorldTransform(world);
                }
            }
        }

        /// <summary>
        /// 位置を強制的に変更する
        /// PhysX上の位置は変わらないようなので、KinematicModeのもののみに使う。
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="z"></param>
        public void setPosition(float x, float y, float z)
        {
            lock (this.actor)
            {
                Vector3 v = new Vector3(x, y, z);
                actor.GlobalPosition = v;
            }
        }

        /// <summary>
        /// 位置を移動する
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="z"></param>
        public void move(float x, float y, float z)
        {
            lock (this.actor)
            {
                Vector3 v = new Vector3(x, y, z);
                actor.GlobalPosition += v;
            }

            //actor.AddForce(v, ForceMode.Force);
            //actor.LinearVelocity = v;
        }
    }
}
