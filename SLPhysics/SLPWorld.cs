/*
 * BulletXラッパー 
 * 
 * 
 */

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using StillDesign.PhysX;
using StillDesign.PhysX.MathPrimitives;

namespace SLP
{
    /// <summary>
    /// デバッグ表示用
    /// </summary>
	public class UserOutput : UserOutputStream
	{
		public override void Print(string message)
		{
			Console.WriteLine("PhysX: " + message);
		}
		public override AssertResponse ReportAssertionViolation(string message, string file, int lineNumber)
		{
			Console.WriteLine("PhysX: " + message);

			return AssertResponse.Continue;
		}
		public override void ReportError(ErrorCode errorCode, string message, string file, int lineNumber)
		{
			Console.WriteLine("PhysX: " + message);
		}
	}
    
    /// <summary>
    /// 物理ワールドのメイン
    /// </summary>
    public class SLPhysicsWorld
    {
        private static DiscreteDynamicsWorld mSingleton = new DiscreteDynamicsWorld();

        Core Core;
        Scene scene;
        public btVector3 Gravity;

        public static DiscreteDynamicsWorld getInstance()
        {
            return mSingleton;
        }
        
        /// <summary>
        /// 物理ワールドの初期化
        /// </summary>
        public void init(CollisionDispatcher d, IBroadphaseInterface b, IConstraintSolver s, ICollisionConfiguration c)
        {
			// Construct physics objects
			CoreDescription coreDesc = new CoreDescription();
            UserOutput output = new UserOutput();

			this.Core = new Core(coreDesc, output);

            Core core = this.Core;

            // デバッグ描画用の設定
            core.SetParameter(PhysicsParameter.VisualizationScale, 2.0f);
            core.SetParameter(PhysicsParameter.VisualizeForceFields, true);
            core.SetParameter(PhysicsParameter.VisualizeCollisionShapes, true);
            core.SetParameter(PhysicsParameter.VisualizeJointLimits, true);
            /*
            core.SetParameter(PhysicsParameter.VisualizeJointLocalAxes, true);
            //core.SetParameter(PhysicsParameter.VisualizeJointWorldAxes, true);
			core.SetParameter(PhysicsParameter.VisualizeClothMesh, true);
			core.SetParameter(PhysicsParameter.VisualizeFluidPosition, true);
			core.SetParameter(PhysicsParameter.VisualizeFluidEmitters, false); // Slows down rendering a bit too much
			core.SetParameter(PhysicsParameter.VisualizeSoftBodyMesh, true);
             */

            // シーンの設定
			SceneDescription sceneDesc = new SceneDescription()
			{
				//SimulationType = SimulationType.Hardware,
				Gravity = new Vector3(0, -9.81f, 0),
				GroundPlaneEnabled = true
			};

			this.scene = core.CreateScene(sceneDesc);

			HardwareVersion ver = Core.HardwareVersion;
            SimulationType simType = this.scene.SimulationType;

			// Connect to the remote debugger if it's there
			//core.Foundation.RemoteDebugger.Connect("localhost");
		}

        /// <summary>
        /// 物理シミュレーションを更新する
        /// </summary>
        public int stepSimulation(float TotalSeconds)
        {
            lock (this.scene)
            {
                // モデルの動きを物理に反映する
                for (int i = 0; i < act_list.Count; i++)
                {
                    SLAct act = act_list[i];
                    act.updateToPhisics(TotalSeconds);
                }

                // Update Physics
                this.scene.Simulate(TotalSeconds);
                //_scene.Simulate( 1.0f / 60.0f );
                this.scene.FlushStream();
                this.scene.FetchResults(SimulationStatus.RigidBodyFinished, true);

                // 物理結果をモデルに反映する
                for (int i = 0; i < act_list.Count; i++) {
                    SLAct act = act_list[i];
                    act.updateToModel(TotalSeconds);
                }

#if false
                int i = 0;
                foreach (Actor actor in this.scene.Actors)
                {
                    // 各アクターの位置・回転情報
                    Matrix m = actor.GlobalPose;

                    // これで位置がとれるので、ここからMotionStateを呼び出せばいい
                    Vector3 v = GetPosition(m);

                    //Console.WriteLine("pos " + i + ":" + v.X + "," + v.Y + "," + v.Z);
                    i++;
                }
#endif
            }

            return 0;
        }

        static Vector3 GetPosition(Matrix m)
        {
            return new Vector3(m.M41, m.M42, m.M43);
        }

        static Quaternion GetRotation(Matrix m)
        {
            return Quaternion.RotationMatrix(m);
        }

        /// <summary>
        /// カプセルを作成してワールドに追加する
        /// </summary>
        /// <param name="pos"></param>
        /// <param name="height"></param>
        /// <param name="radius"></param>
        /// <param name="density"></param>
        /// <returns></returns>
        public SLAct addCapsule(float x, float y, float z, float height, float radius, float density, bool fix)
        {
            SLAct act = SLAct.createCapsule(x, y, z, height, radius, density, fix);
            addRigidBody(act);
            return act;
        }

        public SLAct addSphere(float x, float y, float z, float radius, float density, bool fix)
        {
            SLAct act = SLAct.createSphere(x, y, z, radius, density, fix);
            addRigidBody(act);
            return act;
        }

        public void setPosition(SLAct act, float x, float y, float z)
        {
            lock (this.scene)
            {
                act.setPosition(x, y, z);
            }
        }

        public void movePosition(SLAct act, float x, float y, float z)
        {
            lock (this.scene)
            {
                act.move(x, y, z);
            }
        }

        List<SLAct> act_list = new List<SLAct>();

        public void addRigidBody(SLAct act)
        {
            lock (this.scene)
            {
                Actor actor;

                if (act.mass > 0)
                {
                    act.actorDesc.Density = 0;
                    act.actorDesc.BodyDescription.Mass = act.mass;
                }
                else
                {
                    act.actorDesc.Density = 0;
                    act.actorDesc.BodyDescription.Mass = 0.001f;
                }
#if false
                act.actorDesc.BodyDescription.LinearDamping = act.m_damping;
                act.actorDesc.BodyDescription.AngularDamping = act.m_damping;
                act.actorDesc.BodyDescription.MassSpaceInertia = act.m_inertia;
#endif
                actor = scene.CreateActor(act.actorDesc);

                //actor.MassSpaceInertiaTensor = 

                if (act.no_gravity)
                {
                    // 重力を無効にする場合
                    actor.RaiseBodyFlag(BodyFlag.DisableGravity);
                    // 回転を無効にする場合
                    actor.RaiseBodyFlag(BodyFlag.FrozenRotation);
                    // 移動を無効にする場合
                    actor.RaiseBodyFlag(BodyFlag.FrozenPosition);
                }
                if (act.kinematic_mode)
                {
                    // 重力を無効にする場合
                    actor.RaiseBodyFlag(BodyFlag.DisableGravity);
                    // 回転を無効にする場合
                    actor.RaiseBodyFlag(BodyFlag.FrozenRotation);
                    // 移動を無効にする場合
                    actor.RaiseBodyFlag(BodyFlag.FrozenPosition);

                    actor.RaiseBodyFlag(BodyFlag.Kinematic);
                }

                act.setActor(actor);
                act_list.Add(act);
            }
        }

        public SLJoint addJoint(
            SLAct actorA, SLAct actorB, float anchor_x, float anchor_y, float anchor_z, float axis_x, float axis_y, float axis_z)
        {
            SLJoint joint = new SLJoint(actorA, actorA, anchor_x, anchor_y, anchor_z, axis_x, axis_y, axis_z);
            setJoint(joint);
            return joint;
        }

        public void setJoint(SLJoint joint)
        {
            lock (this.scene)
            {
                joint.joint = (D6Joint)scene.CreateJoint(joint.d6JointDesc);
            }
        }

        int num;

        /// <summary>
        /// BulletX: 剛体をワールドに追加する
        /// </summary>
        public void addRigidBody(RigidBody body, short group, short mask)
        {
            if (body.CollisionFlags == CollisionFlags.CF_KINEMATIC_OBJECT)
            {
                body.shape.act.setKinematicMode(true);
            }
            body.updateAttr();
            addRigidBody(body.shape.act);
        }
        
        /// <summary>
        /// BulletX: 生成した制約をワールドに適用する
        /// </summary>
        /// <param name="joint"></param>
        public void addConstraint(Generic6DofSpringConstraint joint)
        {
#if false
            if (num++ > 2)
            {
                return;
                Console.WriteLine("joi " +
            joint.mr1.shape.act.m_angle + "," + joint.mr1.shape.act.m_damping + "," + joint.mr1.shape.act.m_friction + ", " + joint.mr1.shape.act.m_restitution + "," + joint.mr1.shape.act.mass);

            Console.WriteLine("joi2 " + joint.siff[0] + "," + joint.siff[1] + "," + joint.siff[2]);
            Console.WriteLine("joi3 " + joint.llow.X + "," + joint.llow.Y + "," + joint.llow.Z);
            Console.WriteLine("joi4 " + joint.lup.X + "," + joint.lup.Y + "," + joint.lup.Z);
            Console.WriteLine("joi5 " + joint.alow.X + "," + joint.alow.Y + "," + joint.alow.Z);
            Console.WriteLine("joi6 " + joint.aup.X + "," + joint.aup.Y + "," + joint.aup.Z);
            }
#endif
            SLJoint j = joint.addJoint();
            setJoint(j);
        }

        public void removeConstraint(Generic6DofSpringConstraint joint)
        {
        }

        public void removeRigidBody(RigidBody rigid)
        {
        }

        /// <summary>
        /// デバッグ描画
        /// </summary>
        public void draw()
        {
            lock (this.scene)
            {
                using (DebugRenderable data = this.scene.GetDebugRenderable())
                {
                    if (drawer != null)
                    {
                        //DrawDebug(data);
                    }
                }
            }
        }

        /// <summary>
        /// デバッグ描画
        /// </summary>
        public void debugDrawWorld()
        {
            lock (this.scene)
            {
                using (DebugRenderable data = this.scene.GetDebugRenderable())
                {
                    if (drawer != null)
                    {
                        DrawDebug(data);
                    }
                }
            }
        }

        /// <summary>
        /// デバッグ描画
        /// </summary>
        IDebugDrawer drawer;

        /// <summary>
        /// デバッグ描画
        /// </summary>
        public void setDebugDrawer(IDebugDrawer d)
        {
            drawer = d;
        }

        /// <summary>
        /// デバッグ描画
        /// </summary>
        void DrawDebug(DebugRenderable data)
        {
            if (data.PointCount > 0)
            {
                var points = data.GetDebugPoints();

                for (int i = 0; i < data.LineCount; i++)
                {
                    var point = points[i];
                    Vector3 v0 = point.Point.As<Vector3>();
                    drawer.draw(v0.X, v0.Y - 1f, v0.Z, v0.X, v0.Y + 1f, v0.Z);
                }
            }

            if (data.LineCount > 0)
            {
                var lines = data.GetDebugLines();

                for (int x = 0; x < data.LineCount; x++)
                {
                    DebugLine line = lines[x];
                    Vector3 v0 = line.Point0.As<Vector3>();
                    Vector3 v1 = line.Point1.As<Vector3>();
                    drawer.draw(v0.X, v0.Y, v0.Z, v1.X, v1.Y, v1.Z);
                }
            }

            if (data.TriangleCount > 0)
            {
                var triangles = data.GetDebugTriangles();

                for (int x = 0; x < data.TriangleCount; x++)
                {
                    DebugTriangle triangle = triangles[x];
                    Vector3 v0 = triangle.Point0.As<Vector3>();
                    Vector3 v1 = triangle.Point1.As<Vector3>();
                    Vector3 v2 = triangle.Point2.As<Vector3>();
                    drawer.draw(v0.X, v0.Y, v0.Z, v1.X, v1.Y, v1.Z);
                    drawer.draw(v2.X, v2.Y, v2.Z, v1.X, v1.Y, v1.Z);
                    drawer.draw(v0.X, v0.Y, v0.Z, v2.X, v2.Y, v2.Z);
                }
            }
        }
    }

    public class IDebugDrawer
    {
        public virtual void draw(float X, float Y, float Z, float X2, float Y2, float Z2){
        }
    }


}
