using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace SLP
{
    public class DiscreteDynamicsWorld : SLPhysicsWorld
    {
    }

    /// <summary>
    /// 使わない？
    /// </summary>
    public class ICollisionConfiguration
    {
    }

    /// <summary>
    /// 使わない？
    /// </summary>
    public class CollisionDispatcher
    {
        public CollisionDispatcher(ICollisionConfiguration config)
        {
        }
    }

    /// <summary>
    /// 使わない？
    /// </summary>
    public class IBroadphaseInterface
    {
    }

    /// <summary>
    /// 使わない？
    /// </summary>
    public class IConstraintSolver
    {
    }

    public class DefaultCollisionConfiguration : ICollisionConfiguration
    {
    }

    public class SequentialImpulseConstraintSolver : IConstraintSolver
    {
    }

    public class AxisSweep3 : IBroadphaseInterface
    {
        public AxisSweep3(btVector3 v1, btVector3 v2, int n, Object o, bool b)
        {
        }
    }

    public enum DebugDrawModes
    {
        DBG_DrawWireframe,
        DBG_DrawConstraints
    }

    public class IDebugDraw
    {
        public virtual DebugDrawModes DebugMode
        {
            get;
            set;
        }

        public virtual void drawLine(ref btVector3 from, ref btVector3 to, ref btVector3 color)
        {
        }

        public virtual void drawContactPoint(ref btVector3 PointOnB, ref btVector3 normalOnB, float distance, int lifeTime, ref btVector3 color)
        {
        }

        public virtual void reportErrorWarning(string warningString)
        {
        }

        public virtual void draw3dText(ref btVector3 location, string textString)
        {
        }
    }
}
