using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Leap;

namespace SL
{
    public class SLLeapHand
    {
        /// <summary>
        /// hand, finger, born
        /// </summary>
        public SLVector3[,,] born;

        public static SLLeapHand create()
        {
            SLLeapHand me = new SLLeapHand();
            me.born = new SLVector3[2, 5, 5];
            return me;
        }
    }

    public class SLLeap
    {
        private static float scale = 15, yoffset = 5;
        private static int maxhand = 2;

        private Controller mController;

        public void load()
        {
            mController = new Controller();
        }

        public void unload()
        {
            mController.Dispose();
            mController = null;
        }

        public void update(ref SLLeapHand h, ref SLLeapHand dh)
        {
            if (!mController.IsConnected)
            {
                return;
            }

            // Get the most recent frame and report some basic information
            Frame frame = mController.Frame();
            HandList hands = frame.Hands;
            PointableList pointables = frame.Pointables;
            FingerList fingers = frame.Fingers;
            ToolList tools = frame.Tools;

            for (int ih = 0; ih < 2; ih++)
            {
                for (int i = 0; i < 5; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        dh.born[ih, i, j] = h.born[ih, i, j];
                    }
                }
            }

            int hi = 0;
            foreach (Hand hand in hands)
            {
                int fi = 0;
                foreach (Finger f in hand.Fingers)
                {
                    Bone b = f.Bone(Bone.BoneType.TYPE_DISTAL);
                    h.born[hi, fi, 0].X = b.NextJoint.x / scale;
                    h.born[hi, fi, 0].Y = b.NextJoint.y / scale - yoffset;
                    h.born[hi, fi, 0].Z = b.NextJoint.z / scale;
                    h.born[hi, fi, 1].X = b.PrevJoint.x / scale;
                    h.born[hi, fi, 1].Y = b.PrevJoint.y / scale - yoffset;
                    h.born[hi, fi, 1].Z = b.PrevJoint.z / scale;
                    b = f.Bone(Bone.BoneType.TYPE_INTERMEDIATE);
                    h.born[hi, fi, 2].X = b.PrevJoint.x / scale;
                    h.born[hi, fi, 2].Y = b.PrevJoint.y / scale - yoffset;
                    h.born[hi, fi, 2].Z = b.PrevJoint.z / scale;
                    b = f.Bone(Bone.BoneType.TYPE_PROXIMAL);
                    h.born[hi, fi, 3].X = b.PrevJoint.x / scale;
                    h.born[hi, fi, 3].Y = b.PrevJoint.y / scale - yoffset;
                    h.born[hi, fi, 3].Z = b.PrevJoint.z / scale;
                    b = f.Bone(Bone.BoneType.TYPE_METACARPAL);
                    h.born[hi, fi, 4].X = b.PrevJoint.x / scale;
                    h.born[hi, fi, 4].Y = b.PrevJoint.y / scale - yoffset;
                    h.born[hi, fi, 4].Z = b.PrevJoint.z / scale;
                    fi++;
                }
                hi++;

                if (hi >= maxhand)
                {
                    break;
                }
            }

            for (int ih = 0; ih < 2; ih++)
            {
                for (int i = 0; i < 5; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        dh.born[ih, i, j] = h.born[ih, i, j] - dh.born[ih, i, j];
                    }
                }
            }
        }
    }
}
