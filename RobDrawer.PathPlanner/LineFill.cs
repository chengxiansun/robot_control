using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;

namespace RobDrawer.PathPlanner
{
    class LineFill
    {

        public static List<PathUtil.LineF> FilledWithLine(IEnumerable<CurveFit.ApproxCurveInfo> approx_path)
        {
            return null;
        }

        private struct Bound
        {
            public float top;
            public float bottom;
            public bool with_top;
            public bool with_bottom;
        }

        private abstract class Curve
        {
            public abstract List<float> Intersection(float y);

            private bool IsInBound(float y)
            {
                return
                    ((y < bound_.top) || (bound_.with_top && y == bound_.top))     // if y is under top
                    &&
                    ((y > bound_.bottom) || (bound_.with_bottom && y == bound_.bottom)) // if y is upper bottom
                    ;
            }

            protected Bound bound_;
        }

        private class CurveLine : Curve
        {

            public CurveLine(PointF beg_pt, PointF end_pt)
            {
                // Init A_, B_, C_
                float delta_x = end_pt.X - beg_pt.X;
                float delta_y = end_pt.Y - beg_pt.Y;

                // A_ * delta_x + B_ * delta_y = 0
                if (MathNetIri.Number.AlmostZero(delta_y))
                {
                    B_ = 1;
                    A_ = -delta_y / delta_x;
                }
                else
                {
                    A_ = 1;
                    B_ = -delta_x / delta_y;
                }
                C_ = -(A_ * beg_pt.X + B_ * beg_pt.Y);

#if DEBUG
                float C_2 = -(A_ * end_pt.X + B_ * end_pt.Y);
                Debug.Assert(MathNetIri.Number.AlmostEqual(C_, C_2));
#endif

                // Init bound
                if (beg_pt.Y > end_pt.Y)
                {
                    bound_.top = beg_pt.Y;
                    bound_.with_top = true;
                    bound_.bottom = end_pt.Y;
                    bound_.with_bottom = false;
                }
                else
                {
                    bound_.top = end_pt.Y;
                    bound_.with_top = false;
                    bound_.bottom = beg_pt.Y;
                    bound_.with_bottom = true;
                }

            }

            public override List<float> Intersection(float y)
            {
                List<float> ret = new List<float>();
                if (MathNetIri.Number.AlmostZero(A_))
                {
                    return ret;
                }

                float x_ret = (C_ + B_ * y) / -A_;
                ret.Add(x_ret);

                return ret;
            }

            // Line: A_x + B_y + C_ = 0;
            private float A_;
            private float B_;
            private float C_;
        }

        private class CurveArc: Curve
        {
            public CurveArc(PointF center, float r, float beg_angle, float sweep_angle)
            {
                if (beg_angle < 0 || beg_angle >= 360)
                {
                    throw new SystemException("Only handler angle in [0, 360)");
                }
                if (Math.Abs(sweep_angle) > 360)
                {
                    throw new SystemException("Too large sweep angle");
                }

                float end_angle = beg_angle + sweep_angle;


                this.center_ = center;
                this.r_ = r;

            }

            public override List<float> Intersection(float y)
            {
                throw new NotImplementedException();
            }


            private enum Side
            {
                LEFT,
                RIGHT
            }

            private PointF center_;
            private float r_;
            // Left or right;
            private Side side_;
        }

    }
}
