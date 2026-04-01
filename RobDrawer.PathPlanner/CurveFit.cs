using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RobDrawer.ImageProcessor;

namespace RobDrawer.PathPlanner
{
    public class CurveFit
    {
        #region Struct define
        public struct Arc
        {
            public bool is_line;
            public PointF center;           
            public double radius;           // radius. Double.PositiveInfinity if is line
            public const int p_num = 3;
            public PointF[] three_points;   // Arc's begin, mid and end point. 

            public double arc_beg_angle;    // angle for line from center to arc beg point
            public double arc_angle;        // Arc's angel, > 0 if arc is CCW. 
        }

        public struct ArcLimit
        {
            public double min_r;
            public double min_pt_dis;       // min distance for arc three points
            public double min_arc_angle; 
            public double max_arc_angle; 
        }

        public enum ApproxCurveT
        {
            MOVE,
            LINE,
            ARC
        }

        public struct ApproxCurveInfo
        {
            public ApproxCurveInfo(ApproxCurveT type, PointF[] pts)
            {
                this.type = type;
                this.pts = pts;
                this.arc = null;
            }

            public ApproxCurveInfo(Arc arc)
            {
                this.type = ApproxCurveT.ARC;
                this.pts = null;
                this.arc = arc;
            }
            public bool CheckParam(bool alert_assert_fail = false)
            {
                bool correct;
                switch (type)
                {
                    case ApproxCurveT.LINE:
                        correct = (pts != null && pts.Length == 2);
                        break;
                    case ApproxCurveT.MOVE:
                        correct = (pts != null && pts.Length == 1);
                        break;
                    case ApproxCurveT.ARC:
                        correct = (arc.HasValue);
                        break;
                    default:
                        correct = false;
                        break;
                }

#if DEBUG
                if (!correct && alert_assert_fail)
                {
                    Debug.Assert(false, "ApproxCurveInfo with wrong param");
                }
#endif
                return correct;
            }

            public ApproxCurveT type;
            public PointF[] pts;
            public Arc? arc;
            public static int MaxParamNum = 6;
        }

        #endregion

        public CurveFit(IEnumerable<SVGParser.CurveInfo> curve_path, double max_error, ArcLimit arc_limit)
        {
            this.curves_source_ = curve_path;
            this.max_error_ = max_error;
            this.arc_limit_ = arc_limit;
        }

        public IEnumerable<ApproxCurveInfo> GetResultEnum()
        {
            return new FitResult(this);
        }

        public static void ArcsCorrect(Arc[] arcs, ArcLimit limit)
        {
            if (arcs == null)
            {
                return;
            }

            for (int i = 0; i < arcs.Length; i++)
            {
                // Skip check for line
                if (arcs[i].is_line)
                {
                    continue;
                }

                // Check radius and arc angle
                if ((arcs[i].radius < limit.min_r)
                    || (Math.Abs(arcs[i].arc_angle) < limit.min_arc_angle)
                    || (Math.Abs(arcs[i].arc_angle) > limit.max_arc_angle)
                    )
                {
                    arcs[i].is_line = true;
                }

                // Check distance
                double dis1 = PointDis(arcs[i].three_points[0], arcs[i].three_points[1]);
                double dis2 = PointDis(arcs[i].three_points[0], arcs[i].three_points[2]);
                double dis3 = PointDis(arcs[i].three_points[1], arcs[i].three_points[2]);
                double min_dis = Math.Min(Math.Min(dis1, dis2), dis3);

                if (min_dis < limit.min_pt_dis)
                {
                    arcs[i].is_line = true;
                }

            }
        }

        #region Private

        private class FitResult : IEnumerable<ApproxCurveInfo>
        {
            public FitResult(CurveFit parent)
            {
                this.parent_ = parent;
            }

            IEnumerator<ApproxCurveInfo> IEnumerable<ApproxCurveInfo>.GetEnumerator()
            {
                foreach (var src_info in parent_.curves_source_)
                {
                    switch (src_info.type)
                    {
                        case SVGParser.CurveT.MOVE:
                            // just simple return
                            yield return new ApproxCurveInfo(ApproxCurveT.MOVE, src_info.points);
                            break;

                        case SVGParser.CurveT.LINE:
                            // just simple return
                            yield return new ApproxCurveInfo(ApproxCurveT.LINE, src_info.points);
                            break;

                        case SVGParser.CurveT.BEZIERS:
                            // Need fit
                            BezierFit bf = new BezierFit(src_info.points);
                            var fit_arcs = bf.Fit(parent_.max_error_);
                            foreach (Arc[] fit_arc in fit_arcs)
                            {
                                if (fit_arc == null)
                                {
                                    continue;
                                }
                                ArcsCorrect(fit_arc, this.parent_.arc_limit_);
                                foreach (var arc in fit_arc)
                                {
                                    yield return new ApproxCurveInfo(arc);
                                }
                            }
                            break;

                        default:
                            throw new SystemException("Unknown curve type when fit!");

                    }
                }
            }
            System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator()
            {
                throw new NotImplementedException();
            }

            private CurveFit parent_;

        }

        private static double PointDis(PointF p1, PointF p2)
        {
            float dx = p2.X - p1.X;
            float dy = p2.Y - p1.Y;
            return Math.Sqrt(dx * dx + dy * dy);
        }

        private IEnumerable<SVGParser.CurveInfo> curves_source_;
        private double max_error_;
        private ArcLimit arc_limit_;

        #endregion
    }
}
