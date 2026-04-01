using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNetIri;
using System.Diagnostics;
using VectorD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using Arc = RobDrawer.PathPlanner.CurveFit.Arc;

namespace RobDrawer.PathPlanner
{

    class BezierFit
    {

        public BezierFit(PointD[] ctrl_points)
        {
            // copy control points
            int max_i = Math.Min(ctrl_points.Length, kCtrlPSize);
            for (int i = 0; i < max_i; i++)
            {
                this.ctrl_pts_[i] = ctrl_points[i];
            }

            Init();
        }

        public BezierFit(PointF[] ctrl_points)
        {
            // copy control points
            int max_i = Math.Min(ctrl_points.Length, kCtrlPSize);
            for (int i = 0; i < max_i; i++)
            {
                this.ctrl_pts_[i].X = ctrl_points[i].X;
                this.ctrl_pts_[i].Y = ctrl_points[i].Y;
            }

            Init();
        }

        public PointF[] GetPoint()
        {
            const int p_size = 100;
            PointF[] ret = new PointF[p_size];
            for (int i = 0; i < p_size; i++)
            {
                ret[i].X = (float)bezier_xt_.Evaluate(1.0 / p_size * i);
                ret[i].Y = (float)bezier_yt_.Evaluate(1.0 / p_size * i);
            }

            return ret;
        }

        public PointF GetTopPoint()
        {
            PointF ret = new PointF();
            double top_p_t = GetTopPointT();
            ret.X = (float)bezier_xt_.Evaluate(top_p_t);
            ret.Y = (float)bezier_yt_.Evaluate(top_p_t);
            return ret;
        }

        public BezierFit[] Split(double t)
        {
            if (t < 0 || t > 1)
            {
                return new BezierFit[0];
            }

            Debug.Assert(kOrder == 3 , 
                "Split function only support bezier curve with order 3");
            var v_01 = p_[1] - p_[0];
            var v_12 = p_[2] - p_[1];
            var v_23 = p_[3] - p_[2];

            var p_01 = v_01 * t + p_[0];
            var p_12 = v_12 * t + p_[1];
            var p_23 = v_23 * t + p_[2];

            var v_012 = p_12 - p_01;
            var v_123 = p_23 - p_12;

            var p_012 = v_012 * t + p_01;
            var p_123 = v_123 * t + p_12;

            var v_0123 = p_123 - p_012;

            var p_0123 = v_0123 * t + p_012;

            PointD[] bezier1_p = new PointD[kCtrlPSize];
            bezier1_p[0] = VecToPointD(p_[0]);
            bezier1_p[1] = VecToPointD(p_01);
            bezier1_p[2] = VecToPointD(p_012);
            bezier1_p[3] = VecToPointD(p_0123);

            PointD[] bezier2_p = new PointD[kCtrlPSize];
            bezier2_p[0] = VecToPointD(p_0123);
            bezier2_p[1] = VecToPointD(p_123);
            bezier2_p[2] = VecToPointD(p_23);
            bezier2_p[3] = VecToPointD(p_[3]);

            var ret = new BezierFit[2];
            ret[0] = new BezierFit(bezier1_p);
            ret[1] = new BezierFit(bezier2_p);
            return ret;
        }

        public BezierFit[] SplitOptimizing()
        {
            double split_t = Double.NaN;
            const double t_max = 0.618;
            const double t_min = 1 - t_max;
            // try inflection point
            var inf_ts = GetInflectionT();
            inf_ts = inf_ts.FindAll(
                v => (v <= t_max && v >= t_min)
             );
            if (inf_ts.Any())
            {
                // Get middle point
                inf_ts.Sort();
                int mid_i = inf_ts.Count / 2;
                split_t = inf_ts[mid_i];
            }
            else
            {
                split_t = GetTopPointT();
            }

            if (split_t > t_max)
            {
                split_t = t_max;
            }
            else if (split_t < t_min)
            {
                split_t = t_min;
            }

            return Split(split_t);

        }

        public BinaryTree<Arc[]> Fit(double max_error)
        {
            // Try fit with line
            FitInfo line_fi = LineFit();
            if (Number.AlmostZero(line_fi.max_error, kDError))
            {
                Arc[] arc = this.FitInfoToArc(line_fi);
                return new BinaryTree<Arc[]>(arc);
            }

            // Try fit with single arc
            FitInfo single_fit = SingleCirFit();
            if (single_fit.max_error <= max_error)
            {
                Arc[] arc = this.FitInfoToArc(single_fit);
                return new BinaryTree<Arc[]>(arc);
            }

            // Check b1, b2, c
            if (b1_ > 0 && b2_ > 0 && c_ > 0) 
            {
                // Check if can fit within max_error
                FitInfo info = DoubleCirFitWithMinError(0.001);
                if (info.max_error <= max_error)
                {
                    Arc[] arc = this.FitInfoToArc(info);
                    return new BinaryTree<Arc[]>(arc);
                }
            }

            // Could not Fit, Split
            var sub_beziers = this.SplitOptimizing();
            var left_arcs = sub_beziers[0].Fit(max_error);
            var right_arcs = sub_beziers[1].Fit(max_error);

            return new BinaryTree<Arc[]>(null, left_arcs, right_arcs);
            
        }

        public override string ToString()
        {
            StringBuilder sb = new StringBuilder();
            sb.Append("BezierFit: ");
            foreach (var p in p_)
            {
                sb.Append(String.Format(" ({0},{1}) ", p[0], p[1]));
            }
            return sb.ToString();
        }

        #region Private
        private struct FitCirInfo
        {
            public VectorD c;
            public double r;
            public double t_beg;
            public double t_end;
        }

        private struct FitInfo
        {
            public VectorD mid_point;
            public FitCirInfo[] fit_cirs;
            public double max_error;
        }

        public struct PointD
        {
            public double X;
            public double Y;
        }

        static BezierFit()
        {
            // Init Bezier pol
            bezier_pol = new Polynomial[kCtrlPSize];

            Polynomial pol1 = new Polynomial(new double[] { 0.0, 1.0 });
            Polynomial pol2 = new Polynomial(new double[] { 1.0, -1.0 });

            for (int i = 0; i < kCtrlPSize; i++)
            {
                double coe = Combinatorics.Combinations(kOrder, i);
                var sub_pol1 = PolPow(pol1, i);
                var sub_pol2 = PolPow(pol2, kOrder - i);
                bezier_pol[i] = sub_pol1.Multiply(sub_pol2);
                bezier_pol[i].MultiplyInplace(coe);
            }
        }

        // min_r: If arc's radius less that this value, will use line to fit instead of arc.
        private Arc[] FitInfoToArc(FitInfo info)
        {
            // Check if fit with line
            if (info.fit_cirs == null || info.fit_cirs.Length == 0)
            {
                // Calc line info
                Arc[] ret = new Arc[1];
                ret[0] = MakeLineFitArc(p_[0], p_[3]);

                return ret;
            }
            else if (info.fit_cirs.Length == 1)
            {
                Arc[] ret = new Arc[1];
                ret[0] = CirInfoToArc(info.fit_cirs[0], p_[0], p_[3]);
                return ret;
            }
            else
            {
                Debug.Assert(info.fit_cirs.Length == 2, "Wrong fit_cirs length in FitInfo");
                Arc[] ret = new Arc[2];
                // Arc 1
                ret[0] = CirInfoToArc(info.fit_cirs[0], p_[0], info.mid_point);
                // Arc 2
                ret[1] = CirInfoToArc(info.fit_cirs[1], info.mid_point, p_[3]);

                return ret;
            }
        }

        private void Init()
        {
            // Init vector
            var vec_build = Vector.Build;
            for (int i = 0; i < kCtrlPSize; i++)
            {
                p_[i] = vec_build.Dense(2, 0.0);
                p_[i][0] = this.ctrl_pts_[i].X;
                p_[i][1] = this.ctrl_pts_[i].Y;
            }

            // Check if p_0 == p_1 Or p_2 == p_3
            const double off_factor = 0.0001;
            if (Number.AlmostZero(
                    (p_[1] - p_[0]).L2Norm() ))
            {
                Trace.TraceInformation("Bezier control point 0 and 1 are coincident");
                var p_02 = p_[2] - p_[0];
                p_[1] += p_02 * off_factor;
            }
            if (Number.AlmostZero(
                    (p_[2] - p_[3]).L2Norm() ))
            {
                Trace.TraceInformation("Bezier control point 2 and 3 are coincident");
                var p_13 = p_[3] - p_[1];
                p_[2] += p_13 * off_factor;
            }

            // Init polynomial
            for (int i = 0; i < kCtrlPSize; i++)
            {
                bezier_xt_ += bezier_pol[i] * ctrl_pts_[i].X;
                bezier_yt_ += bezier_pol[i] * ctrl_pts_[i].Y;
            }

            a_ = ((p_[1] - p_[0]).L2Norm() * (p_[2] - p_[3]).L2Norm()) +
                (p_[1] - p_[0]).DotProduct(p_[2] - p_[3]);
            c_ = Math.Pow((p_[3] - p_[0]).L2Norm(), 2.0) / 2.0;
            b1_ = (p_[3] - p_[0]).DotProduct(p_[1] - p_[0]);
            b2_ = (p_[0] - p_[3]).DotProduct(p_[2] - p_[3]);
        }

        #region Fit method
        private FitInfo LineFit()
        {
            FitInfo ret = new FitInfo();
            ret.fit_cirs = null;
            ret.mid_point = null;

            // Check max error
            // Check if p_01 and P_23 is parallel
            var p_01 = p_[1] - p_[0];
            var p_23 = p_[3] - p_[2];
            if (Number.AlmostZero( (p_01[0] * p_23[1] - p_01[1] * p_23[0]), kDError))
            {
                // Is parallel
                // Use the distance of point p[1] and p[2] to line p[0]p[3]
                double dis_1 = DisToLine(p_[0], p_[3], p_[1]);
                double dis_2 = DisToLine(p_[0], p_[3], p_[2]);
                //Console.WriteLine("Parallel, error: " + Math.Max(dis_1, dis_2));
                ret.max_error = Math.Max(dis_1, dis_2);
                return ret;
            }

            // Use distance of top point to line p[0]p[3]
            double top_t = GetTopPointT();
            var top_p = GetBezierPoint(top_t);
            ret.max_error = DisToLine(p_[0], p_[3],top_p);
            return ret;
        }

        private FitInfo SingleCirFit()
        {
            FitInfo ret = new FitInfo();

            ret.fit_cirs = new FitCirInfo[1];
            ret.mid_point = null;

            VectorD c = CalcCircleOrCenter(p_[3] - p_[2], p_[3], p_[0]);
            ret.fit_cirs[0].c = c;
            ret.fit_cirs[0].r = (c - p_[0]).L2Norm();
            ret.fit_cirs[0].t_beg = 0;
            ret.fit_cirs[0].t_end = 1;
            double err1 = MaxDisToCircle(ret.fit_cirs[0]);
            ret.max_error = err1;

#if DEBUG
            {
                double r_other = (c - p_[3]).L2Norm();
                Debug.Assert(
                    Number.AlmostZero( (ret.fit_cirs[0].r - r_other), kDError),
                    "Error in SingleCirFit()"
                    );
            }
#endif

            return ret;
        }

        // Use "0.618" method to find the min value
        private FitInfo DoubleCirFitWithMinError(double accuracy, double allow_error = Double.NaN)
        {
            // Init
            double max_lam = c_ / b1_;
            double min_lam = 0;
            double lam_range = max_lam - min_lam;

            do
            {
                const double mid2_factor = 0.618;
                const double mid1_factor = 1 - mid2_factor;
                double mid1_lam = lam_range * mid1_factor + min_lam;
                FitInfo mid1_lam_info = DoubleCirFit(mid1_lam);
                double mid2_lam = lam_range * mid2_factor + min_lam;
                FitInfo mid2_lam_info = DoubleCirFit(mid2_lam);

                double better_lam;
                double better_error;
                if (mid1_lam_info.max_error > mid2_lam_info.max_error)
                {
                    // mid2 is better than mid1
                    // remove [min, mid1)
                    min_lam = mid1_lam;
                    //min_lam_info = mid1_lam_info;
                    better_lam = mid2_lam;
                    better_error = mid2_lam_info.max_error;
                }
                else
                {
                    // mid1 is better than mid2
                    // remove (mid2, max]
                    max_lam = mid2_lam;
                    //max_lam_info = mid2_lam_info;
                    better_lam = mid1_lam;
                    better_error = mid1_lam_info.max_error;
                }

                if (allow_error != Double.NaN && better_error <= allow_error)
                {
                    min_lam = better_lam;
                    max_lam = better_lam;
                    // The loop then will break
                }

                lam_range = max_lam - min_lam;

            } while (lam_range > accuracy) ;

            double ret_lam = min_lam + lam_range * 0.5;
            return DoubleCirFit(ret_lam);
        }

        private FitInfo DoubleCirFit(double lam)
        {
            // lam
            Debug.Assert(
                lam >= 0 && lam <= (c_ / b1_),
                "lam is out of range");

            double gama = (c_ - b1_ * lam) / (a_ * lam + b2_);
            VectorD p_lam = (p_[1] - p_[0]) * lam + p_[0];
            VectorD p_gama = (p_[2] - p_[3]) * gama + p_[3];

            VectorD v_lam_gama = p_gama - p_lam;
            double len_lam_gam = v_lam_gama.L2Norm();
            double len_0_lam = (p_lam - p_[0]).L2Norm();
            double len_fac = len_0_lam / len_lam_gam;
            VectorD p_mid = v_lam_gama * len_fac + p_lam;

#if DEBUG
            {
                double len_3_gama = (p_gama - p_[3]).L2Norm();
                double sum_len = len_0_lam + len_3_gama;
                Debug.Assert(
                    (Number.AlmostZero( (sum_len - len_lam_gam), kDError) || Number.AlmostEqual(sum_len, len_lam_gam, 0.00001)),
                    "Error in calc point p"
                    );

                double len_fac2 = len_3_gama / sum_len;
                VectorD p_mid2 = v_lam_gama * -1 * len_fac2 + p_gama;
                double diff_len = (p_mid - p_mid2).L2Norm();
                Debug.Assert(
                    Number.AlmostZero(diff_len, kDError),
                    "Error in calc point p"
                    );
            }
#endif

            var inter_t_list = VerticalBezierInterPoint(v_lam_gama, p_mid);
            if (!inter_t_list.Any())
            {
               Trace.TraceWarning("Could not find intersection point of 2 circle center line and Bezier line");
            }

            FitInfo ret = new FitInfo();
            ret.mid_point = p_mid;
            ret.fit_cirs = new FitCirInfo[2];

            // C0
            VectorD c0 = CalcCircleOrCenter(v_lam_gama, p_mid, p_[0]);
            ret.fit_cirs[0].c = c0;
            ret.fit_cirs[0].r = (c0 - p_mid).L2Norm();
            ret.fit_cirs[0].t_beg = 0;
            if (inter_t_list.Any())
            {
                ret.fit_cirs[0].t_end = inter_t_list.Max();
            }
            else
            {
                ret.fit_cirs[0].t_end = 1;
            }
            double err1 = MaxDisToCircle(ret.fit_cirs[0]);

            // C1
            VectorD c1 = CalcCircleOrCenter(v_lam_gama, p_mid, p_[3]);
            ret.fit_cirs[1].c = c1;
            ret.fit_cirs[1].r = (c1 - p_mid).L2Norm();
            if (inter_t_list.Any())
            {
                ret.fit_cirs[1].t_beg = inter_t_list.Min();
            }
            else
            {
                ret.fit_cirs[1].t_beg = 0;
            }
            ret.fit_cirs[1].t_end = 1.0;
            double err2 = MaxDisToCircle(ret.fit_cirs[1]);

            ret.max_error = Math.Max(err1, err2);
            return ret;
        }

        #endregion

        private Arc CirInfoToArc(FitCirInfo c_info, VectorD arc_beg, VectorD arc_end)
        {
            // Get arc from circle endpoints
            // Main work is choose which part of circle
            // the arc is decided by start, middle, end point
            // So the work is decide middle point
            VectorD arc_mid =
                FindArcMidPoint(c_info, arc_beg, arc_end);

            Arc ret = new Arc();
            ret.center = VecToPointF(c_info.c);
            ret.radius = c_info.r;

            // Calc angle
            double a_beg = GetLineAngle(c_info.c, arc_beg);
            double a_mid = GetLineAngle(c_info.c, arc_mid);
            double a_end = GetLineAngle(c_info.c, arc_end);
            ret.arc_beg_angle = a_beg;
            ret.arc_angle = GetArcAngle(a_beg, a_mid, a_end);

            // Store arc points
            ret.three_points = new PointF[Arc.p_num];
            ret.three_points[0] = VecToPointF(arc_beg);
            ret.three_points[1] = VecToPointF(arc_mid);
            ret.three_points[2] = VecToPointF(arc_end);

            return ret;
        }

        // Get T value of inflection points
        private List<double> GetInflectionT()
        {
            var ret = new List<double>();

            // Solve xt_d * yt_dd = xt_dd * yt_d

            Polynomial xt_d = bezier_xt_.Derivation();
            Polynomial xt_dd = xt_d.Derivation();
            Polynomial yt_d = bezier_yt_.Derivation();
            Polynomial yt_dd = yt_d.Derivation();

            Polynomial ft_dd_numerator = (xt_d * yt_dd) - (xt_dd * yt_d);
            var possible_t = PolRoot(ft_dd_numerator, 0.0, 1.0);
            // Add 1.0 make it easy to check the sign between interval in possible_t
            if (!possible_t.Any())
            {
                return ret;
            }
#if DEBUG
            {
                possible_t.ForEach(
                    t => Debug.Assert(
                        Number.AlmostZero(ft_dd_numerator.Evaluate(t), kDError),
                        "Pol root error"
                        )
                    );
            }
#endif
            possible_t.Add(1.0);
            var possible_t_d = possible_t.Distinct().ToList();
            possible_t_d.Sort();

            double last_t = 0.0;
            var sign_list = possible_t_d.Select(
                (t) =>
                {
                    double check_t = (last_t + t) / 2;
                    last_t = t;
                    bool sign_dd_numerator = ft_dd_numerator.Evaluate(check_t) > 0;
                    bool sign_dd_denominator = xt_d.Evaluate(check_t) > 0;
                    return sign_dd_denominator == sign_dd_numerator;
                }
            ).ToArray();

            bool last_sign = sign_list.First();

            var possible_t_enu = possible_t_d.GetEnumerator();
            foreach (bool sign in sign_list)
            {
                if (sign != last_sign)
                {
                    ret.Add(possible_t_enu.Current);
                }
                possible_t_enu.MoveNext();
            }

            
            return ret;
        }

        private VectorD GetBezierPoint(double t)
        {

            var ret = Vector.Build.Dense(2);
            ret[0] = bezier_xt_.Evaluate(t);
            ret[1] = bezier_yt_.Evaluate(t);
            return ret;
        }

        private double GetTopPointT()
        {
            // Solve (x3-x0) * yt_d = (y3-y0) * xt_d

            Polynomial xt_d = bezier_xt_.Derivation();
            Polynomial yt_d = bezier_yt_.Derivation();

            // A = x3 - x0
            double A = p_[3][0] - p_[0][0];
            // B = y3 - y0
            double B = p_[3][1] - p_[0][1];

            Polynomial to_solve = (A * yt_d) - (B * xt_d);
            var roots = PolRoot(to_solve, 0.0, 1.0);

            if (roots.Count == 1)
            {
                return roots.First(); 
            }
            Debug.Assert(roots.Any(), "could not find point with given slope");
            if (!roots.Any())
            {
                return 0.5;    
            }

            // Find point with max distance to line p0_p3
            double max_dis = -1;
            double ret = 0.5;
            foreach(double t in roots)
            {
                var point = GetBezierPoint(t);
                double dis = DisToLine(p_[0], p_[3], point);
                if (dis > max_dis)
                {
                    ret = t; 
                }
            }

            return ret;
        }

        private double MaxDisToCircle(FitCirInfo c_info)
        {
            Polynomial x_dis = PolPow((bezier_xt_ - c_info.c[0]), 2);
            Polynomial y_dis = PolPow((bezier_yt_ - c_info.c[1]), 2);
            Polynomial dis_pol = x_dis + y_dis;
            Polynomial dis_pol_d = dis_pol.Derivation();

            List<double> extreme_x = PolRoot(dis_pol_d, c_info.t_beg, c_info.t_end);
            // add end point
            extreme_x.Add(c_info.t_beg);
            extreme_x.Add(c_info.t_end);
            var extreme_val =
                extreme_x.Select(
                    root => {
                        double dis_to_center = Math.Pow(dis_pol.Evaluate(root), 0.5);
                        // dis_to_center may very close to but less than 0
                        if (Double.IsNaN(dis_to_center))
                        {
                            dis_to_center = 0;
                        }
                        return Math.Abs(dis_to_center - c_info.r);
                    }
                );
            double max_dis = extreme_val.Max();

#if DEBUG
            {
                // Test: Check max_val correct
                const double interval = 0.01;
                for (double i = c_info.t_beg; i <= c_info.t_end; i+=interval)
                {
                    double dis_to_center = Math.Pow(dis_pol.Evaluate(i), 0.5);
                    if (!Double.IsNaN(dis_to_center))
                    {
                        double val = Math.Abs(dis_to_center - c_info.r);
                        Debug.Assert(val <= (max_dis + kDError), "Max value wrong");
                    }
                    
                }
            }
#endif
            return max_dis;
        }

        private double MaxDisToPoint(VectorD pt, IEnumerable<double> param_t_list)
        {
            var vec_build = Vector.Build;
            double max_error = 0;
            foreach(double t in param_t_list)
            {
                var bezier_p = GetBezierPoint(t);
                double err = (bezier_p - pt).L2Norm();
                if (err > max_error)
                {
                    max_error = err; 
                }
            }

            return max_error;

        }


        // Calculate the intersection point of the vertical of given vector and Bezier curve
        // Return param t of Bezier curve
        private List<double> VerticalBezierInterPoint(
                VectorD vec, VectorD pt,
                double min_t = 0.0, double max_t = 1.0
            )
        {
            // line (Middle line) of circle centers: Ax + By + C = 0;
            double A = vec[0];
            double B = vec[1];
            double C = -(A * pt[0] + B * pt[1]);
            Polynomial inter_pol =
                (bezier_xt_ * A) + (bezier_yt_ * B) + C;
            var inter_t_list = PolRoot(inter_pol, min_t, max_t);
            // var inter_t_list_all = PolRoot(inter_pol, 0, 1);

#if DEBUG
            {
                // Debug.Assert(inter_t_list.Count != 0);
                foreach (double t in inter_t_list)
                {
                    var inter_p = GetBezierPoint(t);
                    Debug.Assert(
                        Number.AlmostZero(
                            inter_p[0] * A + inter_p[1] * B + C,
                            kDError),
                        "Intersection point not in line");
                    var mid_vec = inter_p - pt;
                    Debug.Assert(
                        Number.AlmostZero(
                            mid_vec.DotProduct(vec),
                            kDError),
                        "Mid line not vertical with vector");
                }
            }
#endif
            return inter_t_list;
        }

        private VectorD FindArcMidPoint(FitCirInfo c_info, VectorD arc_beg, VectorD arc_end)
        {
            var vec_build = Vector.Build;
            VectorD v_endpoints = arc_end - arc_beg;

            // Use polar coordinates for vertical vector of vector endpoints
            double v_x = -v_endpoints[1];
            double v_y = v_endpoints[0];
            double v_theta = Math.Atan2(v_y, v_x);

            // Translate to x-y coordinates
            VectorD m1 = vec_build.Dense(2);
            m1[0] = c_info.r * Math.Cos(v_theta);
            m1[1] = c_info.r * Math.Sin(v_theta);
            VectorD m2 = vec_build.Dense(2);
            m2[0] = -m1[0];
            m2[1] = -m1[1];

            // offset to circle
            m1 += c_info.c;
            m2 += c_info.c;

#if DEBUG
            {
                // Check if m1 and m2 is in the circle
                var v_m1_c = c_info.c - m1;
                var v_m2_c = c_info.c - m2;
                Debug.Assert(
                    Number.AlmostZero((v_m1_c.L2Norm() - c_info.r), kDError),
                    "middle point error");
                Debug.Assert(
                    Number.AlmostZero((v_m2_c.L2Norm() - c_info.r), kDError),
                    "middle point error");
                // Check vector m1_m2 is vertical to v_endp
                var v_m1_m2 = m2 - m1;
                Debug.Assert(
                    Number.AlmostZero(v_endpoints.DotProduct(v_m1_m2), kDError),
                    "middle point error");
            }
#endif

            var t_list = VerticalBezierInterPoint(v_endpoints, c_info.c, c_info.t_beg, c_info.t_end);
            if (!t_list.Any())
            {
                t_list = VerticalBezierInterPoint(v_endpoints, c_info.c, 0, 1);
                if (!t_list.Any())
                {
                    Trace.TraceWarning("May try get mid point of FitInfo with incorrect Bezier Line");
                    // Simple return 1
                    return m1;
                }
            }
            double max_dis1 = MaxDisToPoint(m1, t_list);
            double max_dis2 = MaxDisToPoint(m2, t_list);

            if (max_dis1 < max_dis2)
            {
                return m1;
            }
            else
            {
                return m2;
            }
        }

        #region Private Static Method
        private static Arc MakeLineFitArc(VectorD start_p, VectorD end_p)
        {

            PointF[] pts = new PointF[Arc.p_num];
            pts[0] = VecToPointF(start_p);
            pts[2] = VecToPointF(end_p);
            var p_mid = (start_p + end_p) / 2;
            pts[1] = VecToPointF(p_mid);

            // return
            Arc ret = new Arc();
            ret.is_line = true;
            ret.radius = Double.PositiveInfinity;
            ret.three_points = pts;
            ret.arc_beg_angle = 0;
            ret.arc_angle = 0;

            return ret;
        }

        // Calc circle center, if could not find circle, return center of tan_pt and ins_pt;
        private static VectorD CalcCircleOrCenter(VectorD tangent, VectorD tan_pt, VectorD ins_pt)
        {
            var circle_c = CalcCircle(tangent, tan_pt, ins_pt);
            if (circle_c != null)
            {
                return circle_c;
            }
            else
            {
#if DEBUG
                Trace.TraceInformation("Could not find circle, use center");
#endif
                return (tan_pt + ins_pt) / 2;
            }
        }

        // Calc circle center. 
        // Return null if could not find circle
        private static VectorD CalcCircle(VectorD tangent, VectorD tan_pt, VectorD ins_pt)
        {
            // 切线、切点和交点的中垂线的交点即为圆心
            VectorD p1 = tan_pt;
            VectorD p2 = (ins_pt - tan_pt) / 2 + tan_pt;
            VectorD tangent2 = ins_pt - tan_pt;

            // Check 2 line is not parallel
            if(Number.AlmostZero(
                    (tangent[0] * tangent2[1]) - (tangent[1] * tangent2[0])))
            {
                return null;
            }
            // Vercial line: Ax + By = C
            // A and B store in this matrix
            var mat = DenseMatrix.OfRowVectors(new VectorD[] { tangent, tangent2 });

            // C store in vector
            // Calc vector
            double v1 = mat[0, 0] * p1[0] + mat[0, 1] * p1[1];
            double v2 = mat[1, 0] * p2[0] + mat[1, 1] * p2[1];
            var vec = new DenseVector(new double[] { v1, v2 });

            // Get intersection point
            var ret = mat.Solve(vec);

            // For debug
#if DEBUG
            {
                double r1 = (ret - tan_pt).L2Norm();
                double r2 = (ret - ins_pt).L2Norm();
                Debug.Assert(
                    Number.AlmostZero(r1 - r2, kDError),
                    "Circle point calc error: r different"
                    );
                Debug.Assert(
                    Number.AlmostZero((ret - tan_pt).DotProduct(tangent), kDError),
                    "Circle point calc error: not vectical to tangent point"
                    );
            }
#endif
            return ret;
        }

        private static double GetArcAngle(double angle_beg, double angle_mid, double angle_end)
        {
            double a_beg_mid = (angle_mid - angle_beg + 360.0) % 360.0;
            double a_beg_end = (angle_end - angle_beg + 360.0) % 360.0;

            double a_arc;
            if (a_beg_mid < a_beg_end)
            {
                // CCW
                a_arc = a_beg_end;
            }
            else
            {
                // CW
                a_arc = a_beg_end - 360;
            }

            return a_arc;
        }

        private static double GetLineAngle(VectorD pt_beg, VectorD pt_end)
        {
            VectorD vect = pt_end - pt_beg;
            double radian = Math.Atan2(vect[1], vect[0]);

            const double fac = 180.0 / Math.PI;
            double cw_angle = (radian * fac);

            return (cw_angle + 360) % 360;
        }

        private static List<double> PolRoot(Polynomial pol, double lower_bound, double upper_bound)
        {
            // Translate coefficient
            int max_order = pol.Order;
            while (max_order >= 0)
            {
                if (Number.AlmostZero(pol[max_order]))
                {
                    max_order--;
                }
                else
                {
                    break;
                }
            }

            if (max_order < 0)
            {
                return new List<double>();
            }

            double[] coe = new double[max_order + 1];
            for (int i = 0; i <= max_order; i++)
            {
                coe[i] = pol[max_order - i];
            }

            // Solve
            var roots = RealPolynomialRootFinder.FindRoots(coe);

            // Get result in bounder
            var root_satisfied = new List<double>();
            foreach (var root in roots)
            {
                if (!Number.AlmostZero(root.Imaginary))
                {
                    continue; 
                }
                double root_real = root.Real;
                if (root_real <= upper_bound && root_real >= lower_bound)
                {
                    root_satisfied.Add(root_real);
                }
            }

            return root_satisfied;
        }

        // simple power function of polynomial
        private static Polynomial PolPow(Polynomial pol, int power)
        {
            Polynomial ret = new Polynomial(new double[] { 1.0 });
            for (int i = 0; i < power; i++)
            {
                ret = pol.Multiply(ret);
            }

            return ret;
        }

        // Find point distance from point to line
        private static double DisToLine(VectorD line_p1, VectorD line_p2, VectorD point)
        {
            // Line Vector
            var line_v = line_p2 - line_p1;
            if (Number.AlmostZero(line_v.L2Norm(), kDError))
            {
                // line p2 and line p1 is the same point
                return (point - line_p1).L2Norm();
            }
            // Line: Ax + By + C = 0;
            double A = line_v[1];
            double B = -line_v[0];
            double C = -(A * line_p1[0] + B * line_p1[1]);
            // Distance for point(xt, yt): |A*xt + B*yt + C| / [(A^2 + B^2) ^ 0.5]
            double numerator = Math.Abs(A * point[0] + B * point[1] + C);
            double denominator = Math.Sqrt(A * A + B * B);
            double dis = numerator / denominator;
#if DEBUG
            {
                double dis_to_p1 = (point - line_p1).L2Norm();
                double dis_to_p2 = (point - line_p1).L2Norm();
                Debug.Assert(dis_to_p1 >= dis && dis_to_p2 >= dis, "Distance to line calc error");
            }
#endif

            return dis;
        }

        private static PointD VecToPointD(VectorD vec)
        {
            var ret = new PointD();
            ret.X = vec[0];
            ret.Y = vec[1];
            return ret;
        }

        private static PointF VecToPointF(VectorD vec)
        {
            var ret = new PointF();
            ret.X = (float)vec[0];
            ret.Y = (float)vec[1];
            return ret;
        }

        private static double PointDis(PointF p1, PointF p2)
        {
            float dx = p2.X - p1.X;
            float dy = p2.Y - p1.Y;
            return Math.Sqrt(dx * dx + dy * dy);
        }

        #endregion

#if DEBUG
        //private static int fit_count_ = 0;
#endif
        private const double kDError = 1.0e-5;
        private const int kOrder = 3;
        private const int kCtrlPSize = kOrder + 1;
        private static Polynomial[] bezier_pol;

        private PointD[] ctrl_pts_ = new PointD[kCtrlPSize];
        private VectorD[] p_ = new VectorD[kCtrlPSize];
        private Polynomial bezier_xt_ = new Polynomial(kOrder);
        private Polynomial bezier_yt_ = new Polynomial(kOrder);

        double a_;
        double c_;
        double b1_;
        double b2_;
        #endregion
    }
}
