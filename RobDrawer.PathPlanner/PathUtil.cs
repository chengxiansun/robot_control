using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using RobDrawer.ImageProcessor;
using System.Text;

namespace RobDrawer.PathPlanner
{
    public class PathUtil
    {

        public struct LineF
        {
            public LineF(PointF start, PointF end)
            {
                this.start = start;
                this.end = end;
            }

            public PointF start;
            public PointF end;
        }

        public struct FillLineParam
        {
            public float row_dis;               // distance for row
            public float line_shrink_size;      // shrink of endpoint of line 
            public float min_length;            // Line shorter than min_length will be ignored
        }


        public class PlanningPath
        {
            public PlanningPath (List<List<CurveFit.ApproxCurveInfo>> approx_boundary, List<LineF> filling_lines)
            {
                this.ApproxBoundary = approx_boundary;
                this.FillingLines = filling_lines;
            }

            public void ToImage(Image dst, Pen p)
            {
                Graphics graphic = Graphics.FromImage(dst);

                // Draw boundary
                foreach (var approx_path in this.ApproxBoundary)
                {
                    List<GraphicsPath> gra_paths = PathUtil.ToGraphicPath(approx_path);
                    foreach (GraphicsPath gra_path in gra_paths)
                    {
                        graphic.DrawPath(p, gra_path);
                    }
                }

                // Draw lines
                foreach (LineF line in this.FillingLines)
                {
                    graphic.DrawLine(p, line.start, line.end);
                }
            }

            public string ToYaml()
            {
                /* Yaml Format
                 * ---
                 * Boundary:
                 *   - Graphic:
                 *     - Path:
                 *       - Line: [1.0, 2.0, 3.0, 4.0]
                 *       - Line: [1.1, 2.1, 3.1, 4.1]
                 *       - Arc:  [1.2, 2.2, 3.3, 4.3, 5.3, 6.3]
                 *       #...
                 *     - Path:
                 *       #...
                 *   - Graphic:
                 *     #...
                 * FillingLines:
                 *   - [1.0, 2.0, 3.0, 4.0]
                 *   - [1.0, 2.0, 3.0, 4.0]
                 *   - [1.0, 2.0, 3.0, 4.0]
                 *   - [1.0, 2.0, 3.0, 4.0]
                 *   #...
                 * */
                StringBuilder sb = new StringBuilder();
                // Begin
                AppendYamlLine(sb, 0, "---");

                // Boundary
                AppendYamlLine(sb, 0, "Boundary:");
                // For each graphic
                foreach (List<CurveFit.ApproxCurveInfo> path in ApproxBoundary)
                {
                    AppendYamlLine(sb, 1, "- Graphic:");
                    if (path.Any())
                    {
                        if (path.First().type != CurveFit.ApproxCurveT.MOVE)
                        {
                            AppendYamlLine(sb, 2, "- Path:");
                        }
                    }

                    foreach(CurveFit.ApproxCurveInfo curve_info in path)
                    {
                        switch (curve_info.type)
                        {
                            case CurveFit.ApproxCurveT.MOVE:
                                // New path
                                AppendYamlLine(sb, 2, "- Path:");
                                break;
                            case CurveFit.ApproxCurveT.LINE:
                                {
                                    string cmd = String.Format(
                                        "- Line: [{0}, {1}, {2}, {3}]",
                                        curve_info.pts[0].X,
                                        curve_info.pts[0].Y,
                                        curve_info.pts[1].X,
                                        curve_info.pts[1].Y
                                        );

                                    AppendYamlLine(sb, 3, cmd);
                                }
                                break;
                            case CurveFit.ApproxCurveT.ARC:
                                {
                                    string cmd = String.Format(
                                        "- Arc:  [{0}, {1}, {2}, {3}, {4}, {5}]",
                                        curve_info.arc.Value.three_points[0].X,
                                        curve_info.arc.Value.three_points[0].Y,
                                        curve_info.arc.Value.three_points[1].X,
                                        curve_info.arc.Value.three_points[1].Y,
                                        curve_info.arc.Value.three_points[2].X,
                                        curve_info.arc.Value.three_points[2].Y
                                        );

                                    AppendYamlLine(sb, 3, cmd);
                                }
                                break;
                            default:
                                break;
                        }
                    }
                }

                // FillingLines
                AppendYamlLine(sb, 0, "FillingLines:");
                foreach(LineF line in FillingLines)
                {
                    string cmd = String.Format(
                        "- [{0}, {1}, {2}, {3}]",
                        line.start.X,
                        line.start.Y,
                        line.end.X,
                        line.end.Y
                        );

                    AppendYamlLine(sb, 2, cmd);
                }

                return sb.ToString();

            }

            public List<List<CurveFit.ApproxCurveInfo>> ApproxBoundary { get; private set; }
            public List<LineF> FillingLines { get; private set; }

            private static void AppendYamlLine(StringBuilder string_builder, int indent_level, string str)
            {
                for (int i = 0; i < indent_level; i++)
                {
                    string_builder.Append("  ");
                }
                string_builder.Append(str);
                string_builder.Append("\n");
            }
        }

        public static List<CurveFit.ApproxCurveInfo> LinesToMovement(IEnumerable<LineF> lines)
        {

            var path_ret = new List<CurveFit.ApproxCurveInfo>();

            foreach (var l in lines)
            {
                // Add Move info
                {
                    var motion_move = new CurveFit.ApproxCurveInfo();
                    motion_move.type = CurveFit.ApproxCurveT.MOVE;
                    motion_move.pts = new PointF[1];
                    motion_move.pts[0] = l.start;
                    path_ret.Add(motion_move);
                }

                // Add line info
                {
                    var motion_line = new CurveFit.ApproxCurveInfo();
                    motion_line.type = CurveFit.ApproxCurveT.LINE;
                    motion_line.pts = new PointF[2];
                    motion_line.pts[0] = l.start;
                    motion_line.pts[1] = l.end;
                    path_ret.Add(motion_line);
                }
            }

            return path_ret;
        }

        public static PlanningPath PlanPath(
            List<List<SVGParser.CurveInfo>> curve_gra, 
            double fit_max_err, CurveFit.ArcLimit arc_limit, FillLineParam fill_param)
        {
            var approx_gra = new List<List<CurveFit.ApproxCurveInfo>>();
            var filling_lines = new List<LineF>();

            foreach(var curve_path in curve_gra)
            {
                // FitCurvePath
                CurveFit curve_fit = new CurveFit(curve_path, fit_max_err, arc_limit);
                List<CurveFit.ApproxCurveInfo> approx_path = curve_fit.GetResultEnum().ToList();
                approx_gra.Add(approx_path);

                // Get filled lines
                List<PathUtil.LineF> lines = FilledWithLineZigzag(approx_path, fill_param);
                filling_lines.AddRange(lines);
            }

            PlanningPath ret = new PlanningPath(approx_gra, filling_lines);
            return ret;
        }

        public static List<LineF> FilledWithLineZigzag(IEnumerable<CurveFit.ApproxCurveInfo> approx_path, FillLineParam fill_param)
        {
            // TODO: Fix this trick
            List<GraphicsPath> gra_path = PathUtil.ToGraphicPath(approx_path);
            RectangleF bound = gra_path.First().GetBounds();
            Region gra_reg = PathUtil.GetPathRegion(gra_path);
            List<PathUtil.LineF> lines = PathUtil.FilledWithLineZigzag(gra_reg, bound, fill_param);
            return lines;
        }

        public static List<LineF> FilledWithLineZigzag(Region region, RectangleF bound, FillLineParam fill_param)
        {
            const float TestLineWidth = 1F;
            var mat = new System.Drawing.Drawing2D.Matrix();
            var lines = new LinkedList<LinkedList<LineF>>();

            RectangleF line_rect = new RectangleF( );
            line_rect.Width = bound.Width;
            line_rect.Height = TestLineWidth;
            line_rect.X = bound.X;
            float y = bound.Y - (fill_param.row_dis / 2.0F);

            Region test_region = region.Clone();

            while(y < bound.Bottom)
            {
                line_rect.Y = y;
                // Why can't I just get intersection region but not update the source?
                test_region.Intersect(line_rect);

                var rects = test_region.GetRegionScans(mat);

                if (rects.Any())
                {
                    var row_line = new LinkedList<LineF>();
                    foreach(var rect in rects)
                    {
                        PointF start_p = new PointF(rect.Left + fill_param.line_shrink_size, y);
                        PointF end_p = new PointF(rect.Right - fill_param.line_shrink_size, y);
                        if ( (end_p.X - start_p.X) >= fill_param.min_length)
                        {
                            row_line.AddLast(new LineF(start_p, end_p));
                        }
                    }
                    if (row_line.Any())
                    {
                        lines.AddLast(row_line);
                    }
                }

                // Make test_region back to region
                test_region.Union(region);
                y += fill_param.row_dis;
            }

            return OptmizeFillPathZigzag(lines);
        }

        public static List<GraphicsPath> ToGraphicPath(IEnumerable<CurveFit.ApproxCurveInfo> approx_path)
        {
            GraphicsPath gra_path = new GraphicsPath();
            var gra_path_list = new List<GraphicsPath>();

            foreach (CurveFit.ApproxCurveInfo approx_info in approx_path)
            {
                approx_info.CheckParam(alert_assert_fail: true);
                switch (approx_info.type)
                {
                    case CurveFit.ApproxCurveT.MOVE:
                        if (gra_path.PointCount > 0)
                        {
                            gra_path.CloseFigure();
                            gra_path_list.Add(gra_path);
                            gra_path = new GraphicsPath();
                        }
                        break;

                    case CurveFit.ApproxCurveT.LINE:
                        PointF[] pts = approx_info.pts;
                        gra_path.AddLine(pts[0], pts[1]);
                        break;

                    case CurveFit.ApproxCurveT.ARC:
                        AddFitArc(gra_path, approx_info.arc.Value);
                        break;

                    default:
                        throw new SystemException("Unknown ApproxCurveInfo type");
                } // switch
            } //for each

            // Add last path to list
            gra_path_list.Add(gra_path);
            gra_path = null;

            return gra_path_list;
        }


        // Get region surrounded by path. The first region in path_lis is mother region, and remain region are holes.
        public static Region GetPathRegion(IEnumerable<GraphicsPath> path_list)
        {
            var g_itera = path_list.GetEnumerator();
            g_itera.MoveNext();
            Region ret = new Region(g_itera.Current);
            while (g_itera.MoveNext())
            {
                Region hole_region = new Region(g_itera.Current);
                ret.Exclude(hole_region);
            }

            return ret;
        }


        #region Private
        private static float DisPointF(PointF p1, PointF p2)
        {
            double square_sum = Math.Pow((p2.X - p1.X), 2f) + Math.Pow((p2.Y - p1.Y), 2);
            return (float)Math.Sqrt(square_sum);
        }

        // @return: next itera related to need_reverse
        private static LinkedListNode<LineF> MoveLineFToList(
                        LinkedListNode<LineF> itera, ICollection<LineF> dest, bool need_move_reverse )
        {
            if (itera == null)
            {
                throw new Exception("itera to push is null");
            }

            // move and get next itera
            LinkedListNode<LineF> itera_next;
            LineF item = itera.Value;
            if ( !need_move_reverse)
            {
                dest.Add(new LineF(item.start, item.end));
                itera_next = itera.Next;
            }
            else
            {
                dest.Add(new LineF(item.end, item.start));
                itera_next = itera.Previous;
            }

            // remove and ret
            itera.List.Remove(itera);
            return itera_next;
        }

        private static Tuple<LinkedListNode<LineF>, float>
            ClosestLinef(LineF src, LinkedList<LineF> candidate, bool is_now_move_reverse)
        {
            if (!candidate.Any())
            {
                throw new Exception("Candidate list is empty");
            }

            float min_dis = float.MaxValue;
            LinkedListNode<LineF> closest_itera = null;

            var itera = candidate.First;
            while (itera != null)
            {
                float dis = LineJumpDisInterRow(src, itera.Value, is_now_move_reverse);
                if (dis <= min_dis)
                {
                    min_dis = dis;
                    closest_itera = itera;
                }

                itera = itera.Next;
            }

            return new Tuple<LinkedListNode<LineF>, float>(closest_itera, min_dis);
        }


        private static float LineJumpDisInRow(LineF l_start, LineF l_end, bool is_move_reverse)
        {
            if (!is_move_reverse)
            {
                return DisPointF(l_start.end, l_end.start);
            }
            else
            {
                return DisPointF(l_start.start, l_end.end);
            }
        }

        private static float LineJumpDisInterRow(LineF l_start, LineF l_end, bool is_now_move_reverse)
        {
            if (!is_now_move_reverse)
            {
                return DisPointF(l_start.end, l_end.end);
            }
            else
            {
                return DisPointF(l_start.start, l_end.start);
            }
        }

        // @return element item removed
        private static int CleanEmptyElem<T>(LinkedList<LinkedList<T>> l)
        {
            if (!l.Any())
            {
                return 0;
            }

            int item_removed = 0;
            LinkedListNode<LinkedList<T>> out_iter = l.First;
            while (out_iter != null)
            {
                var in_item = out_iter.Value;
                if (!in_item.Any())
                {
                    var next = out_iter.Next;
                    out_iter.List.Remove(out_iter);
                    ++item_removed;
                    out_iter = next;
                }
                else
                {
                    out_iter = out_iter.Next;
                }
            }

            return item_removed;
        }


        private static List<LineF> OptmizeFillPathZigzag(LinkedList<LinkedList<LineF>> lines)
        {
            var lines_ret = new List<LineF>();
            int empty_item = CleanEmptyElem(lines);
            if (empty_item != 0)
            {
                Trace.TraceWarning("Empty row in lines, there may be some error when generate lines' informations");
            }

            if (!lines.Any())
            {
                return lines_ret;
            }


            bool is_move_reverse = false; // Is line need reverse move
            bool is_row_reverse = false;    // Is row traversal from down to up 
            LinkedListNode<LinkedList<LineF>> row_i = lines.First;
            LinkedListNode<LineF> next_itera = lines.First().First;
            if (next_itera == null)
            {
                // row could not be empty because I have clean it before, else there's some bug
                throw new Exception("empty row in list");
            }

            while (true)
            {
                var last_item = next_itera.Value;
                var next_in_row = MoveLineFToList(next_itera, lines_ret, is_move_reverse);

                var next_row_i = is_row_reverse ? row_i.Previous : row_i.Next;
                if (next_row_i == null)
                {
                    // Reverse row order
                    is_row_reverse ^= true;
                    next_row_i = is_row_reverse ? row_i.Previous : row_i.Next;
                }

                // Check if need delete current row
                if (!row_i.Value.Any())
                {
                    // clean
                    lines.Remove(row_i);
                    if (!lines.Any())
                    {
                        break;
                    }
                }

                float dis_in_row;
                if (next_in_row == null)
                {
                    dis_in_row = float.MaxValue;
                }
                else
                {
                    dis_in_row = LineJumpDisInRow(last_item, next_in_row.Value, is_move_reverse);
                }


                if (next_row_i != null)
                {
                    Tuple<LinkedListNode<LineF>, float> closest_next_r =
                            ClosestLinef(last_item, next_row_i.Value, is_move_reverse);

                    if (closest_next_r.Item2 <= dis_in_row)
                    {
                        next_itera = closest_next_r.Item1;
                        // Move to next line
                        is_move_reverse ^= true;
                        row_i = next_row_i;
                    }
                    else
                    {
                        next_itera = next_in_row;
                    }
                }
                else
                {
                    // There is no next row
                    next_itera = next_in_row;
                }

                // If there's only elements in current row and in the backward size
                // the next_in_row and next_row_i will both be null
                // therefore next_itera will be null
                // Just reset next_itera and row_i to the first element to solved this problem
                if (next_itera == null)
                {
                    row_i = lines.First;
                    next_itera = lines.First().First;
                }

            }
            return lines_ret;
        }

        private static void AddFitArc(GraphicsPath gra_path, CurveFit.Arc arc)
        {
            if (arc.is_line)
            {
                gra_path.AddLine(arc.three_points.First(), arc.three_points.Last());
            }
            else
            {
                float r = (float)arc.radius;
                gra_path.AddArc(arc.center.X - r, arc.center.Y - r, r * 2, r * 2, (float)arc.arc_beg_angle, (float)arc.arc_angle);
            }
        }

        #endregion
    }
}
