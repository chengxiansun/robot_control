using RobDrawer.ImageProcessor;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;

namespace RobDrawer.ImageProcessor
{
    public class SVGUtil
    {
        public struct ZoomedSVGGraphic
        {
            public List<List<SVGParser.CurveInfo>> curve_graphic;
            public ImageUtil.ImageZoomInfo zoom_info;
            public SizeF img_size;  // SVG image size
        }

        static public void TransformCurve(List<SVGParser.CurveInfo> curve_path, PointF trans, PointF scale)
        {
            foreach(var draw_info in curve_path)
            {
                PointF[] points = draw_info.points;
                for (int i = 0; i < points.Length; i++)
                {
                    points[i].X *= scale.X;
                    points[i].X += trans.X;
                    points[i].Y *= scale.Y;
                    points[i].Y += trans.Y;
                }
            }
        }

        static public void TransformCurve(List<List<SVGParser.CurveInfo>> curve_graphic, PointF trans, PointF scale)
        {
            foreach(var path in curve_graphic)
            {
                TransformCurve(path, trans, scale);
            }
        }

        public static List<GraphicsPath> ToGraphicPath(IEnumerable<SVGParser.CurveInfo> curve_path)
        {
            GraphicsPath gra_path = new GraphicsPath();
            var gra_path_list = new List<GraphicsPath>();

            foreach (SVGParser.CurveInfo approx_info in curve_path)
            {
                PointF[] pts = approx_info.points;
                switch (approx_info.type)
                {
                    case SVGParser.CurveT.MOVE:
                        if (gra_path.PointCount > 0)
                        {
                            gra_path.CloseFigure();
                            gra_path_list.Add(gra_path);
                            gra_path = new GraphicsPath();
                        }
                        break;

                    case SVGParser.CurveT.LINE:
                        gra_path.AddLine(pts[0], pts[1]);
                        break;

                    case SVGParser.CurveT.BEZIERS:
                        gra_path.AddBezier( pts[0], pts[1], pts[2], pts[3]);
                        break;

                    default:
                        throw new SystemException("Unknown CurveInfo type");
                } // switch
            } //for each

            // Add last path to list
            gra_path_list.Add(gra_path);
            gra_path = null;

            return gra_path_list;
        }


        public static ZoomedSVGGraphic ZoomSVG(SVGParser.SVGInfo svg_info, float width, float height, float margin)
        {
            var curve_gra = svg_info.graphic;

            // Calc transform info
            SizeF src_size = svg_info.size;
            SizeF dest_size = new SizeF();
            dest_size.Width = (width - margin * 2);
            dest_size.Height = (height -margin * 2);

            ImageUtil.ImageZoomInfo zoom_info = ImageUtil.CalcZoomInfo(src_size, dest_size);

            PointF scale = svg_info.scale;
            PointF trans = svg_info.trans;
            // change scale
            scale.X *= zoom_info.scale;
            scale.Y *= zoom_info.scale;
            trans.X *= zoom_info.scale;
            trans.Y *= zoom_info.scale;
            // change transform
            trans.X += zoom_info.trans.X;
            trans.Y += zoom_info.trans.Y;
            trans.X += margin;
            trans.Y += margin;

            // Get draw info
            TransformCurve(curve_gra, trans, scale);

            ZoomedSVGGraphic ret = new ZoomedSVGGraphic();
            ret.curve_graphic = curve_gra;
            ret.zoom_info = zoom_info;
            SizeF img_size = new SizeF(svg_info.size.Width * zoom_info.scale, svg_info.size.Height * zoom_info.scale);
            ret.img_size = img_size;
            return ret;
        }

    }
}
