using System;
using System.Collections.Generic;
using System.Linq;
using System.Drawing;
using System.Diagnostics;
using System.Xml;
using Ader.Text;

namespace RobDrawer.ImageProcessor
{
    public class SVGParser
    {
        #region Public methods

        public enum CurveT
        {
            LINE,       // A line defined by 2 points
            BEZIERS,    // A beziers curve defined by 4 points
            MOVE        // A move command by 1 point
        }

        // Curve info with abs point and a start point
        public struct CurveInfo
        {
            public CurveT type;
            public PointF[] points;
        }

        public struct SVGInfo
        {
            public SizeF size;
            public PointF trans;
            public PointF scale;
            public List<List<CurveInfo>> graphic;
        }

        static public SVGInfo ParseSVG(string xml_str)
        {
            SVGInfoRaw info_raw = ParseSVGRaw(xml_str);
            List<List<CurveInfo>> curve_gra = TransferToCurveGraphic(info_raw.graphic);

            SVGInfo ret = new SVGInfo()
            {
                size = info_raw.size,
                trans = info_raw.trans,
                scale = info_raw.scale,
                graphic = curve_gra
            };

            return ret;
        }

        #endregion

        #region Public methods for raw svg info parser for low-level usage
        public enum SVGPathCmd
        {
            MOVE,
            LINETO,
            BEZIERS,
            CLOSE,
            NOT_A_CMD,
        }

        // Path cmd info parse from SVG file
        public struct SVGPathCmdInfo
        {
            public bool is_abs;
            public SVGPathCmd cmd;
            public PointF[] points;
        }

        public struct SVGInfoRaw
        {
            public SizeF size;
            public PointF trans;
            public PointF scale;
            public List<List<SVGPathCmdInfo>> graphic;
        }

        static public SVGInfoRaw ParseSVGRaw(string xml_str)
        {
            XmlDocument xml_doc = new XmlDocument();
            xml_doc.XmlResolver = null;
            xml_doc.LoadXml(xml_str);

            SVGInfoRaw svginfo_out = new SVGInfoRaw();
            svginfo_out.size = new SizeF(0F, 0F);
            svginfo_out.trans = new PointF(0F, 0F);
            svginfo_out.scale = new PointF(1F, 1F);
            svginfo_out.graphic = new List<List<SVGPathCmdInfo>>();

            var nsmgr = new XmlNamespaceManager(xml_doc.NameTable);
            nsmgr.AddNamespace("ns", xml_doc.DocumentElement.NamespaceURI);

            // Get size info
            var svg_node = xml_doc.SelectSingleNode("/ns:svg", nsmgr);
            svginfo_out.size = ParseSVGSizeInfo(svg_node);
            
            // Get g node
            var g_nodes = svg_node.SelectNodes("ns:g", nsmgr);
            if (g_nodes.Count == 0)
            {
                Trace.TraceWarning(TAG + "Could not find <g> node");
                return svginfo_out;
            }
            if (g_nodes.Count > 1)
            {
                Trace.TraceWarning(TAG + "Too much <g> node, only process first one");
            }
            var g_node = g_nodes[0];
            // Get transform info
            var transform_attr = g_node.Attributes["transform"];
            if (transform_attr != null)
            {
                string trans_str = transform_attr.Value;
                svginfo_out.trans = ParseSVGInfoPoint(trans_str, "translate", svginfo_out.trans);
                svginfo_out.scale = ParseSVGInfoPoint(trans_str, "scale", svginfo_out.trans);
            }

            var path_list = g_node.SelectNodes("ns:path/@d", nsmgr);

            foreach (XmlNode path_node in path_list)
            {
                string path_str = path_node.Value;
                var svg_path = ParseSVGPath(path_str);
                svginfo_out.graphic.Add(svg_path);
            }

            return svginfo_out;
        }

        static public List<List<CurveInfo>> TransferToCurveGraphic(List<List<SVGPathCmdInfo>> svg_graphic)
        {
            var gra_ret = new List<List<CurveInfo>>();
            foreach(var path in svg_graphic)
            {
                var curve_path = new List<CurveInfo>();
                var path_convertor = new PathInfoConvertor();
                CurveInfo curve_info = new CurveInfo();
                foreach(var path_info in path)
                {
                    if (path_convertor.Convert(path_info, ref curve_info) )
                    {
                        curve_path.Add(curve_info);
                    }
                }

                gra_ret.Add(curve_path);
            }
            return gra_ret;
        }

        #endregion

        // ================= Private ===================================
        #region Private

        static SVGParser()
        {

            int cmd_size = Enum.GetNames(typeof(SVGPathCmd)).Length;

            // Init cmd_ctrl_point
            path_cmd_pt_num_ = new int[cmd_size];
            path_cmd_pt_num_[(int)SVGPathCmd.MOVE] = 1;
            path_cmd_pt_num_[(int)SVGPathCmd.LINETO] = 1;
            path_cmd_pt_num_[(int)SVGPathCmd.BEZIERS] = 3;
            path_cmd_pt_num_[(int)SVGPathCmd.CLOSE] = 0;
            path_cmd_pt_num_[(int)SVGPathCmd.NOT_A_CMD] = 0;

            // Init cmd_map
            cmd_map_ = new Dictionary<char, SVGPathCmd>();
            cmd_map_.Add('M', SVGPathCmd.MOVE);
            cmd_map_.Add('L', SVGPathCmd.LINETO);
            cmd_map_.Add('C', SVGPathCmd.BEZIERS);
            cmd_map_.Add('Z', SVGPathCmd.CLOSE);
        }

        static private List<SVGPathCmdInfo> ParseSVGPath(string path_str)
        {
            var path_ret = new List<SVGPathCmdInfo>();
            var tokens = ScanStr(path_str).GetEnumerator();

            // A SVG path command may be followed by many groups of (param) points
            // Each group of points and the command make up a cmd info
            SVGPathCmd last_cmd = SVGPathCmd.NOT_A_CMD;
            bool is_abs = false;
            // Parse
            while(tokens.MoveNext())
            {
                string token_str = tokens.Current.Value;
                // Check if a command
                if (IsCmd(token_str))
                {
                    var path_infos = GetCmds(token_str);
                    Debug.Assert(path_infos.Any());

                    // Push all command that do not need any param
                    foreach(var info in path_infos)
                    {
                        if (path_cmd_pt_num_[(int)info.cmd] == 0)
                        {
                            SVGPathCmdInfo need_info = info;
                            need_info.points = new PointF[0];
                            path_ret.Add(need_info);
                        }
                    }

                    // Store last command
                    var last_info = path_infos.ElementAt(path_infos.Count - 1);
                    last_cmd = last_info.cmd;
                    is_abs = last_info.is_abs;
                }
                else
                {
                    // Not a command, Try read command's param
                    if (last_cmd == SVGPathCmd.NOT_A_CMD)
                    {
                        Trace.WriteLine(TAG + "Not recognize cmd, ignore data");
                        continue;
                    }

                    // Read params (points)
                    PointF[] points;
                    int point_num = path_cmd_pt_num_[(int)last_cmd];
                    bool success = ReadPoints(point_num, ref tokens, out points);

                    if (success)
                    {
                        SVGPathCmdInfo path_cmd = new SVGPathCmdInfo();
                        path_cmd.is_abs = is_abs;
                        path_cmd.cmd = last_cmd;
                        path_cmd.points = points;

                        path_ret.Add(path_cmd);
                    }
                    else
                    {
                        Trace.TraceWarning(TAG + "Could not get enough data for command");
                    }

                }

            }
            return path_ret;
        }

        private class PathInfoConvertor
        {
            public bool Convert(SVGPathCmdInfo path_info, ref CurveInfo out_info)
            {
                // Check command type
                PointF[] src_points = null;
                bool is_abs = path_info.is_abs;
                bool need_beg_pt;

                switch (path_info.cmd)
                {
                    case SVGPathCmd.MOVE:
                        out_info.type = CurveT.MOVE;
                        sub_path_beg_ =
                            path_info.is_abs ?
                            path_info.points[0] :
                            Offset(path_info.points[0], cur_pos_);
                        need_beg_pt = false;
                        src_points = path_info.points;
                        Debug.Assert(src_points.Length == 1, "Wrong point number of SVG Path command");
                        break;

                    case SVGPathCmd.LINETO:
                        out_info.type = CurveT.LINE;
                        need_beg_pt = true;
                        src_points = path_info.points;
                        Debug.Assert(src_points.Length == 1, "Wrong point number of SVG Path command");
                        break;

                    case SVGPathCmd.BEZIERS:
                        out_info.type = CurveT.BEZIERS;
                        need_beg_pt = true;
                        src_points = path_info.points;
                        Debug.Assert(src_points.Length == 3, "Wrong point number of SVG Path command");
                        break;

                    case SVGPathCmd.CLOSE:
                        out_info.type = CurveT.LINE;
                        need_beg_pt = true;
                        src_points = new PointF[1] { sub_path_beg_ };
                        is_abs = true;
                        break;

                    default:
                        need_beg_pt = false;
                        src_points = null;
                        break;
                }

                // Convert point
                if (src_points != null)
                {
                    out_info.points = this.ConvertPoint(src_points, is_abs, need_beg_pt);
                    return true;
                }
                else
                {
                    return false;
                }
            }

            private PointF[] ConvertPoint(PointF[] path_points, bool is_abs, bool need_beg_pt)
            {
                int beg_pt_off = need_beg_pt ? 1 : 0;
                PointF[] ret = new PointF[path_points.Length + beg_pt_off];

                if (need_beg_pt)
                {
                    ret[0] = cur_pos_;
                }
                for (int i = 0; i < path_points.Length; i++)
                {
                    ret[i + beg_pt_off] =
                           is_abs ?
                           path_points[i] :
                           Offset(path_points[i], cur_pos_);

                }

                cur_pos_ = ret.Last();
                return ret;
            }
            private static PointF Offset(PointF src, PointF off)
            {
                return new PointF(src.X + off.X, src.Y + off.Y);
            }

            private PointF cur_pos_ = new PointF(0F, 0F);
            private PointF sub_path_beg_ = new PointF(0F, 0F);

        }

        private static SizeF ParseSVGSizeInfo(XmlNode svg_node)
        {
            SizeF size_info = new SizeF(0F, 0F);
            string width_str = svg_node.Attributes["width"].Value;
            string height_str = svg_node.Attributes["height"].Value;
            foreach (Token token in ScanStr(width_str))
            {
                if (token.Kind == TokenKind.Number)
                {
                    size_info.Width = float.Parse(token.Value);
                    break;
                }
            }
            foreach (Token token in ScanStr(height_str))
            {
                if (token.Kind == TokenKind.Number)
                {
                    size_info.Height = float.Parse(token.Value);
                    break;
                }
            }
            return size_info;
        }

        private static PointF ParseSVGInfoPoint(string str, string cmd, PointF def_val )
        {
            float[] point_data = new float[2];
            // Get command
            var tokens = ScanStr(str).GetEnumerator();
            while(tokens.MoveNext())
            {
                Token token = tokens.Current;
                if (token.Value.Equals(cmd))
                {
                    break; 
                }
            }

            int i = 0;
            while(i < point_data.Length && tokens.MoveNext())
            {
                Token token = tokens.Current;
                if (token.Kind == TokenKind.Number)
                {
                    point_data[i] = float.Parse(token.Value);
                    ++i;
                }
            }

            if (i == point_data.Length)
            {
                // get success
                return new PointF(point_data[0], point_data[1]);
            }
            else
            {
                return def_val;
            }
        }

        private static bool ReadPoints(int point_num, ref List<Token>.Enumerator tokens_beg, out PointF[] points_out)
        {
            points_out = new PointF[point_num];
            if (point_num == 0)
            {
                return true;
            }

            float[] data_buffer = new float[point_num * 2];

            data_buffer[0] = float.Parse(tokens_beg.Current.Value);
            for (int i = 1; i < point_num * 2; i++)
            {
                if (tokens_beg.MoveNext())
                {
                    try
                    {
                        data_buffer[i] = float.Parse(tokens_beg.Current.Value);
                    }
                    catch (Exception e)
                    {
                        // Just throw...
                        throw e;
                    }
                }
                else
                {
                    points_out = null;
                    return false;
                }
            }

            for (int i = 0; i < point_num; i++ )
            {
                points_out[i].X = data_buffer[i * 2];
                points_out[i].Y = data_buffer[i * 2 + 1];
            }

            return true;
        }

        private static List<Token> ScanStr(string str)
        {
            StringTokenizer str_tok = new StringTokenizer(str);
            str_tok.IgnoreWhiteSpace = true;
            str_tok.SymbolChars = new char[] { ',', '(',  ')'};

            Token token = str_tok.Next();
            List<Token> output = new List<Token>();
            while(token.Kind != TokenKind.EOF)
            {
                switch(token.Kind)
                {
                    case TokenKind.Number:
                    case TokenKind.QuotedString:
                    case TokenKind.Unknown:
                    case TokenKind.Word:
                        output.Add(token);
                        break;

                    default:
                        break;

                }
                token = str_tok.Next();
            }
            return output;
        }

        static private bool IsCmd(string token_str)
        {
            return Char.IsLetter(token_str[0]);
        }

        static private List<SVGPathCmdInfo> GetCmds(string token_str)
        {
            bool is_abs = false;

            var ret = new List<SVGPathCmdInfo>();
            if (!IsCmd(token_str))
            {
                return ret;
            }

            foreach(char c in token_str)
            {
                is_abs = Char.IsUpper(c);
                try
                {
                    SVGPathCmd result = cmd_map_[Char.ToUpper(c)];
                    SVGPathCmdInfo cmd_info = new SVGPathCmdInfo(); ;
                    cmd_info.cmd = result;
                    cmd_info.is_abs = is_abs;
                    ret.Add(cmd_info);

                }
                catch (KeyNotFoundException)
                {
                    SVGPathCmdInfo not_cmd_info = new SVGPathCmdInfo();
                    not_cmd_info.is_abs = is_abs;
                    not_cmd_info.cmd = SVGPathCmd.NOT_A_CMD;
                    ret.Add(not_cmd_info);
                }
            }

            return ret;
        }

        private const string TAG = "SVGPathParser";
        // TODO: Change path_cmd_point_num_ to map
        private static int[] path_cmd_pt_num_;
        private static Dictionary<char, SVGPathCmd> cmd_map_;

        #endregion
    }
}
