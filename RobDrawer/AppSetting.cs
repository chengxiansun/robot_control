using RobDrawer.PathPlanner;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;

namespace RobDrawer
{
    class AppSetting
    {
        public AppSetting(bool print_setting)
        {
            this.print_setting_ = print_setting;
            DoReadSetting();
        }

        public PathUtil.FillLineParam kFillLineParam { get; private set; }
        public CurveFit.ArcLimit kArcLimit {get; private set;}
        public Size kSvgOutSize {get; private set;}

        public float kFitMaxError {get; private set;}
        public float kFitScaleFactor {get; private set;}

        public float kPaperWidth {get; private set;}
        public float kPaperHeight {get; private set;}
        public float kPaperMargin {get; private set;}

        public String kThirdPartyDir {get; private set;}

        #region Private
        private void DoReadSetting()
        {
            TryConsoleWriteLine("");

            // Draw paper
            kPaperWidth = ReadFloatSetting("PaperWidth");
            kPaperHeight = ReadFloatSetting("PaperHeight");
            kPaperMargin = ReadFloatSetting("PaperMargin");

            // SVG image size
            int svg_w = ReadIntSetting("SVGWidth");
            int svg_h = ReadIntSetting("SVGHeight");
            kSvgOutSize = new System.Drawing.Size(svg_w, svg_h);

            // Bezier fitting
            kFitScaleFactor = ReadFloatSetting("FitScale");
            kFitMaxError = ReadFloatSetting("FitMaxError") * kFitScaleFactor;

            // Arc limit
            float _min_r = ReadFloatSetting("ArcMinRadius");
            float _min_arc_angle = ReadFloatSetting("ArcMinAngle");
            float _min_pt_dis = ReadFloatSetting("ArcMinPtDis") * kFitScaleFactor;
            kArcLimit = new CurveFit.ArcLimit() { 
                min_r =  _min_r, min_arc_angle = _min_arc_angle, min_pt_dis = _min_pt_dis
            };

            // Fill line
            float _row_dis = ReadFloatSetting("FLRowDis") * kFitScaleFactor;
            float _line_shrink_size = ReadFloatSetting("FLLineShrinkSize") * kFitScaleFactor;
            kFillLineParam = new PathUtil.FillLineParam()
            {
                row_dis = _row_dis,
                line_shrink_size = _line_shrink_size
            };

            // Third party dir
            kThirdPartyDir = ReadSetting("DIR_ThirdParty");

            TryConsoleWriteLine("");
        }


        // Read setting from app configure, return and print in console
        // If key is not found, throw KeyNotFoundException
        private string ReadSetting(string key)
        {
            var app_settng = System.Configuration.ConfigurationManager.AppSettings;
            var ret = app_settng[key];
            string log_str = string.Format("\t{0}\t: {1}", key, ret ?? "Key Not Found!");
            TryConsoleWriteLine(log_str);

            if (ret == null)
            {
                throw new KeyNotFoundException("Key: " + key);
            }

            return ret;
        }

        private int ReadIntSetting(string key)
        {
            string val = ReadSetting(key);
            return int.Parse(val);
        }

        private float ReadFloatSetting(string key)
        {
            string val = ReadSetting(key);
            return float.Parse(val);
        }

        private void TryConsoleWriteLine(string str)
        {
            if (print_setting_)
            {
                Trace.WriteLine(str);
            }
        }

        private bool print_setting_;
        #endregion
    }
}
