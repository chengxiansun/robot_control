using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Drawing;
using System.IO;
using RobDrawer.ImageProcessor;

namespace RobDrawer
{
    class Program
    {
        static void Main(string[] args)
        {
            var program = new Program(args);
            program.BeginWork();
        }

        public Program(string[] args)
        {
            setting_ = new AppSetting(print_setting: false);
            opts = new Options();

            if (!CommandLine.Parser.Default.ParseArguments(args, opts))
            {
                Environment.Exit(CommandLine.Parser.DefaultExitCodeFail);
            }

            cmd_list_ = new List<Options.DebugCommandType>();

            ImageProcessor.ImageUtil.InitParam init_param = new ImageProcessor.ImageUtil.InitParam();
            init_param.thirdparty_abs_dir = GetAbsThirdPartyDir(setting_.kThirdPartyDir);
            img_util_ = new ImageProcessor.ImageUtil(init_param);
        }

        class DataWrapper
        {
            public DataWrapper(byte[] bytes)
            {
                this.DataBytes = bytes;
                this.DataObjs = null;
                this.IsBytes = true;
            }

            // Params obj_is_not_bytes is used to overload when obj is byte[]
            public DataWrapper(Object obj, bool obj_is_not_bytes = true)
            {
                if (obj_is_not_bytes == false)
                {
                    throw new SystemException("Wrong argument");
                }
                this.DataBytes = null;
                this.DataObjs = obj;
                this.IsBytes = false;
            }

            public void WriteToStream(Stream out_stream)
            {
                if (this.IsBytes)
                {
                    out_stream.Write(this.DataBytes, 0, this.DataBytes.Length);
                }
                else
                {
                    if (DataObjs is Bitmap)
                    {
                        Bitmap bm = (Bitmap)DataObjs;
                        bm.Save(out_stream, System.Drawing.Imaging.ImageFormat.Jpeg);
                    }
                    else if (DataObjs is string)
                    {
                        string str = (string)DataObjs;
                        StreamWriter sw = new StreamWriter(out_stream);
                        sw.Write(str);
                        sw.Flush();
                        sw.Close();
                    }
                }
            }

            public Bitmap TryGetBitmap()
            {
                if (this.IsBytes)
                {
                    Bitmap bmp;
                    using (var ms = new MemoryStream(this.DataBytes))
                    {
                        bmp = new Bitmap(ms);
                    }
                    // new a Bitmap to free the lock
                    return new Bitmap(bmp);
                }
                else
                {
                    return (Bitmap)DataObjs;
                }
            }

            public string TryGetString()
            {
                if (this.IsBytes)
                {
                    return System.Text.Encoding.UTF8.GetString(this.DataBytes);
                }
                else
                {
                    return (string)DataObjs;
                }

            }

            public byte[] DataBytes { get; private set; }
            public Object DataObjs { get; private set; }
            public bool IsBytes { get; private set; }
        }
        


        private void BeginWork()
        {
            InitCmdList();
            InitFirstInput();

            // Handler command
            foreach (Options.DebugCommandType cmd in cmd_list_)
            {
                switch (cmd)
                {
                    case Options.DebugCommandType.Binarization:
                        {
                            Bitmap src = next_in_.TryGetBitmap();
                            Bitmap out_bm = this.ImageBinaryzation(
                                src, opts.DarkThreshold, opts.AdaptiveBlkSize, opts.AdaptiveOff);
                            next_in_ = new DataWrapper(out_bm);
                        }
                        break;

                    case Options.DebugCommandType.Vectorization:
                        {
                            Bitmap src = next_in_.TryGetBitmap();
                            string svg_str = this.ImageToSVG(src);
                            next_in_ = new DataWrapper(svg_str);
                        }
                        break;

                    case Options.DebugCommandType.PathPlanning:
                        {
                            string svg_str = next_in_.TryGetString(); ;
                            string path_str = this.PathPlanning(svg_str);
                            next_in_ = new DataWrapper(path_str);
                        }
                        break;

                    default:
                        throw new SystemException("Unknown debug command");
                }
            }

            OutputData();
        }

        private Bitmap ImageBinaryzation(Bitmap src_bitmap, int dark_thresh, 
                    int adaptive_blksize, double ada_thresh_off)
        {
            return img_util_.ImageBinaryzation(
                src_bitmap, dark_thresh, adaptive_blksize, ada_thresh_off);
        }

        private string ImageToSVG(Bitmap src_bitmap)
        {
            return img_util_.ImageToSVG(src_bitmap, setting_.kSvgOutSize);
        }

        private string PathPlanning(string src_svg)
        {
            SVGUtil.ZoomedSVGGraphic svg_gra = ParseAndAutoZoomSVG(src_svg);
            PathPlanner.PathUtil.PlanningPath planning_path = PathPlanner.PathUtil.PlanPath(
                svg_gra.curve_graphic, setting_.kFitMaxError, setting_.kArcLimit, setting_.kFillLineParam);

            return planning_path.ToYaml();
        }

        private void InitFirstInput()
        {
            if (opts.InputFile == null)
            {
                // Read from stdin
                using (Stream stdin = Console.OpenStandardInput())
                {
                    byte[] stdin_data = ReadFully(stdin);
                    next_in_ = new DataWrapper(stdin_data);
                }
            }
            else
            {
                byte[] file_data = System.IO.File.ReadAllBytes(opts.InputFile);
                next_in_ = new DataWrapper(file_data);
            }
        }

        private void OutputData()
        {
            if (opts.OutputFile == null)
            {
                using (Stream stdout = Console.OpenStandardOutput())
                {
                    next_in_.WriteToStream(stdout);
                }
            }
            else
            {
                Stream out_stream = System.IO.File.OpenWrite(opts.OutputFile);
                next_in_.WriteToStream(out_stream);
            }
        }

        private void InitCmdList()
        {
            switch (opts.Cmd)
            {
                case Options.CommandType.All:
                    cmd_list_.Add(Options.DebugCommandType.Binarization);
                    cmd_list_.Add(Options.DebugCommandType.Vectorization);
                    cmd_list_.Add(Options.DebugCommandType.PathPlanning);
                    break;

                case Options.CommandType.ImgProcess:
                    cmd_list_.Add(Options.DebugCommandType.Binarization);
                    cmd_list_.Add(Options.DebugCommandType.Vectorization);
                    break;

                case Options.CommandType.PathPlanning:
                    cmd_list_.Add(Options.DebugCommandType.PathPlanning);
                    break;

                case Options.CommandType.Debug:
                    cmd_list_.Add(opts.DebugCmd);
                    break;

                default:
                    throw new SystemException("Unknown Option");

            }
        }

        public static byte[] ReadFully(Stream stream)
        {
            byte[] buffer = new byte[32768];
            using (MemoryStream ms = new MemoryStream())
            {
                while (true)
                {
                    int read = stream.Read(buffer, 0, buffer.Length);
                    if (read <= 0)
                        return ms.ToArray();
                    ms.Write(buffer, 0, read);
                }
            }
        }
        private static string GetAbsThirdPartyDir(string dir)
        {
            string base_path = System.AppDomain.CurrentDomain.BaseDirectory;
            return System.IO.Path.Combine(base_path, dir);
        }

        private SVGUtil.ZoomedSVGGraphic ParseAndAutoZoomSVG(string svg_str)
        {

            float width = setting_.kPaperWidth * setting_.kFitScaleFactor;
            float height = setting_.kPaperHeight * setting_.kFitScaleFactor;
            float margin = setting_.kPaperMargin * setting_.kFitScaleFactor;

            var svg_info = SVGParser.ParseSVG(svg_str);
            return SVGUtil.ZoomSVG(svg_info, width, height, margin);
        }

        private ImageProcessor.ImageUtil img_util_;
        private DataWrapper next_in_;
        private AppSetting setting_;
        private Options opts;
        private List<Options.DebugCommandType> cmd_list_;
    }
}
