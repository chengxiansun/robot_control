using System;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.IO;

namespace RobDrawer.ImageProcessor
{
    public class ImageUtil
    {
        public struct InitParam
        {
            public string thirdparty_abs_dir;
        }
        public struct ImageZoomInfo
        {
            public float scale;     // The scale from source image to generated svg path 
            public PointF trans;    // Transform of scaled image
        }


        public ImageUtil(InitParam init_param)
        {
            this.init_param = init_param;
        }

        public static ImageZoomInfo CalcZoomInfo(SizeF src_size, SizeF dest_size)
        {
            // Calc transform info
            float scale_x = dest_size.Width / src_size.Width;
            float scale_y = dest_size.Height / src_size.Height;
            float scale_f = Math.Min(scale_x, scale_y);

            float dest_w = src_size.Width * scale_f;
            float dest_h = src_size.Height * scale_f;
            float trans_x = (dest_size.Width - dest_w) / 2;
            float trans_y = (dest_size.Height - dest_h) / 2;

            ImageZoomInfo ret = new ImageZoomInfo();
            ret.scale = scale_f;
            ret.trans.X = trans_x;
            ret.trans.Y = trans_y;

            return ret;
        }
        public static RectangleF[] ScaleRect(SizeF src_ref, SizeF dst_ref, params RectangleF[] rects)
        {
            double scale_x = (double)dst_ref.Width / (double)src_ref.Width;
            double scale_y = (double)dst_ref.Height / (double)src_ref.Height;

            var ret = new RectangleF[rects.Length];
            for (int i = 0; i < rects.Length; i++)
            {
                ret[i].X = (float)(rects[i].X * scale_x);
                ret[i].Width = (float)(rects[i].Width * scale_x);

                ret[i].Y = (float)(rects[i].Y * scale_y);
                ret[i].Height = (float)(rects[i].Height * scale_y);
            }

            return ret;
        }


        public Bitmap ImageBinaryzation(Bitmap src_bitmap, int dark_thresh, 
                    int adaptive_blksize, double ada_thresh_off = 5.0)
        {
            string image_util_path = System.IO.Path.Combine(
                init_param.thirdparty_abs_dir, "ImageUtil");
            System.Diagnostics.Process process = new System.Diagnostics.Process();
            process.StartInfo.UseShellExecute = false;
            process.StartInfo.FileName = image_util_path;
            process.StartInfo.Arguments = String.Format(
                "--Command Binarization --DarkThreshold {0} --AdaptiveBlkSize {1} --AdaptiveOffset {2}",
                dark_thresh, adaptive_blksize, ada_thresh_off);
            //process.StartInfo.WindowStyle = System.Diagnostics.ProcessWindowStyle.Hidden;
            process.StartInfo.RedirectStandardInput = true;
            process.StartInfo.RedirectStandardOutput = true;

            // Start process
            process.Start();

            // Write bitmap to process's stdin
            // The ImageUtil could not decode BMP image now.
            src_bitmap.Save(process.StandardInput.BaseStream, System.Drawing.Imaging.ImageFormat.Jpeg);
            process.StandardInput.Close();

            byte[] bmp_out_bytes = ReadFully(process.StandardOutput.BaseStream);
            process.Close();

            // Get output and close 
            Bitmap bmp_ret;
            using (var ms = new MemoryStream(bmp_out_bytes))
            {
                bmp_ret = new Bitmap(ms);
            }

            // new a Bitmap to fix Save problem
            return new Bitmap(bmp_ret);
        }

        public string ImageToSVG(Bitmap src_bitmap, System.Drawing.Size target_size)
        {
            // Calc actual target size
            var src_size = src_bitmap.Size;
            float scale_x = (float)src_size.Width / (float)target_size.Width;
            float scale_y = (float)src_size.Height / (float)target_size.Height;
            float scale_factor = Math.Max(scale_x, scale_y);

            int act_w = (int)(src_size.Width / scale_factor);
            int act_h = (int)(src_size.Height / scale_factor);

            // Scale bitmap
            Bitmap small_bitmap = ResizeImage(src_bitmap, act_w, act_h);

            // Call potrace to convert image to SVG
            // Init Process info
            // No ".exe" postfix to make it compatible in both windows and linux
            string potrace_path = System.IO.Path.Combine(
                init_param.thirdparty_abs_dir, "potrace", "potrace");
            System.Diagnostics.Process process = new System.Diagnostics.Process();
            process.StartInfo.UseShellExecute = false;
            process.StartInfo.FileName = potrace_path;
            process.StartInfo.Arguments = "-t 10 -s";
            //process.StartInfo.WindowStyle = System.Diagnostics.ProcessWindowStyle.Hidden;
            process.StartInfo.RedirectStandardInput = true;
            process.StartInfo.RedirectStandardOutput = true;

            // Start process
            process.Start();
            // Write bitmap to process's stdin
            small_bitmap.Save(process.StandardInput.BaseStream, System.Drawing.Imaging.ImageFormat.Bmp);
            process.StandardInput.Close();

            // Get output
            //process.WaitForExit();
            string ret = process.StandardOutput.ReadToEnd();
            process.Close();

            return ret;
        }


        public Rectangle[] FaceDetect(Bitmap src_bitmap, double min_size_factor)
        {
            string image_util_path = System.IO.Path.Combine(
                init_param.thirdparty_abs_dir, "ImageUtil");
            System.Diagnostics.Process process = new System.Diagnostics.Process();
            process.StartInfo.UseShellExecute = false;
            process.StartInfo.FileName = image_util_path;
            process.StartInfo.Arguments = String.Format(
                "--Command FaceDetect --MinSizeFactor {0}",
                min_size_factor);
            //process.StartInfo.WindowStyle = System.Diagnostics.ProcessWindowStyle.Hidden;
            process.StartInfo.RedirectStandardInput = true;
            process.StartInfo.RedirectStandardOutput = true;
            process.StartInfo.RedirectStandardError = true;
            process.StartInfo.WorkingDirectory = System.IO.Path.GetDirectoryName(image_util_path);

            // Start process
            process.Start();

            // Write bitmap to process's stdin
            // The ImageUtil could not decode BMP image now.
            src_bitmap.Save(process.StandardInput.BaseStream, System.Drawing.Imaging.ImageFormat.Jpeg);
            process.StandardInput.Close();

            string result_str = process.StandardOutput.ReadToEnd();
            string err_str = process.StandardError.ReadToEnd();
            if (err_str.Length != 0)
            {
                Trace.TraceError("Error message when calling ImageUtil: ");
                Trace.TraceError(err_str);
            }
            process.Close();
            

            // Parse rectangle in str
            char[] spliter = new char[] { ' ', '\r', '\n', '\t' };
            string[] tokens = result_str.Split( spliter, StringSplitOptions.RemoveEmptyEntries);
            if (tokens.Length == 0)
            {
                return new Rectangle[0];
            }

            try
            {
                int rect_count = int.Parse(tokens[0]);
                if (tokens.Length != rect_count * 4 + 1)
                {
                    throw new System.SystemException("ImageUtil Output format error");
                }

                Rectangle[] ret = new Rectangle[rect_count];
                int num_i = 1;
                for (int i = 0; i < rect_count; i++)
                {
                    ret[i].X = int.Parse(tokens[num_i++]);
                    ret[i].Y = int.Parse(tokens[num_i++]);
                    ret[i].Width = int.Parse(tokens[num_i++]);
                    ret[i].Height = int.Parse(tokens[num_i++]);
                }

                return ret;
            }
            catch (Exception e)
            {
                Trace.TraceError(e.ToString());
                throw new System.SystemException("Could not parse output of ImageUtil");
            }
        }

        public void CycleConvert(Bitmap src_bitmap, int model_type)
        {
            string image_util_path = System.IO.Path.Combine(
                init_param.thirdparty_abs_dir, "ImageUtil");
            System.Diagnostics.Process process = new System.Diagnostics.Process();
            process.StartInfo.UseShellExecute = false;
            process.StartInfo.FileName = image_util_path;
            process.StartInfo.Arguments = String.Format(
                "--Command CycleConvert --CycleModel {0}",
                model_type);
            //process.StartInfo.WindowStyle = System.Diagnostics.ProcessWindowStyle.Hidden;
            process.StartInfo.RedirectStandardInput = true;
            process.StartInfo.RedirectStandardOutput = true;

            // Start process
            process.Start();

            // Write bitmap to process's stdin
            // The ImageUtil could not decode BMP image now.
            src_bitmap.Save(process.StandardInput.BaseStream, System.Drawing.Imaging.ImageFormat.Jpeg);
            process.StandardInput.Close();

            byte[] bmp_out_bytes = ReadFully(process.StandardOutput.BaseStream);

            process.Close();

            // Get output and close 
            //Bitmap bmp_ret;
           // using (var ms = new MemoryStream(bmp_out_bytes))
            //{
              //  bmp_ret = new Bitmap(ms);
            //}

            // new a Bitmap to fix Save problem
            return ;
          
        }
        #region Private

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

        /// <summary>
        /// Resize the image to the specified width and height.
        /// Ref: http://stackoverflow.com/questions/1922040/resize-an-image-c-sharp
        /// </summary>
        /// <param name="image">The image to resize.</param>
        /// <param name="width">The width to resize to.</param>
        /// <param name="height">The height to resize to.</param>
        /// <returns>The resized image.</returns>
        private static Bitmap ResizeImage(Image image, int width, int height)
        {
            var destRect = new Rectangle(0, 0, width, height);
            var destImage = new Bitmap(width, height);

            // Changing from Ref. 
            // These code will not work if the source image does not have DPI information.
            // And it's not meaningful.
            //destImage.SetResolution(image.HorizontalResolution, image.VerticalResolution);

            using (var graphics = Graphics.FromImage(destImage))
            {
                graphics.CompositingMode = CompositingMode.SourceCopy;
                graphics.CompositingQuality = CompositingQuality.HighQuality;
                graphics.InterpolationMode = InterpolationMode.HighQualityBicubic;
                graphics.SmoothingMode = SmoothingMode.HighQuality;
                graphics.PixelOffsetMode = PixelOffsetMode.HighQuality;

                using (var wrapMode = new System.Drawing.Imaging.ImageAttributes())
                {
                    wrapMode.SetWrapMode(WrapMode.TileFlipXY);
                    graphics.DrawImage(image, destRect, 0, 0, image.Width, image.Height, GraphicsUnit.Pixel, wrapMode);
                }
            }

            return destImage;
        }

        private InitParam init_param;

        #endregion

    }
}
