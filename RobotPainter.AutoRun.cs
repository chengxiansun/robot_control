using System;
using System.IO;
using ImageUtil; // 原图片工具类
using RobDrawer.ImageProcessor; // 原图片处理模块
using RobDrawer.PathPlanner; // 原路径规划模块
using RobDrawer; // 原机器人驱动模块

namespace RobotPainter.AutoRun
{
    class Program
    {
        // 配置参数（可改为读取配置文件，如app.config）
        private const string IMAGE_FOLDER_PATH = @"D:\RobotPainter\InputImages"; // 指定图片文件夹
        private const string ROBOT_IP = "192.168.1.100"; // 机器人控制器IP
        private const int ROBOT_PORT = 8080; // 机器人通信端口
        private const string IMAGE_EXTENSION = "*.bmp"; // 只读取黑白BMP（可改为png/jpg）

        static void Main(string[] args)
        {
            try
            {
                Console.WriteLine("===== 机器人自动绘画程序 =====");
                // 1. 读取指定文件夹下的黑白图片
                var imageFiles = Directory.GetFiles(IMAGE_FOLDER_PATH, IMAGE_EXTENSION);
                if (imageFiles.Length == 0)
                {
                    Console.WriteLine($"错误：在{IMAGE_FOLDER_PATH}未找到{IMAGE_EXTENSION}格式的图片！");
                    Console.ReadLine();
                    return;
                }
                // 取第一个图片文件（可扩展为遍历所有图片）
                string targetImagePath = imageFiles[0];
                Console.WriteLine($"读取图片：{targetImagePath}");

                // 2. 黑白图片预处理（复用原模块）
                Console.WriteLine("开始处理图片...");
                var imageProcessor = new ImageProcessorCore(); // 原图片处理核心类（需与原项目类名一致）
                // 黑白图二值化/轮廓提取（原项目已实现的逻辑）
                var contourPoints = imageProcessor.ExtractContourFromBwImage(targetImagePath);
                if (contourPoints == null || contourPoints.Count == 0)
                {
                    Console.WriteLine("错误：图片轮廓提取失败！");
                    Console.ReadLine();
                    return;
                }
                Console.WriteLine($"图片处理完成，提取轮廓点数量：{contourPoints.Count}");

                // 3. 路径规划（复用原模块）
                Console.WriteLine("开始生成机器人运动路径...");
                var pathPlanner = new PathPlannerCore(); // 原路径规划核心类
                var robotConfig = new RobotConfig() // 机器人配置（原项目中提取的参数）
                {
                    MaxSpeed = 50, // 最大运动速度(mm/s)
                    ToolCoordinate = "Tool1", // 工具坐标系
                    WorkObject = "Wobj0" // 工件坐标系
                };
                var robotPaths = pathPlanner.GenerateSmoothPath(contourPoints, robotConfig);
                Console.WriteLine($"路径规划完成，生成运动段数量：{robotPaths.Count}");

                // 4. 连接机器人并下发绘画指令（复用原模块）
                Console.WriteLine($"连接机器人：{ROBOT_IP}:{ROBOT_PORT}...");
                var robotClient = new ABBRobotClient(ROBOT_IP, ROBOT_PORT); // 原机器人通信类
                if (!robotClient.Connect())
                {
                    Console.WriteLine("错误：机器人连接失败！");
                    Console.ReadLine();
                    return;
                }
                Console.WriteLine("机器人连接成功，开始绘画...");

                // 下发路径指令并执行
                robotClient.ExecutePaths(robotPaths);

                // 5. 绘画完成，断开连接
                Console.WriteLine("绘画完成！");
                robotClient.Disconnect();
            }
            catch (Exception ex)
            {
                Console.WriteLine($"程序异常：{ex.Message}\n{ex.StackTrace}");
            }
            finally
            {
                Console.WriteLine("按任意键退出...");
                Console.ReadLine();
            }
        }
    }
}