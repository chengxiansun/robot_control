namespace RobDrawer
{
    #region Using Directives
    using CommandLine;
    using CommandLine.Text;
    #endregion

    internal class Options
    {
        public enum CommandType
        {
            ImgProcess,
            PathPlanning,
            All,
            Debug
        }

        public enum DebugCommandType
        {
            Binarization,
            Vectorization,
            PathPlanning
        }

        [Option('i', "input",
            HelpText = "input file, omitted option means to standard input")]
        public string InputFile { get; set; }

        [Option('o', "output",
            HelpText = "output file, omitted option means to standard output")]
        public string OutputFile { get; set; }

        [Option('c', "command", Required = true, 
            HelpText = "Command: ImgProcess | PathPlanning | All | Debug")]
        public CommandType Cmd { get; set; }

        [Option("DarkThreshold", DefaultValue = 20,
            HelpText = "Threshold when doing the global binarization" )]
        public int DarkThreshold { get; set; }

        [Option("BlockSize", DefaultValue = 35,
            HelpText = "Block size when doing the local binarization" )]
        public int AdaptiveBlkSize { get; set; }

        [Option("AdaptiveOff", DefaultValue = 4.5f,
            HelpText = "Threshold offset when doing the local binarization" )]
        public float AdaptiveOff { get; set; }

        [Option("debug", 
            HelpText = "Debug command: Binarization | Vectorization | PathPlanning")]
        public DebugCommandType DebugCmd { get; set; }

        [HelpOption]
        public string GetUsage()
        {
            return HelpText.AutoBuild(this, current => HelpText.DefaultParsingErrorsHandler(this, current));
        }
    }
}
