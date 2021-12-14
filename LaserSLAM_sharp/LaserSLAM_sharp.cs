using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Threading;
using System.Text;
using YDLidarSharp;
using static SLAM_utils;

namespace YDLidar_scan
{
    class best_pos_hits
    {
        public double x;
        public double y;
        public double theta;
        public List<double> hits = new List<double>();
    }

    class YDLidar_scan
    {
        public static bool miniUpdated = false;        
        public static uint first_scan = 0;
        public static double[] pos;
        public static double[] pos_guess;
        public static double[,] map_points = new double[2, 2];  
        public static double[] resolution = new double[2] { 0.05, 0.005 }; // [0] rotation, [1] translation
        public static double[] fast_res = new double[2] { resolution[0] / 2, resolution[1] / 2 };
        public static double[] dp_threshold = new double[2] { 0.01, 2 };
        public static double[] large_dp_threshold = new double[2] { 0.05, 3.141592654 };
        public static double[,] min_max_val_1 = new double[2, 2];
        public static double[,] min_max_val_2 = new double[2, 2];
        public static double[,] BWmap = new double[2, 2];
        public static double[,] BWmap2 = new double[2, 2];
        public static double pixelSize = 0.1;
        public static double hit_threshold = 0.1;
        public static double imu_theta = 0.0;
        public static List<double[]> path;
        public static List<double[]> map_key_pose;
        
        static void Main(string[] args)
        {
            Console.WriteLine("Hello Lidar SLAM");

            var ydl = new YDLidarSDK();
            ydl = Lidar_par_set(ydl, "/dev/ttyUSB0");  //linux
            //ydl = Lidar_par_set(ydl, "COM3");        //Windows
            bool ret = ydl.Initialize();
            ret = ydl_r.Initialize();
            ret = ydl.TurnOn();
            public LaserScan raw_scan_points_f;
            if (ret == true)
            {
                while (true)
                {
                    raw_scan_points_f = ydl.GetData();
                    SLAM(raw_scan_points);         
                }
                ret = ydl.TurnOff();
                Console.WriteLine("end");
            }
        }
        public static void SLAM(object obj_f)
        {
            LaserScan raw_scan_points_f = (LaserScan)obj_f;
            int scan_size = raw_scan_points_f.LaserPoints.Count;
            double[,] p2c_points = new double[scan_size, 2];
            p2c_points = SLAM_utils.pol2cart(raw_scan_points_f, p2c_points);
            
            if (first_scan == 0)  // Initialize
            {
                pos = new double[3] { 0, 0, 0 };
                pos_guess = new double[3] { 0, 0, 0 };
                map_points = new double[scan_size, 2];
                path = new List<double[]>();
                map_key_pose = new List<double[]>();
                map_points = SLAM_utils.transform(p2c_points, pos, scan_size);
                path.Add(pos);
                map_key_pose.Add(pos);
                miniUpdated = true;
            }
            // initialize

            ///////// local map & grid
            if (miniUpdated == true)
            {
                double[,] localMap = SLAM_utils.ExtractLocalMap(map_points, p2c_points, pos);
                int map_size = localMap.GetLength(0);
                double[,] min_max_val = SLAM_utils.min_max_func(localMap, map_size);
                min_max_val_1[0, 0] = min_max_val[0, 0] - 3 * pixelSize;
                min_max_val_1[0, 1] = min_max_val[0, 1] - 3 * pixelSize;
                min_max_val_1[1, 0] = min_max_val[1, 0] + 3 * pixelSize;
                min_max_val_1[1, 1] = min_max_val[1, 1] + 3 * pixelSize;
                BWmap = SLAM_utils.create_BWmap(localMap, min_max_val_1, pixelSize);

                min_max_val_2[0, 0] = min_max_val[0, 0] - 3 * (pixelSize / 2);
                min_max_val_2[0, 1] = min_max_val[0, 1] - 3 * (pixelSize / 2);
                min_max_val_2[1, 0] = min_max_val[1, 0] + 3 * (pixelSize / 2);
                min_max_val_2[1, 1] = min_max_val[1, 1] + 3 * (pixelSize / 2);
                BWmap2 = SLAM_utils.create_BWmap(localMap, min_max_val_2, (pixelSize / 2));
            }
            /////////local map & grid ///////////

            ///////////predict current pose
            if (first_scan <= 1)
            {
                first_scan++;
                pos_guess = pos;
            }
            else if (first_scan > 1)
            {
                int path_size = path.Count - 2;
                double[] pg_dp = SLAM_utils.diffPos(path[path_size], pos);
                pos_guess[0] = pos[0] + pg_dp[0];
                pos_guess[1] = pos[1] + pg_dp[1];
                pos_guess[2] = pos[2] + pg_dp[2];
            }

            ////////// fast matching
            best_pos_hits pos_hits = new best_pos_hits() { };

            if (miniUpdated == true)
                pos_hits = fastMatch(BWmap, p2c_points, pos_guess, min_max_val_1, resolution, pixelSize, scan_size, pos_hits);
            else
                pos_hits = fastMatch(BWmap2, p2c_points, pos_guess, min_max_val_2, resolution, (pixelSize / 2), scan_size,pos_hits);

            double[] temp_pos = new double[3] { pos_hits.x, pos_hits.y, pos_hits.theta };
            pos_hits = fastMatch(BWmap2, p2c_points, temp_pos, min_max_val_2, fast_res, (pixelSize / 2), scan_size, pos_hits);
            pos[0] = pos_hits.x; pos[1] = pos_hits.y; pos[2] = pos_hits.theta;
            pos[0] = Math.Round(pos[0], 3);
            pos[1] = Math.Round(pos[1], 3);
            pos[2] = Math.Round(pos[2], 3);
            

                int map_key_size = map_key_pose.Count - 1;
                double[] dp = SLAM_utils.diffPos(map_key_pose[map_key_size], pos);
                //Console.WriteLine(dp[0]+", "+dp[1]+", "+dp[2]);
                if (Math.Abs(dp[0]) > dp_threshold[0] || Math.Abs(dp[1]) > dp_threshold[0] || Math.Abs(dp[2]) > SLAM_utils.deg2rad(dp_threshold[1]))
                {
                    miniUpdated = true;
                    double[] dp_l = SLAM_utils.diffPos(map_key_pose[map_key_size], pos);
                    if (Math.Abs(dp_l[0]) > large_dp_threshold[0] || Math.Abs(dp_l[1]) > large_dp_threshold[0] || Math.Abs(dp_l[2]) > large_dp_threshold[1])
                    {
                        Console.WriteLine("Large Error");
                        pos = map_key_pose[map_key_size];
                        Console.WriteLine("last POS : " + map_key_pose[map_key_size][0] + ", " + map_key_pose[map_key_size][1] + ", " + map_key_pose[map_key_size][2]);
                    }
                    double[,] addScan_w = SLAM_utils.transform(p2c_points, pos, scan_size);

                    double[,] newPoints = new double[scan_size, 2];
                    bool check_newPoints = false;
                    int cnt = 0;

                    for (int i = 0; i < scan_size; i++)
                    {
                        if (pos_hits.hits[i] > hit_threshold)
                        {
                            newPoints[cnt, 0] = Math.Round(addScan_w[i, 0], 5);
                            newPoints[cnt, 1] = Math.Round(addScan_w[i, 1], 5);
                            cnt++;
                            if (check_newPoints != true)
                            {
                                check_newPoints = true;
                            }
                        }
                    }
                    if (check_newPoints == true)
                    {
                        int map_size = map_points.GetLength(0);
                        int new_map_points_size = cnt + map_size;
                        double[,] temp_map_points = new double[new_map_points_size, 2];
                        
                        for (int i = 0; i < map_size; i++)
                        {
                            if (map_points[i, 0] != pos[0] && map_points[i, 1] != pos[1])
                            {
                                temp_map_points[i, 0] = map_points[i, 0];
                                temp_map_points[i, 1] = map_points[i, 1];
                            }
                        }
                        for (int i = 0; i < cnt; i++)
                        {
                            if (newPoints[i, 0] != pos[0] && newPoints[i, 1] != pos[1])
                            {
                                int addPindx = i + (map_size - 1);
                                temp_map_points[addPindx, 0] = newPoints[i, 0];
                                temp_map_points[addPindx, 1] = newPoints[i, 1];
                            }
                        }
                        
                        map_key_pose.Add(pos);
                        map_points = new double[new_map_points_size, 2];
                        map_points = temp_map_points;
                    }
                    else
                    {

                    }
                }
                else
                {
                    miniUpdated = false;
                }
            
            path.Add(pos);
        }
        public static best_pos_hits fastMatch(double[,] BWmap, double[,] p2c_points, double[] pos, double[,] min_max_val, double[] resolution, double pixelSize, int scan_size, best_pos_hits pos_hits)
        {
            double ipixel = 1 / pixelSize;
            double minX = min_max_val[0, 0];
            double minY = min_max_val[0, 1];
            double r = resolution[0];        // rotation
            double t = resolution[1];        // translation
            double best_score = 99999999;
            double[] theta = new double[3];
            double[] tx = new double[3];
            double[] ty = new double[3];
            double[,] pixel_scan = new double[scan_size, 2];
            int nCols = BWmap.GetLength(1);
            int nRows = BWmap.GetLength(0);
            int max_Iter = 50;
            int maxDepth = 3;
            int depth = 0;
            bool noChange = true;
            r = SLAM_utils.deg2rad(r);
            pos_hits.x = pos[0]; pos_hits.y = pos[1]; pos_hits.theta = pos[2];

            for (int m = 0; m < scan_size; m++)
            {
                pixel_scan[m, 0] = p2c_points[m, 0] * ipixel;
                pixel_scan[m, 1] = p2c_points[m, 1] * ipixel;
            }
            for (int iter = 0; iter < max_Iter; iter++)
            {
                tx[0] = pos_hits.x - t; tx[1] = pos_hits.x; tx[2] = pos_hits.x + t;
                ty[0] = pos_hits.y - t; ty[1] = pos_hits.y; ty[2] = pos_hits.y + t;
                theta[0] = pos_hits.theta - r; theta[1] = pos_hits.theta; theta[2] = pos_hits.theta + r;
                noChange = true;
                for (int t_i = 0; t_i < 3; t_i++)
                {
                    double c_theta = Math.Cos(theta[t_i]);
                    double s_theta = Math.Sin(theta[t_i]);
                    double[,] S_ = new double[scan_size, 2];
                    double[,] trans_theta = new double[2, 2];
                    trans_theta[0, 0] = c_theta; trans_theta[0, 1] = s_theta; trans_theta[1, 0] = -s_theta; trans_theta[1, 1] = c_theta;
                    for (int i = 0; i < scan_size; i++)
                    {
                        for (int j = 0; j < 2; j++)
                        {
                            for (int k = 0; k < 2; k++)
                            {
                                S_[i, j] += pixel_scan[i, k] * trans_theta[k, j];
                            }
                        }
                    }
                    for (int j = 0; j < 3; j++)
                    {
                        int[] s_x = new int[scan_size];
                        double[] temp_best_hits = new double[scan_size];
                        for (int i = 0; i < scan_size; i++)
                        {
                            s_x[i] = (int)Math.Round(S_[i, 0] + (tx[j] - minX) * ipixel) + 1;
                        }
                        for (int k = 0; k < 3; k++)
                        {
                            int[] s_y = new int[scan_size];
                            double score = 0;
                            for (int i = 0; i < scan_size; i++)
                            {
                                s_y[i] = (int)Math.Round(S_[i, 1] + (ty[k] - minY) * ipixel) + 1;
                                if (s_x[i] > 1 && s_y[i] > 1 && s_x[i] < nCols && s_y[i] < nRows)
                                {
                                    int ix = s_x[i];
                                    int iy = s_y[i];
                                    int idx = iy + (ix - 1) * nRows;
                                    int r_x = idx % nRows;
                                    double t_r_y = idx / nRows;
                                    int r_y = (int)Math.Truncate(t_r_y);
                                    score = score + BWmap[r_x, r_y];
                                    temp_best_hits[i] = BWmap[r_x, r_y];
                                }
                            }
                            if (score < best_score)
                            {
                                noChange = false;
                                pos_hits.x = tx[j]; pos_hits.y = ty[k]; pos_hits.theta = theta[t_i];
                                best_score = score;
                                for (int i = 0; i < scan_size; i++)
                                {
                                    pos_hits.hits.Add(temp_best_hits[i]);
                                }
                            }
                        }
                    }
                }
                if (noChange == true)
                {
                    r /= 2;
                    t /= 2;
                    depth++;
                    if (depth > maxDepth)
                        break;
                }
            }
            return pos_hits;
        }
        

        public static void write_data(string savePath, double point1, double point2)
        {
            string ptx = point1.ToString();
            string pty = point2.ToString();
            string output_ = "\r\n" + ptx + "," + pty;
            System.IO.File.AppendAllText(savePath, output_);
        }

        public static YDLidarSDK Lidar_par_set(YDLidarSDK ydl, string com)
        {
            //string[] port_list = YDLidarSDK.GetLidarPorts();
            Console.WriteLine(com);
            ydl.SerialPort = com;
            ydl.SerialBaudrate = 230400;
            ydl.SingleChannel = false;
            ydl.ScanFrequency = 10.0F;
            ydl.LidarType = LidarType.TRIANGLE;
            ydl.SampleRate = 5;
            ydl.AbnormalCheckCount = 2;
            ydl.Reversion = true;
            ydl.Inverted = true;
            ydl.AutoReconnect = true;
            ydl.FixedResolution = true;
            ydl.MaxAngle = 150.0F;
            ydl.MinAngle = -30.0F;
            ydl.MaxRange = 16.0F;
            ydl.MinRange = 0.1F;
            return ydl;
        }
    }
}
