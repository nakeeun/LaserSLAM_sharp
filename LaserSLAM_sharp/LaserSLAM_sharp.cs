using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading.Tasks;
using System.Text;
using YDLidarSharp;

namespace YDLidar_scan
{
    class YDLidar_scan
    {
        public static double[] t_pos = new double[3];

        static async Task Main(string[] args)
        {
            
            Console.WriteLine("Hello SLAM!");

            var ydl = new YDLidarSDK();
            string[] port_list = YDLidarSDK.GetLidarPorts();
            ydl.SerialPort = "COM3";
            Console.WriteLine(ydl.SerialPort);
            /*
             * 아래의 파라미터 설정은 YDLidar G2의 고유 파라미터로 설정(수정 불필요)
            */
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
            ydl.MaxAngle = 180.0F;
            ydl.MinAngle = -180.0F;
            ydl.MaxRange = 64.0F;
            ydl.MinRange = 0.01F;
            ////////////////////////////////////////


            /////////////////////////////////////////
            bool ret = ydl.Initialize();
            if (ret == true)
            {
                List<double[]> path = new List<double[]>();
                List<double[]> map_key_pose = new List<double[]>();

                //ret = ydl.TurnOff();
                ret = ydl.TurnOn();
                bool miniUpdated = false;
                uint first_scan = 0;
                bool first_send = true;
                LaserScan raw_scan;
                double[] pos = new double[3];
                double[,] map_points = new double[2, 2];
                double[] resolution = new double[2] {0.5, 0.05};
                double[] fast_res = new double[2] {0.25, 0.025};
                double[,] min_max_val_1 = new double[2, 2];
                double[,] min_max_val_2 = new double[2, 2];
                double[,] BWmap = new double[2, 2];
                double[,] BWmap2 = new double[2, 2];
                double pixelSize = 0.2;
                double[] pos_guess = new double[3];
                t_pos[0] = 0; t_pos[1] = 0; t_pos[2] = 0;
                pos[0] = 0; pos[1] = 0; pos[2] = 0;

                while (true)
                //for (int q =0; q< 10; q++) 
                {
                    Stopwatch sw = new Stopwatch();
                    sw.Start();
                    // Lidar scan
                    raw_scan = ydl.GetData();
                    int scan_size = raw_scan.LaserPoints.Count;
                    double[,] p2c_scan = new double[scan_size, 2];
                    p2c_scan = pol2cart(raw_scan, scan_size, p2c_scan);
                    // Lidar scan
                    // initialize
                    if (first_scan == 0)
                    {
                        map_points = new double[scan_size, 2];
                        map_points = transform(p2c_scan, pos, scan_size, map_points);
                        path.Add(pos);
                        map_key_pose.Add(pos);
                        miniUpdated = true;
                    }
                    // initialize
                    // local map & grid
                    if (miniUpdated == true) {
                        int map_size = map_points.GetLength(0);
                        double[,] localMap = new double[map_size, 2];
                        localMap = ExtractLocalMap(map_points, p2c_scan, pos, scan_size, localMap);

                        min_max_val_1 = min_max_func(localMap, map_size, min_max_val_1);
                        min_max_val_1[0, 0] = min_max_val_1[0, 0] - 3 * pixelSize;
                        min_max_val_1[0, 1] = min_max_val_1[0, 1] - 3 * pixelSize;
                        min_max_val_1[1, 0] = min_max_val_1[1, 0] + 3 * pixelSize;
                        min_max_val_1[1, 1] = min_max_val_1[1, 1] + 3 * pixelSize;
                        int[] sgrid1 = new int[2];  // small grid
                        sgrid1 = extract_sgrid(min_max_val_1, pixelSize, sgrid1);  
                        int[] idx = new int[map_size];
                        idx = cal_idx(localMap, min_max_val_1, sgrid1, pixelSize, map_size, idx);
                        bool[,] gridMap1 = new bool[sgrid1[0], sgrid1[1]];
                        for (int i = 0; i < map_size; i++)
                        {
                            gridMap1[idx[i] % sgrid1[0], idx[i] / sgrid1[0]] = true;
                        }
                        BWmap = new double[sgrid1[0], sgrid1[1]];
                        BWmap = distanceBW(gridMap1, BWmap);

                        min_max_val_2 = min_max_func(localMap, map_size, min_max_val_2);
                        min_max_val_2[0, 0] = min_max_val_2[0, 0] - 3 * (pixelSize / 2);
                        min_max_val_2[0, 1] = min_max_val_2[0, 1] - 3 * (pixelSize / 2);
                        min_max_val_2[1, 0] = min_max_val_2[1, 0] + 3 * (pixelSize / 2);
                        min_max_val_2[1, 1] = min_max_val_2[1, 1] + 3 * (pixelSize / 2);
                        int[] sgrid2 = new int[2];
                        sgrid2 = extract_sgrid(min_max_val_2, (pixelSize / 2), sgrid2);
                        int[] idx2 = new int[map_size];
                        idx2 = cal_idx(localMap, min_max_val_2, sgrid2, (pixelSize / 2), map_size, idx2);
                        bool[,] gridMap2 = new bool[sgrid2[0], sgrid2[1]];
                        for (int i = 0; i < map_size; i++)
                        {
                            gridMap2[idx2[i] % sgrid2[0], idx2[i] / sgrid2[0]] = true;
                        }
                        BWmap2 = new double[sgrid2[0], sgrid2[1]];
                        BWmap2 = distanceBW(gridMap2, BWmap2);
                    }
                    ///////////predict current pose
                    if (first_scan <= 1)
                    {
                        first_scan++;
                        pos_guess = pos;
                    }
                    else if (first_scan > 1)
                    {
                        double[] pg_dp = new double[3];
                        int path_size = path.Count-2;
                        pg_dp = diffPos(path[path_size], pos, pg_dp);
                        pos_guess[0] = pos[0] + pg_dp[0];
                        pos_guess[1] = pos[1] + pg_dp[1];
                        pos_guess[2] = pos[2] + pg_dp[2];
                    }
                    // local map & grid
                    if (miniUpdated == true)
                        pos = fastMatch(BWmap, p2c_scan, min_max_val_1, resolution, pixelSize, pos_guess, scan_size);
                    else
                        pos = fastMatch(BWmap2, p2c_scan, min_max_val_2, resolution, (pixelSize / 2), pos_guess, scan_size);
                    //refine pose using smaller pixel
                    double[] best_hits = new double[scan_size];
                    best_hits = fastMatch(BWmap2, p2c_scan, min_max_val_2, fast_res, pixelSize, pos, scan_size, best_hits);
                    pos = t_pos;

                    //fast matching
                    double[] dp = new double[3];
                    int map_key_size = map_key_pose.Count-1;
                    dp = diffPos(map_key_pose[map_key_size], pos, dp);
                    if (Math.Abs(dp[0]) > 0.1 || Math.Abs(dp[1]) > 0.1 || Math.Abs(dp[2]) > deg2rad(5))
                    {
                        miniUpdated = true;

                        dp = diffPos(map_key_pose[map_key_size], pos, dp);
                        if (Math.Abs(dp[0]) > 0.5 || Math.Abs(dp[1]) > 0.5 || Math.Abs(dp[2]) > Math.PI)
                        {
                            pos = map_key_pose[map_key_size];
                        }
                        double[,] addScan_w = new double[scan_size, 2];
                        addScan_w = transform(p2c_scan, pos, scan_size, addScan_w);
                        double[,] newPoints = new double[scan_size, 2];
                        bool check_newPoints = false;
                        int cnt = 0;
                        for (int i = 0; i < scan_size; i++)
                        {
                            if (best_hits[i] > 1.1)
                            {
                                newPoints[cnt, 0] = addScan_w[i, 0];
                                newPoints[cnt, 1] = addScan_w[i, 1];
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
                                temp_map_points[i, 0] = map_points[i, 0];
                                temp_map_points[i, 1] = map_points[i, 1];
                            }
                            for (int i = 0; i < cnt; i++)
                            {
                                if (newPoints[i, 0] != 0 && newPoints[i, 1] != 0)
                                {
                                    int addPindx = i + map_size;
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

                    int prev_cnt = path.Count - 1;
                    //if (pos[0] != path[prev_cnt][0] && pos[1] != path[prev_cnt][1] && pos[2] != path[prev_cnt][2])
                    //{
                    path.Add(pos);
                    //}
                    sw.Stop();
                    // Console.WriteLine(sw.Elapsed.Seconds.ToString() + "sec "+sw.ElapsedMilliseconds.ToString() + "ms");
                }
                Console.WriteLine("end");
            }
            ret = ydl.TurnOff();
        }

        public static double[] fastMatch(double[,] BWmap, double[,] p2c_scan, double[,] min_max_val,double[] resolution, double pixelSize,double[] pos, int scan_size, double[] best_hits)
        {
            double ipixel = 1 / (pixelSize/2);
            double minX = min_max_val[0, 0];
            double minY = min_max_val[0, 1];
            int nCols = BWmap.GetLength(0);
            int nRows = BWmap.GetLength(1);
            int max_Iter = 50;
            int maxDepth = 3;
            int depth = 0;
            double[] bestPos = pos;
            double best_score = 99999999;
            double[,] pixel_scan = new double[scan_size, 2];
            bool noChange = true;
            double r = resolution[0];
            double t = resolution[1];
            r = deg2rad(r);
            double[] theta = new double[3] { -r + pos[2], pos[2], r + pos[2] };
            double[] tx = new double[3] { -t + pos[0], pos[0], t + pos[0] };
            double[] ty = new double[3] { -t + pos[1], pos[1], t + pos[1] };

            for (int m = 0; m < scan_size; m++)
            {
                pixel_scan[m, 0] = p2c_scan[m, 0] * ipixel;
                pixel_scan[m, 1] = p2c_scan[m, 1] * ipixel;
            }
            for (int iter = 0; iter < max_Iter; iter++)
            {
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
                        int[] s_y = new int[scan_size];
                        double[] temp_best_hits = new double[scan_size];
                        for (int i = 0; i < scan_size; i++)
                        {
                            s_x[i] = (int)Math.Round(S_[i, 0] + (tx[j] - minX) * ipixel) + 1;
                        }
                        for (int k = 0; k < 3; k++) 
                        {
                            for (int i = 0; i < scan_size; i++)
                            {
                                s_y[i] = (int)Math.Round(S_[i, 1] + (ty[k] - minY) * ipixel) + 1;
                            }
                            double score = 0;
                            for (int i = 0; i < scan_size; i++)
                            {
                                if (s_x[i] > 1 && s_y[i] > 1 && s_x[i] < nCols && s_y[i] < nRows)
                                {
                                    int ix = s_x[i];
                                    int iy = s_y[i];
                                    int idx = iy + (ix - 1) * nRows;
                                    int r_x = idx % BWmap.GetLength(0); // 재 검토 필요
                                    int r_y = idx / BWmap.GetLength(0);
                                    score = score + BWmap[r_x, r_y];
                                    temp_best_hits[i] = BWmap[r_x, r_y];
                                }
                            }
                            if (score < best_score)
                            {
                                noChange = false;
                                bestPos[0] = tx[j]; bestPos[1] = ty[k]; bestPos[2] = theta[t_i];
                                best_score = score;
                                best_hits = temp_best_hits;
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
            t_pos = bestPos;
            return best_hits;
        }
        public static double[] fastMatch(double[,] BWmap, double[,] p2c_scan, double[,] min_max_val, double[] resolution, double pixelSize, double[] pos_guess, int scan_size)
        {
            double ipixel = 1 / pixelSize;
            double minX = min_max_val[0, 0];
            double minY = min_max_val[0, 1];
            int nCols = BWmap.GetLength(0);
            int nRows = BWmap.GetLength(1);
            int max_Iter = 50;
            int maxDepth = 3;
            int depth = 0;
            double[] bestPos = new double[3] { pos_guess[0], pos_guess[1], pos_guess[2] }; 
            double best_score = 99999999;
            double[,] pixel_scan = new double[scan_size, 2];
            bool noChange = true;
            double r = resolution[0];
            double t = resolution[1];
            r = deg2rad(r);
            double[] tx = new double[3] { -t + pos_guess[0], pos_guess[0], t + pos_guess[0] };
            double[] ty = new double[3] { -t + pos_guess[1], pos_guess[1], t + pos_guess[1] };
            double[] theta = new double[3] { -r + pos_guess[2], pos_guess[2], r + pos_guess[2] };
            for (int m = 0; m < scan_size; m++)
            {
                pixel_scan[m, 0] = p2c_scan[m, 0] * ipixel;
                pixel_scan[m, 1] = p2c_scan[m, 1] * ipixel;
            }
            for (int iter = 0; iter < max_Iter; iter++)
            {
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
                        int[] s_y = new int[scan_size];
                        for (int i = 0; i < scan_size; i++)
                        {
                            s_x[i] = (int)Math.Round(S_[i, 0] + (tx[j] - minX) * ipixel) + 1;
                        }
                        for (int k = 0; k < 3; k++)
                        {
                            for (int i = 0; i < scan_size; i++)
                            {
                                s_y[i] = (int)Math.Round(S_[i, 1] + (ty[k] - minY) * ipixel) + 1;
                            }
                            double score = 0;
                            for (int i = 0; i < scan_size; i++)
                            {
                                if (s_x[i] > 1 && s_y[i] > 1 && s_x[i] < nCols && s_y[i] < nRows)
                                {

                                    int ix = s_x[i];
                                    int iy = s_y[i];
                                    int idx = iy + (ix - 1) * nRows;
                                    int r_x = idx % BWmap.GetLength(0);
                                    int r_y = idx / BWmap.GetLength(0);
                                    score = score + BWmap[r_x, r_y];
                                }
                            }
                            if (score < best_score)
                            {
                                noChange = false;
                                bestPos[0] = tx[j]; bestPos[1] = ty[k]; bestPos[2] = theta[t_i];
                               // Console.WriteLine(j+", "+k+", "+t_i);
                                best_score = score;
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
            return bestPos;
        }
        public static double[,] distanceBW(bool[,] gridMap, double[,] BWmap) 
        {
            int p_x = gridMap.GetLength(0);
            int p_y = gridMap.GetLength(1);
            bool first = false;
            for (int i = 0; i < p_x; i++) 
            {
                for (int j = 0; j < p_y; j++)
                {
                    if (gridMap[i, j] == true)
                    {
                        BWmap[i, j] = 0.0;
                        for (int m = 0; m < p_x; m++)
                        {
                            for (int n = 0; n < p_y; n++)
                            {
                                double distance = Math.Sqrt(Math.Pow(m - i,2) + Math.Pow(n - j,2));
                                if (distance > 10) distance = 10;
                                if (first == false)
                                {
                                    BWmap[m, n] = distance;
                                }
                                else if (first == true)
                                {
                                    if (BWmap[m, n] > distance)
                                    {
                                        BWmap[m, n] = distance;
                                    }
                                }

                            }
                        }
                        if (first == false)
                            first = true;
                    }
                }
            }
            return BWmap;
        } 
        public static int[] extract_sgrid( double[,] min_max_val,double pixelSize, int[] sgrid)
        {
            sgrid[0] = (int)(Math.Round((min_max_val[1, 0] - min_max_val[0, 0]) / pixelSize) + 1);
            sgrid[1] = (int)(Math.Round((min_max_val[1, 1] - min_max_val[0, 1]) / pixelSize) + 1);
            return sgrid;
        }
        public static int[] cal_idx(double[,] localMap, double[,] min_max_val,int[] sgrid, double pixelSize, int scan_size, int[] idx) {
            double[,] reMap = new double[scan_size, 2];
            for (int i = 0; i < scan_size; i++)
            {
                reMap[i, 0] = min_max_val[0, 0];
                reMap[i, 1] = min_max_val[0, 1];
            }
            int[,] hits = new int[scan_size, 2];
            for (int i = 0; i < scan_size; i++)
            {
                hits[i, 0] = (int)(Math.Round((localMap[i, 0] - reMap[i, 0]) / pixelSize) + 1);
                hits[i, 1] = (int)(Math.Round((localMap[i, 1] - reMap[i, 1]) / pixelSize) + 1);
                idx[i] = (hits[i, 0] - 1) * sgrid[1] + hits[i, 1];
            }
            return idx;
        }
        public static double[,] ExtractLocalMap(double[,] map_points, double[,] scan, double[] pos, int scan_size, double[,] localMap) {
            double[,] scan_w = new double[scan_size, 2];
            double[,] min_max_val = new double[2, 2];
            scan_w = transform(scan, pos, scan_size, scan_w);
            min_max_val = min_max_func(scan_w, scan_size, min_max_val);
            double minX=min_max_val[0,0]-1, minY= min_max_val[0, 1]- 1, maxX= min_max_val[1, 0]+ 1, maxY= min_max_val[1, 1]+ 1;
            for (int i = 0; i < map_points.GetLength(0); i++)
            {
                if (map_points[i, 0] > minX && map_points[i, 0] < maxX && map_points[i, 1] > minY && map_points[i, 1] < maxY)
                { 
                    localMap[i, 0] = map_points[i, 0];
                    localMap[i, 1] = map_points[i, 1];
                }
            }
            return localMap;
        }
        public static double[,] transform(double[,] scan,double[] pos, int scan_size, double[,] trans_scan)
        {
            double t_x = pos[0];
            double t_y = pos[1];
            double t_theta = pos[2];
            double c_theta = Math.Cos(t_theta);
            double s_theta = Math.Sin(t_theta);
            double[,] R_ = new double[2, 2];
            R_[0, 0] = c_theta; R_[0, 1] = -s_theta; R_[1, 0] = s_theta; R_[1, 1] = c_theta;
            R_ = Dgetr(R_);
            string savePath = @"C:\Users\SAMSUNG\Desktop\trans.txt";
            for (int i = 0; i < scan_size; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    for (int k = 0; k < 2; k++)
                    {
                        trans_scan[i, j] += scan[i, k] * R_[k, j];
                    }
                }
                trans_scan[i, 0] += t_x;
                trans_scan[i, 1] += t_y;
            }
            return trans_scan;
        }
        public static double[,] pol2cart(LaserScan scan, int scan_size, double[,] p2c)
        {
            for (int i = 0; i < scan_size; i++)
            {
                double angle = (double)scan.LaserPoints[i].Angle;
                double range = (double)scan.LaserPoints[i].Range;
                p2c[i, 0] = range * Math.Cos(angle);
                p2c[i, 1] = range * Math.Sin(angle);
            }
            return p2c;
        }
        public static void write_data(string savePath, double point1, double point2) {
            string ptx = point1.ToString();
            string pty = point2.ToString();
            string output_ = "\r\n" + ptx + "," + pty;
            System.IO.File.AppendAllText(savePath, output_);
        }
        public static double[,] Dgetr(double[,] A)
        {
            int m = A.GetLength(0);
            int n = A.GetLength(1);
            double[,] tA = new double[n, m];
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < m; j++)
                {
                    tA[i, j] = A[j, i];
                }
            }
            return tA;
        }
        public static double deg2rad(double degree)
        {
            double radian = 0.0;
            radian = degree / Math.PI;
            return radian;
        }
        public static double[] diffPos(double[] prev_pos, double[] pos, double[] dp)
        {
            dp[0] = pos[0] - prev_pos[0];
            dp[1] = pos[1] - prev_pos[1];
            dp[2] = Math.Atan2(Math.Sin(pos[2] - prev_pos[2]), Math.Cos(pos[2] - prev_pos[2]));
            return dp;
        }
        public static double[,] min_max_func(double[,] points, int scan_size, double[,] min_max_value)
        {
            double minX = 99999999, minY = 99999999, maxX = -99999999, maxY = -99999999;
            for (int i = 0; i < scan_size; i++)
            {
                if (points[i, 0] < minX)
                    minX = points[i, 0];
                if (points[i, 1] < minY)
                    minY = points[i, 1];
                if (points[i, 0] > maxX)
                    maxX = points[i, 0];
                if (points[i, 1] > maxY)
                    maxY = points[i, 1];
            }
            min_max_value[0, 0] = minX; 
            min_max_value[0, 1] = minY; 
            min_max_value[1, 0] = maxX; 
            min_max_value[1, 1] = maxY;
            return min_max_value;
        }
    }
    class ChangeResponse
    {
        public int Code { get; set; }
        public string Message { get; set; }
    }
}
