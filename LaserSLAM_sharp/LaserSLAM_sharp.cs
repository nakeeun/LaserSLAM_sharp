using System;
using System.Collections.Generic;
using YDLidarSharp;

namespace LaserSLAM_sharp
{
    class LaserSLAM
    {
        public static double[] t_pos = new double[3];
        static void Main(string[] args)
        {
            var ydl = new YDLidarSDK();
            ydl = Lidar_par_set(ydl);
            bool ret = ydl.Initialize();
            if (ret == true)
            {
                ret = ydl.TurnOn();
                LaserScan raw_scan;
                List<double[]> path = new List<double[]>();
                bool miniUpdated = false;
                int first_scan = 0;
                double[] prev_pos = new double[3];
                double[,] map_points = new double[2, 2];  
                double[] resolution = new double[2] { 0.5, 0.05 };
                double[] fast_res = new double[2] { 0.25, 0.025 };
                double[,] min_max_val = new double[2, 2];
                double[,] BWmap = new double[2, 2];
                double[,] BWmap2 = new double[2, 2];
                double pixelSize = 0.2;
                double[] pos_guess = new double[3];
                double[] pos = new double[3];
                pos[0] = 0; pos[1] = 0; pos[2] = 0;
                t_pos[0] = 0; t_pos[1] = 0; t_pos[2] = 0;
                while (true)
                {
                    // Lidar scan
                    raw_scan = ydl.GetData();
                    int scan_size = raw_scan.LaserPoints.Count;
                    double[,] p2c_scan = new double[scan_size, 2];
                    p2c_scan = pol2cart(raw_scan, scan_size, p2c_scan);
                    // Lidar scan/////////////////////////////////////
                    // initialize///////////
                    if (first_scan == 0)
                    {
                        map_points = new double[scan_size, 2];
                        map_points = transform(p2c_scan, pos, scan_size, map_points);
                        path.Add(pos);
                    }
                    // initialize////////////
                    // local map & grid//////
                    if (miniUpdated == true)
                    {
                        double[,] localMap = new double[scan_size, 2];
                        localMap = ExtractLocalMap(map_points, p2c_scan, pos, scan_size, localMap);

                        int[] sgrid = new int[2];
                        min_max_val = min_max_func(localMap, scan_size, min_max_val);
                        sgrid = extract_sgrid(min_max_val, pixelSize, sgrid);
                        int[] idx = new int[scan_size];
                        idx = cal_idx(localMap, min_max_val, sgrid, pixelSize, scan_size, idx);
                        bool[,] gridMap = new bool[sgrid[1], sgrid[0]];
                        for (int i = 0; i < scan_size; i++)
                        {
                            int j = 0, k = 0;
                            j = idx[i] % sgrid[1];
                            k = idx[i] / sgrid[1];
                            gridMap[j, k] = true;
                        }
                        BWmap = new double[sgrid[1], sgrid[0]];
                        BWmap = distanceBW(gridMap, BWmap);

                        sgrid = new int[2];
                        min_max_val = min_max_func(localMap, scan_size, min_max_val);
                        sgrid = extract_sgrid(min_max_val, pixelSize / 2, sgrid);
                        idx = new int[scan_size];
                        idx = cal_idx(localMap, min_max_val, sgrid, pixelSize / 2, scan_size, idx);
                        gridMap = new bool[sgrid[1], sgrid[0]];
                        for (int i = 0; i < scan_size; i++)
                        {
                            int j = 0, k = 0;
                            j = idx[i] % sgrid[1];
                            k = idx[i] / sgrid[1];
                            gridMap[j, k] = true;
                        }
                        BWmap2 = new double[sgrid[1], sgrid[0]];
                        BWmap2 = distanceBW(gridMap, BWmap2);
                    }
                    //Predict current pose using constant velocity motion model
                    if (first_scan <= 1)
                    {
                        first_scan++;
                        pos_guess = pos;
                    }
                    else if (first_scan > 1)
                    {
                        double[] pg_dp = new double[3];
                        pg_dp = diffPos(prev_pos, pos, pg_dp);
                        pos_guess[0] = pos[0] + pg_dp[0];
                        pos_guess[1] = pos[1] + pg_dp[1];
                        pos_guess[2] = pos[2] + pg_dp[2];
                    }
                    //Predict current pose using constant velocity motion model
                    // local map & grid/////////////////////////
                    //fast matching///////////
                    if (miniUpdated == true)
                        pos = fastMatch(BWmap, p2c_scan, min_max_val, resolution, pixelSize, pos_guess, scan_size);
                    else
                        pos = fastMatch(BWmap2, p2c_scan, min_max_val, resolution, pixelSize, pos_guess, scan_size);
                    //fast matching//////////////////////////////////////////////////////////////////////////////////
                    //refine pose using smaller pixel
                    double[] best_hits = new double[scan_size];
                    best_hits = fastMatch(BWmap2, p2c_scan, min_max_val, fast_res, pixelSize, pos, scan_size, best_hits);
                    pos = t_pos;
                    //////////////////////////
                    //Execute a mini update, if the robot has moved a certain distance
                    double[] dp = new double[3];
                    dp = diffPos(prev_pos, pos, dp);
                    if (Math.Abs(dp[0]) > 0.1 || Math.Abs(dp[1]) > 0.1 || Math.Abs(dp[2]) > deg2rad(5))
                    {
                        miniUpdated = true;
                        Console.WriteLine("updated");
                        dp = diffPos(prev_pos, pos, dp);
                        if (Math.Abs(dp[0]) > 0.5 || Math.Abs(dp[1]) > 0.5 || Math.Abs(dp[2]) > Math.PI)
                        {
                            Console.WriteLine("Large Error");
                            pos = prev_pos;
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
                                    check_newPoints = true;
                            }
                        }
                        if (check_newPoints == true)
                        {
                            Console.WriteLine("new points");
                            int map_size = map_points.GetLength(0);
                            int new_map_points_size = cnt + map_size;
                            double[,] temp_map_points = new double[new_map_points_size, 2];
                            map_points = new double[new_map_points_size, 2];
                            
                            for (int i = 0; i < map_size; i++)
                            {
                                temp_map_points[i, 0] = map_points[i, 0];
                                temp_map_points[i, 1] = map_points[i, 1];
                                
                            }
                            for (int i = 0; i < cnt; i++)
                            {
                                int addPindx = i + map_size;
                                temp_map_points[addPindx, 0] = newPoints[i, 0];
                                temp_map_points[addPindx, 1] = newPoints[i, 1];
                                
                            }
                            prev_pos = pos;
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
                ret = ydl.TurnOff();
            }
        }
        public static YDLidarSDK Lidar_par_set(YDLidarSDK ydl) 
        {
            string[] port_list = YDLidarSDK.GetLidarPorts();
            Console.WriteLine(port_list[0]);
            ydl.SerialPort = "COM3";
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
            return ydl;
        }
        public static double[] fastMatch(double[,] BWmap, double[,] p2c_scan, double[,] min_max_val, double[] resolution, double pixelSize, double[] pos, int scan_size, double[] best_hits)
        {
            double ipixel = 1 / pixelSize;
            double minX = min_max_val[0, 0] - 3 * pixelSize;
            double minY = min_max_val[0, 1] - 3 * pixelSize;
            int nCols = BWmap.GetLength(1);
            int nRows = BWmap.GetLength(0);
            int max_Iter = 50;
            int maxDepth = 3;
            int depth = 0;
            double[] bestPos = pos;
            double best_score = 99999999;
            double[,] pixel_scan = new double[scan_size, 2];
            bool noChange = true;
            double[] theta = new double[3];
            double[] tx = new double[3];
            double[] ty = new double[3];
            double r = resolution[0];
            double t = resolution[1];
            r = deg2rad(r);
            theta[0] = -r + pos[2]; theta[1] = pos[2]; theta[2] = r + pos[2];
            tx[0] = -t + pos[0]; tx[1] = pos[0]; tx[2] = t + pos[0];
            ty[0] = -t + pos[1]; tx[1] = pos[1]; tx[2] = t + pos[1];

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
                        double score = 0;
                        int[] s_x = new int[scan_size];
                        int[] s_y = new int[scan_size];
                        double[] temp_best_hits = new double[scan_size];
                        int r_x = 0;
                        int r_y = 0;
                        for (int i = 0; i < scan_size; i++)
                        {
                            s_x[i] = (int)Math.Round(S_[i, 0] + (tx[j] - minX) * ipixel) + 1;
                            s_y[i] = (int)Math.Round(S_[i, 1] + (ty[j] - minY) * ipixel) + 1;
                            if (s_x[i] > 1 && s_y[i] > 1 && s_x[i] < nCols && s_y[i] < nRows)
                            {
                                int ix = 0;
                                int iy = 0;
                                ix = s_x[i];
                                iy = s_y[i];
                                int idx = iy + (ix - 1) * nRows;
                                r_x = idx % BWmap.GetLength(0); // 재 검토 필요
                                r_y = idx / BWmap.GetLength(0);
                                score = score + BWmap[r_x, r_y];
                                temp_best_hits[i] = BWmap[r_x, r_y];
                            }
                        }
                        if (score < best_score)
                        {
                            noChange = false;
                            bestPos[0] = tx[j]; bestPos[1] = ty[j]; bestPos[2] = theta[t_i];
                            best_score = score;
                            best_hits = temp_best_hits;
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
            double minX = min_max_val[0, 0] - 3 * pixelSize;
            double minY = min_max_val[0, 1] - 3 * pixelSize;
            int nCols = BWmap.GetLength(1);
            int nRows = BWmap.GetLength(0);
            int max_Iter = 50;
            int maxDepth = 3;
            int depth = 0;
            double[] bestPos = new double[3] { pos_guess[0], pos_guess[1], pos_guess[2] };
            double best_score = 99999999;
            double[,] pixel_scan = new double[scan_size, 2];
            bool noChange = true;
            double[] theta = new double[3];
            double[] tx = new double[3];
            double[] ty = new double[3];
            double r = resolution[0];
            double t = resolution[1];
            r = deg2rad(r);
            theta[0] = -r + pos_guess[2]; theta[1] = pos_guess[2]; theta[2] = r + pos_guess[2];
            tx[0] = -t + pos_guess[0]; tx[1] = pos_guess[0]; tx[2] = t + pos_guess[0];
            ty[0] = -t + pos_guess[1]; tx[1] = pos_guess[1]; tx[2] = t + pos_guess[1];

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
                        double score = 0;
                        int[] s_x = new int[scan_size];
                        int[] s_y = new int[scan_size];
                        double[] temp_best_hits = new double[scan_size];
                        int r_x = 0;
                        int r_y = 0;
                        for (int i = 0; i < scan_size; i++)
                        {
                            s_x[i] = (int)Math.Round(S_[i, 0] + (tx[j] - minX) * ipixel) + 1;
                            s_y[i] = (int)Math.Round(S_[i, 1] + (ty[j] - minY) * ipixel) + 1;
                            if (s_x[i] > 1 && s_y[i] > 1 && s_x[i] < nCols && s_y[i] < nRows)
                            {
                                int ix = 0;
                                int iy = 0;
                                ix = s_x[i];
                                iy = s_y[i];
                                int idx = iy + (ix - 1) * nRows;
                                r_x = idx % BWmap.GetLength(0); 
                                r_y = idx / BWmap.GetLength(0);
                                score = score + BWmap[r_x, r_y];
                            }
                        }
                        if (score < best_score)
                        {
                            noChange = false;
                            bestPos[0] = tx[j]; bestPos[1] = ty[j]; bestPos[2] = theta[t_i];
                            best_score = score;
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
        public static int[] extract_sgrid(double[,] min_max_val, double pixelSize, int[] sgrid)
        {
            min_max_val[0, 0] = min_max_val[0, 0] - 3 * pixelSize;
            min_max_val[0, 1] = min_max_val[0, 1] - 3 * pixelSize;
            min_max_val[1, 0] = min_max_val[1, 0] + 3 * pixelSize;
            min_max_val[1, 1] = min_max_val[1, 1] + 3 * pixelSize;
            sgrid[0] = (int)(Math.Round((min_max_val[1, 0] - min_max_val[0, 0]) / pixelSize) + 1);
            sgrid[1] = (int)(Math.Round((min_max_val[1, 1] - min_max_val[0, 1]) / pixelSize) + 1);
            return sgrid;
        }
        public static int[] cal_idx(double[,] localMap, double[,] min_max_val, int[] sgrid, double pixelSize, int scan_size, int[] idx)
        {
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
        public static double[,] ExtractLocalMap(double[,] map_points, double[,] scan, double[] pos, int scan_size, double[,] localMap)
        {
            double[,] scan_w = new double[scan_size, 2];
            double[,] min_max_val = new double[2, 2];
            scan_w = transform(scan, pos, scan_size, scan_w);
            min_max_val = min_max_func(scan_w, scan_size, min_max_val);
            double minX = min_max_val[0, 0], minY = min_max_val[0, 1], maxX = min_max_val[1, 0], maxY = min_max_val[1, 1];
            
            for (int i = 0; i < scan_size; i++)
            {
                if (map_points[i, 0] > minX && map_points[i, 0] < maxX && map_points[i, 1] > minY && map_points[i, 1] < maxY)
                {
                    localMap[i, 0] = map_points[i, 0];
                    localMap[i, 1] = map_points[i, 1];
            
                }
            }
            return localMap;
        }
        public static double[,] transform(double[,] scan, double[] pos, int scan_size, double[,] trans_scan)
        {
            double t_x = pos[0];
            double t_y = pos[1];
            double t_theta = pos[2];
            double c_theta = Math.Cos(t_theta);
            double s_theta = Math.Sin(t_theta);
            double[,] R_ = new double[2, 2];
            R_[0, 0] = c_theta; R_[0, 1] = -s_theta; R_[1, 0] = s_theta; R_[1, 1] = c_theta;
            R_ = Dgetr(R_);
           
            for (int i = 0; i < scan_size; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    for (int k = 0; k < 2; k++)
                    {
                        trans_scan[i, j] += scan[i, k] * R_[k, j];
                    }
                    if (j == 0)
                        trans_scan[i, j] = trans_scan[i, j] + t_x;
                    else if (j == 1)
                        trans_scan[i, j] = trans_scan[i, j] + t_y;
                }
                
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
        public static void write_data(string savePath, double point1, double point2)
        {
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
            double minX = 0, minY = 0, maxX = 0, maxY = 0;
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
            min_max_value[0, 0] = minX; min_max_value[0, 1] = minY; min_max_value[1, 0] = maxX; min_max_value[1, 1] = maxY;
            return min_max_value;
        }
    }
}
