using System;
using System.Collections.Generic;
using System.Text;
using YDLidarSharp;

class SLAM_utils
{
     public static double[,] create_BWmap(double[,] localMap, double[,] min_max_val, double pixel_size) 
     {
        int map_size = localMap.GetLength(0);
        int[] sgrid = SLAM_utils.extract_sgrid(min_max_val, pixel_size);
        int[] idx = SLAM_utils.cal_idx(localMap, min_max_val, sgrid, pixel_size, map_size);
        bool[,] gridMap = new bool[sgrid[1], sgrid[0]];
        for (int i = 0; i < map_size; i++)
        {
            double temp_y = idx[i] / sgrid[1];
            int t_y = (int)Math.Truncate(temp_y);
            gridMap[idx[i] % sgrid[1], t_y] = true;
        }
        double[,] BWmap = new double[sgrid[1], sgrid[0]];
        BWmap = SLAM_utils.distanceBW(gridMap, BWmap);
        return BWmap;
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
                            double distance = Math.Sqrt(Math.Pow(m - i, 2) + Math.Pow(n - j, 2));
                            distance = Math.Min(distance, 10);
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
        sgrid[0] = (int)(Math.Round((min_max_val[1, 0] - min_max_val[0, 0]) / pixelSize) + 1);
        sgrid[1] = (int)(Math.Round((min_max_val[1, 1] - min_max_val[0, 1]) / pixelSize) + 1);
        return sgrid;
    }
    public static int[] cal_idx(double[,] localMap, double[,] min_max_val, int[] sgrid, double pixelSize, int map_size, int[] idx)
    {
        int[,] hits = new int[map_size, 2];
        for (int i = 0; i < map_size; i++)
        {
            hits[i, 0] = (int)(Math.Round((localMap[i, 0] - min_max_val[0, 0]) / pixelSize) + 1);
            hits[i, 1] = (int)(Math.Round((localMap[i, 1] - min_max_val[0, 1]) / pixelSize) + 1);
            idx[i] = (hits[i, 0] - 1) * sgrid[1] + hits[i, 1];
        }
        return idx;
    }
    public static double[,] ExtractLocalMap(double[,] map_points, double[,] scan, double[] pos)
    {
        double[,] scan_w = new double[scan.GetLength(0), 2];
        double[,] min_max_val = new double[2, 2];
        scan_w = transform(scan, pos, scan.GetLength(0), scan_w);
        double minX = 99999999, minY = 99999999, maxX = -99999999, maxY = -99999999;
        for (int i = 0; i < scan.GetLength(0); i++)
        {
            if (scan_w[i, 0] - 1 < minX)
                minX = scan_w[i, 0] - 1;
            if (scan_w[i, 1] - 1 < minY)
                minY = scan_w[i, 1] - 1;
            if (scan_w[i, 0] + 1 > maxX)
                maxX = scan_w[i, 0] + 1;
            if (scan_w[i, 1] + 1 > maxY)
                maxY = scan_w[i, 1] + 1;
        }
        int cnt = 0;
        double[,] temp_localMap = new double[map_points.GetLength(0), 2];
        for (int i = 0; i < map_points.GetLength(0); i++)
        {
            if (map_points[i, 0] > minX && map_points[i, 0] < maxX && map_points[i, 1] > minY && map_points[i, 1] < maxY)
            {
                temp_localMap[cnt, 0] = map_points[i, 0];
                temp_localMap[cnt, 1] = map_points[i, 1];
                cnt++;
            }
        }
        double[,] localMap = new double[cnt, 2];
        for (int i = 0; i < cnt; i++)
        {
            localMap[i, 0] = temp_localMap[i, 0];
            localMap[i, 1] = temp_localMap[i, 1];
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
            }
            trans_scan[i, 0] += t_x;
            trans_scan[i, 1] += t_y;
        }
        return trans_scan;
    }
    public static double[,] pol2cart(LaserScan scan, double[,] p2c)
    {
        int scan_size = scan.LaserPoints.Count;
            
        for (int i = 0; i < scan_size; i++)
        {
            double angle = (double)scan.LaserPoints[i].Angle;
            double range = (double)scan.LaserPoints[i].Range;
            p2c[i, 0] = range * Math.Cos(angle);
            p2c[i, 1] = range * Math.Sin(angle);
            //write_data(savePath, p2c[i, 0], p2c[i, 1]);
        }
        return p2c;
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
        radian = degree * Math.PI / 180;
        return radian;
    }
    public static double rad2deg(double radian)
    {
        double degree = 0.0;
        //radian = degree / Math.PI;
        degree = radian * Math.PI;
        return radian;
    }
    public static double[] diffPos(double[] prev_pos, double[] pos, double[] dp)
    {
        dp[0] = pos[0] - prev_pos[0];
        dp[1] = pos[1] - prev_pos[1];
        //dp[2] = Math.PI - Math.Abs(Math.Abs(pos[2] - prev_pos[2]) - Math.PI);
        dp[2] = Math.Atan2(Math.Sin(prev_pos[2] - pos[2]), Math.Cos(prev_pos[2] - pos[2])); // 이 식 재 검토 필요

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

