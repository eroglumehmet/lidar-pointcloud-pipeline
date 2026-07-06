#include<iostream>
#include <cmath>
#include<vector>
#include<fstream>
#include <random>
#include <chrono>
#include <omp.h>

struct Point{
    double x;
    double y;
    double z;
};

Point Sub(const Point& a, const Point& b){
    return Point{a.x-b.x, a.y-b.y, a.z-b.z};
}

Point Cross(const Point& a, const Point& b){
    return Point{
        a.y*b.z - a.z*b.y,
        a.z*b.x - a.x*b.z,
        a.x*b.y - a.y*b.x
    };
}

double Dot(const Point& a, const Point& b){
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

double Norm(const Point& a){
    return std::sqrt(Dot(a,a));
}

int main(){
    std::vector<Point> points;
    std::ifstream input_file("cpp/data/cloud.csv");
    if (!input_file){
        std::cerr << "Error opening file!" << std::endl;
        return 1;
    }
    double x, y, z;
    char comma;
    while(input_file >> x >> comma >> y >> comma >> z){
        points.push_back(Point{x, y, z});
    }
    std::size_t N = points.size();
    std::cout << "Nokta sayısı: " << N << std::endl;

    int num_iter = 1000;
    double threshold = 15.0;

    int global_best_count = 0;
    Point global_best_n = {};
    double global_best_d = 0;

    auto t0 = std::chrono::steady_clock::now();

    #pragma omp parallel
    {
        int local_best_count = 0;
        Point local_best_n = {};
        double local_best_d = 0;

        std::mt19937 gen(42 + omp_get_thread_num());
        std::uniform_int_distribution<int> dist(0, (int)N-1);

        #pragma omp for nowait
        for(int iter = 0; iter < num_iter; iter++){
            int i = dist(gen);
            int j = dist(gen);
            while(j == i) j = dist(gen);
            int k = dist(gen);
            while(k == i || k == j) k = dist(gen);

            Point p0 = points[i], p1 = points[j], p2 = points[k];
            Point n = Cross(Sub(p1,p0), Sub(p2,p0));
            double n_length = Norm(n);
            if(n_length < 1e-6) continue;

            double d = -Dot(n, p0);
            double threshold_scaled = threshold * n_length;

            int count = 0;
            for(std::size_t m = 0; m < N; m++){
                double val = std::fabs(Dot(n, points[m]) + d);
                if(val < threshold_scaled) count++;
            }

            if(count > local_best_count){
                local_best_count = count;
                local_best_n = n;
                local_best_d = d;
            }
        }

        #pragma omp critical
        {
            if(local_best_count > global_best_count){
                global_best_count = local_best_count;
                global_best_n = local_best_n;
                global_best_d = local_best_d;
            }
        }
    }

    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    double length = Norm(global_best_n);
    std::cout << "Best count: " << global_best_count << std::endl;
    std::cout << "Best normal: ("
              << global_best_n.x/length << ", "
              << global_best_n.y/length << ", "
              << global_best_n.z/length << ")" << std::endl;
    std::cout << "Measured time: " << ms << "ms" << std::endl;
    return 0;
}
