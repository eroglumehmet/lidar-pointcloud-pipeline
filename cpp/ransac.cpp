#include<iostream>
#include <cmath>
#include<vector>
#include<fstream>
#include <random>
#include <chrono>

struct Point{
    double x;
    double y;
    double z;
};

Point Sub(const Point& a, const Point& b){
    Point result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;
    return result;
}

Point Cross(const Point& a, const Point& b){
    Point result;
    result.x = a.y * b.z - a.z * b.y;
    result.y = a.z * b.x - a.x * b.z;
    result.z = a.x * b.y - a.y * b.x;
    return result;
}

double Dot(const Point& a, const Point& b){
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

double Norm(const Point& a){
    return std::sqrt(Dot(a , a));
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
    
    std::mt19937 gen(42);
    std::uniform_int_distribution<int> dist(0, N-1);
    
    int best_count = 0;
    Point best_n = {};
    double best_d = 0;
    
    int num_iter = 1000;
    double threshold = 15.0;
    auto t0 = std::chrono::steady_clock::now();
    for(int iter = 0 ; iter < num_iter ; iter++){
        int i = dist(gen);
        int j = dist(gen);
        while(j == i){
            j = dist(gen);
        }
        int k = dist(gen);
        while(k == i || k == j){
            k = dist(gen);
        }
        
        Point p0 = points[i];
        Point p1 = points[j];
        Point p2 = points[k];
        
        Point n = Cross(Sub(p1  ,p0), Sub(p2 , p0));
        
        double n_length = Norm(n);
        if(n_length < 1e-6) continue;
        
        double d = -Dot(n , p0);
        
        int count = 0;
        for(std::size_t m = 0 ; m < N ; m++){
            double distance = std::fabs(Dot(n, points[m]) + d) / n_length;
            if(distance < threshold) count++;
        }
        if(count > best_count){
            best_count = count;
            best_n = n;
            best_d = d;
        }
    }
    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::cout << "Best count: " << best_count << std::endl;
    double length = Norm(best_n);
    std::cout << "Best normal: ("
              << best_n.x / length << ", "
              << best_n.y / length << ", "
              << best_n.z / length << ")" << std::endl;
    std::cout << "Measured time: " << ms << "ms"<< std::endl;
    return 0;
}

