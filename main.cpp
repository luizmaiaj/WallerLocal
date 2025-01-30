#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <chrono>
#include <random>

#include "environment.h"
#include "robot.h"
#include "constants.h"
#include "gp_engine.hpp"
#include "robot_gp.hpp"

// Image tracking globals (kept for compatibility)
unsigned char best_track[HEIGHT][WIDTH][3];
int robot_track[HEIGHT][WIDTH];
int ball_track[HEIGHT][WIDTH];

// File handling helper functions
int countExistingFiles(const std::string& baseName, const std::string& extension) {
    int count = 0;
    for (int i = 0;; ++i) {
        std::stringstream ss;
        ss << baseName << std::setfill('0') << std::setw(3) << i << extension;
        if (!std::filesystem::exists(ss.str())) {
            break;
        }
        count++;
    }
    return count;
}

void saveBestTrack(int generation) {
    std::cout << "\n\t\tSaving visualization...\n";
    std::ostringstream filename_ss, command_ss;
    filename_ss << "paths/caminho" << std::setfill('0') << std::setw(3) << generation << ".ppm";
    
    std::ofstream track_file(filename_ss.str(), std::ios::out | std::ios::binary);
    if (!track_file) {
        std::cerr << "Failed to create track file: " << filename_ss.str() << "\n";
        return;
    }

    // Write PPM header
    track_file << "P3\n200 200\n255\n";

    // Write pixel data
    for (int lin = 0; lin < HEIGHT; lin++) {
        for (int col = 0; col < WIDTH; col++) {
            track_file << (int)best_track[lin][col][0] << " "
                      << (int)best_track[lin][col][1] << " "
                      << (int)best_track[lin][col][2] << "   ";
        }
        track_file << "\n";
    }
    track_file.close();

    // Convert to GIF and cleanup
    command_ss << "magick " << filename_ss.str() << " paths/caminho" 
               << std::setfill('0') << std::setw(3) 
               << generation << ".gif && rm " << filename_ss.str();
    system(command_ss.str().c_str());
}

void updateBestTrack(const Environment& env, const Robot& robot, const ball_data& ball) {
    // Clear track
    std::memset(best_track, 0, sizeof(best_track));
    
    // Draw borders and obstacles
    for (int lin = 0; lin < HEIGHT; lin++) {
        for (int col = 0; col < WIDTH; col++) {
            if (lin == 0 || lin == HEIGHT-1 || col == 0 || col == WIDTH-1 || 
                env.getCell(lin, col) == 2) {
                best_track[lin][col][0] = 255;
                best_track[lin][col][1] = 255;
                best_track[lin][col][2] = 255;
            }
        }
    }

    // Draw robot path
    for (int lin = 0; lin < HEIGHT; lin++) {
        for (int col = 0; col < WIDTH; col++) {
            if (robot_track[lin][col] > 0) {
                if (robot_track[lin][col] < 20) {
                    best_track[lin][col][0] = 255;
                    best_track[lin][col][1] = 255;
                    best_track[lin][col][2] = 0;
                } else {
                    best_track[lin][col][0] = 255;
                    best_track[lin][col][1] = 0;
                    best_track[lin][col][2] = 0;
                }
            }
        }
    }

    // Draw ball path
    for (int lin = 0; lin < HEIGHT; lin++) {
        for (int col = 0; col < WIDTH; col++) {
            if (ball_track[lin][col] == 1) {
                best_track[lin][col][0] = 0;
                best_track[lin][col][1] = 255;
                best_track[lin][col][2] = 0;
            }
        }
    }
}

int main() {
    // Initialize random number generator
    std::random_device rd;
    auto seed = rd();
    std::mt19937 rng(seed);

    // Create environment and robot
    Environment env;
    Robot robot(env);
    ball_data ball{};

    // Initialize GP engine components
    robot_gp::TreeGenerator tree_generator(rng);
    robot_gp::FitnessEvaluator fitness_evaluator(env, robot, ball);

    // Configure GP parameters
    gp::GPEngine<robot_gp::RobotNodeValue, robot_gp::FitnessEvaluator>::Parameters params;
    params.population_size = POPULATION;
    params.generations = GENS;
    params.crossover_rate = static_cast<double>(CROSSING) / POPULATION;
    params.mutation_rate = 0.1;  // Added mutation which wasn't in original
    params.tournament_size = 5;
    params.max_depth = 17;      // Equivalent to original LIMIT
    params.max_nodes = 100;     // New parameter for safety

    // Create GP engine
    gp::GPEngine<robot_gp::RobotNodeValue, robot_gp::FitnessEvaluator> gp_engine(params, fitness_evaluator);

    // Setup data logging
    auto data_file_count = countExistingFiles("data/data", ".txt");
    std::ofstream data_file("data/data" + std::to_string(data_file_count) + ".txt");
    if (!data_file) {
        std::cerr << "Failed to create data file\n";
        return 1;
    }

    // Write header
    data_file << "ROBO SEGUIDOR v2.0\n";
    data_file << "GERACAO\tMEDIA\t\tMAIOR\n";

    // Record start time
    auto start_time = std::chrono::system_clock::now();

    std::cout << "\nInitializing population...\n";
    gp_engine.initialize_population([&]() {
        return tree_generator.generate_tree(params.max_depth);
    });

    // Main evolution loop
    std::cout << "\nStarting evolution...\n";
    for (int gen = 0; gen < params.generations; gen++) {
        std::cout << "\nGeneration " << gen << " -> ";
        
        // Evolve one generation
        auto [best_fitness, avg_fitness] = gp_engine.evolve_with_stats();
        
        // Update visualization for best individual
        updateBestTrack(env, robot, ball);
        saveBestTrack(gen);

        // Log progress
        std::cout << "\nAverage Fitness: " << avg_fitness 
                  << "\nBest Fitness: " << best_fitness << "\n";
        
        data_file << gen << "\t" << avg_fitness << "\t" << best_fitness << "\n";
        data_file.flush();
    }

    // Save final population
    std::cout << "\nSaving final population...\n";
    auto robot_file_count = countExistingFiles("robots/rb", "tr.txt");
    for (int i = 0; i < 100 && i < params.population_size; i++) {
        std::string filename = "robots/rb" + 
            std::to_string((robot_file_count + i) % 1000) + "tr.txt";
            
        std::ofstream robot_file(filename);
        if (robot_file) {
            // Save in compatible format
            robot_file << gp_engine.get_individual(i).to_string() << "\n";
            robot_file << "LENGTH = " << gp_engine.get_individual(i).size() << "\n";
            robot_file << "FITNESS = " << gp_engine.get_individual(i).fitness << "\n";
        }
    }

    // Calculate and display runtime
    auto end_time = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
    int hours = duration.count() / 3600;
    int minutes = (duration.count() % 3600) / 60;
    int seconds = duration.count() % 60;

    std::cout << "\nTotal runtime: " 
              << std::setfill('0') << std::setw(2) << hours << ":"
              << std::setfill('0') << std::setw(2) << minutes << ":"
              << std::setfill('0') << std::setw(2) << seconds << "\n";

    return 0;
}
