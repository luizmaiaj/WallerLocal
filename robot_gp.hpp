#ifndef ROBOT_GP_HPP
#define ROBOT_GP_HPP

#include "gp_engine.hpp"
#include "robot.h"

struct ball_data {
    int dir;
    double lin;
    double col;
};
#include <variant>

namespace robot_gp {

// Command types
enum class RobotCommand {
    // Function nodes
    PROGN3,     // Execute 3 commands in sequence
    PROGN2,     // Execute 2 commands in sequence
    IFWALL,     // Execute left if near wall, right otherwise
    IFBALL,     // Execute left if ball visible, right otherwise
    
    // Terminal nodes
    WALKFRONT,  // Move forward
    WALKBACK,   // Move backward
    LEFT,       // Turn left
    RIGHT,      // Turn right
    ALIGN       // Orient towards ball
};

// Node value that holds a robot command
struct RobotNodeValue {
    RobotCommand cmd;
    
    // Get number of children required for this command
    [[nodiscard]] size_t children_count() const {
        switch (cmd) {
            case RobotCommand::PROGN3:
                return 3;
            case RobotCommand::PROGN2:
            case RobotCommand::IFWALL:
            case RobotCommand::IFBALL:
                return 2;
            default:
                return 0;
        }
    }
    
    // Check if this is a function node
    [[nodiscard]] bool is_function() const {
        return children_count() > 0;
    }
    
    // Get character representation (for compatibility with existing code)
    [[nodiscard]] char to_char() const {
        switch (cmd) {
            case RobotCommand::PROGN3: return '3';
            case RobotCommand::PROGN2: return '2';
            case RobotCommand::IFWALL: return 'I';
            case RobotCommand::IFBALL: return 'C';
            case RobotCommand::WALKFRONT: return 'F';
            case RobotCommand::WALKBACK: return 'B';
            case RobotCommand::LEFT: return 'L';
            case RobotCommand::RIGHT: return 'R';
            case RobotCommand::ALIGN: return 'A';
        }
        return '?';
    }
    
    // Create from character (for compatibility with existing code)
    static RobotNodeValue from_char(char c) {
        switch (c) {
            case '3': return {RobotCommand::PROGN3};
            case '2': return {RobotCommand::PROGN2};
            case 'I': return {RobotCommand::IFWALL};
            case 'C': return {RobotCommand::IFBALL};
            case 'F': return {RobotCommand::WALKFRONT};
            case 'B': return {RobotCommand::WALKBACK};
            case 'L': return {RobotCommand::LEFT};
            case 'R': return {RobotCommand::RIGHT};
            case 'A': return {RobotCommand::ALIGN};
            default: throw std::invalid_argument("Invalid robot command character");
        }
    }
};

// Evaluator class that executes robot commands
class RobotEvaluator {
private:
    Robot& robot;
    struct ball_data& ball;

public:
    RobotEvaluator(Robot& r, struct ball_data& b) : robot(r), ball(b) {}
    
    void execute_command(const RobotNodeValue& cmd) {
        switch (cmd.cmd) {
            case RobotCommand::WALKFRONT:
                robot.walkFront();
                break;
            case RobotCommand::WALKBACK:
                robot.walkBack();
                break;
            case RobotCommand::LEFT:
                robot.turnLeft();
                break;
            case RobotCommand::RIGHT:
                robot.turnRight();
                break;
            case RobotCommand::ALIGN:
                robot.align(ball.lin, ball.col);
                break;
            default:
                // Function nodes are handled by the GP engine
                break;
        }
    }
    
    bool check_condition(const RobotNodeValue& cmd) {
        switch (cmd.cmd) {
            case RobotCommand::IFWALL:
                return robot.isNearWall();
            case RobotCommand::IFBALL:
                return robot.canSeeBall(ball.lin, ball.col);
            default:
                return false;
        }
    }
};

// Tree generator for robot programs
class TreeGenerator {
private:
    std::mt19937& rng;

public:
    explicit TreeGenerator(std::mt19937& random_engine) : rng(random_engine) {}

    // Generate random terminal command
    RobotNodeValue generate_terminal() {
        static const RobotCommand terminals[] = {
            RobotCommand::WALKFRONT,
            RobotCommand::WALKBACK,
            RobotCommand::LEFT,
            RobotCommand::RIGHT,
            RobotCommand::ALIGN
        };
        
        std::uniform_int_distribution<size_t> dist(0, std::size(terminals) - 1);
        return RobotNodeValue{terminals[dist(rng)]};
    }

    // Generate random function command
    RobotNodeValue generate_function() {
        static const RobotCommand functions[] = {
            RobotCommand::PROGN3,
            RobotCommand::PROGN2,
            RobotCommand::IFWALL,
            RobotCommand::IFBALL
        };
        
        std::uniform_int_distribution<size_t> dist(0, std::size(functions) - 1);
        return RobotNodeValue{functions[dist(rng)]};
    }

    // Point mutation - changes command while preserving arity
    RobotNodeValue point_mutate(const RobotNodeValue& old_value) {
        if (old_value.is_function()) {
            // Mutate to another function with same arity
            size_t old_arity = old_value.children_count();
            RobotNodeValue new_value;
            do {
                new_value = generate_function();
            } while (new_value.children_count() != old_arity);
            return new_value;
        } else {
            // Mutate to another terminal
            return generate_terminal();
        }
    }

    // Create a complete random tree
    gp::Tree<RobotNodeValue> generate_tree(size_t max_depth) {
        using namespace gp;
        Tree<RobotNodeValue> tree;
        
        // Create root node
        tree.root = std::make_unique<Node<RobotNodeValue>>(generate_function());
        
        // Build tree recursively
        std::function<void(Node<RobotNodeValue>*, size_t)> build_tree;
        build_tree = [this, &build_tree](Node<RobotNodeValue>* node, size_t depth) {
            size_t num_children = node->value.value.children_count();
            
            for (size_t i = 0; i < num_children; ++i) {
                RobotNodeValue child_value;
                if (depth <= 1) {
                    child_value = generate_terminal();
                } else {
                    std::uniform_int_distribution<int> dist(0, 1);
                    child_value = dist(rng) == 0 ? generate_terminal() : generate_function();
                }
                
                auto child = std::make_unique<Node<RobotNodeValue>>(child_value);
                if (child_value.is_function() && depth > 1) {
                    build_tree(child.get(), depth - 1);
                }
                node->add_child(std::move(child));
            }
        };
        
        build_tree(tree.root.get(), max_depth);
        return tree;
    }
};

} // namespace robot_gp

#endif // ROBOT_GP_HPP