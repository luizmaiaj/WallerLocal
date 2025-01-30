#ifndef GP_ENGINE_HPP
#define GP_ENGINE_HPP

#include <memory>
#include <vector>
#include <random>
#include <functional>
#include <span>
#include <type_traits>
#include <algorithm>
#include <queue>
#include <stack>

namespace gp {

// Forward declarations
template<typename T> class Node;
template<typename T> class Tree;

// Type traits for compile-time checks
template<typename T>
struct is_evaluable {
    template<typename U>
    static auto test(U*) -> decltype(std::declval<U>().evaluate(), std::true_type{});
    
    template<typename>
    static std::false_type test(...);
    
    static constexpr bool value = decltype(test<T>(nullptr))::value;
};

template<typename T>
struct is_crossable {
    template<typename U>
    static auto test(U*) -> decltype(std::declval<U>().crossover(std::declval<U>()), std::true_type{});
    
    template<typename>
    static std::false_type test(...);
    
    static constexpr bool value = decltype(test<T>(nullptr))::value;
};

// Node structure using type-safe value storage
template<typename T>
class Node {
public:
    using NodePtr = std::unique_ptr<Node<T>>;
    
    struct NodeValue {
        T value;
        std::function<double()> evaluate;
        bool is_function{false}; // true if node represents a function, false if terminal
    };

    NodeValue value;
    std::vector<NodePtr> children;
    Node* parent{nullptr}; // Non-owning pointer to parent for easier tree manipulation
    
    // Constructor for terminal nodes
    explicit Node(T val) 
        : value{std::move(val), nullptr, false} {}
    
    // Constructor for function nodes
    Node(T val, std::function<double()> eval_func) 
        : value{std::move(val), std::move(eval_func), true} {}

    // Deep copy constructor
    Node(const Node& other) : value(other.value) {
        children.reserve(other.children.size());
        for (const auto& child : other.children) {
            auto new_child = std::make_unique<Node>(*child);
            new_child->parent = this;
            children.push_back(std::move(new_child));
        }
    }
    
    // Move constructor
    Node(Node&&) noexcept = default;
    
    // Deep copy assignment
    Node& operator=(const Node& other) {
        if (this != &other) {
            value = other.value;
            children.clear();
            children.reserve(other.children.size());
            for (const auto& child : other.children) {
                auto new_child = std::make_unique<Node>(*child);
                new_child->parent = this;
                children.push_back(std::move(new_child));
            }
        }
        return *this;
    }
    
    // Move assignment
    Node& operator=(Node&&) noexcept = default;

    [[nodiscard]] double evaluate() const {
        if (value.evaluate) {
            return value.evaluate();
        }
        return 0.0; // Terminal node default
    }

    // Add child node
    void add_child(NodePtr child) {
        if (child) {
            child->parent = this;
            children.push_back(std::move(child));
        }
    }

    // Get all nodes in the subtree (including this node)
    [[nodiscard]] std::vector<Node*> get_subtree_nodes() {
        std::vector<Node*> nodes;
        std::queue<Node*> queue;
        queue.push(this);

        while (!queue.empty()) {
            auto current = queue.front();
            queue.pop();
            nodes.push_back(current);

            for (auto& child : current->children) {
                queue.push(child.get());
            }
        }
        return nodes;
    }

    // Get node depth
    [[nodiscard]] size_t depth() const {
        size_t max_child_depth = 0;
        for (const auto& child : children) {
            max_child_depth = std::max(max_child_depth, child->depth());
        }
        return 1 + max_child_depth;
    }

    // Replace this node with another node
    void replace_with(NodePtr new_node) {
        if (!parent || !new_node) return;

        for (auto& sibling : parent->children) {
            if (sibling.get() == this) {
                new_node->parent = parent;
                sibling = std::move(new_node);
                break;
            }
        }
    }
};

// Tree class representing a complete genetic program
template<typename T>
class Tree {
public:
    using NodeType = Node<T>;
    using NodePtr = std::unique_ptr<NodeType>;

    NodePtr root;
    double fitness{0.0};

    Tree() = default;
    explicit Tree(NodePtr r) : root(std::move(r)) {}

    // Deep copy constructor
    Tree(const Tree& other) : fitness(other.fitness) {
        if (other.root) {
            root = std::make_unique<NodeType>(*other.root);
        }
    }

    // Move constructor
    Tree(Tree&&) noexcept = default;

    // Copy assignment
    Tree& operator=(const Tree& other) {
        if (this != &other) {
            fitness = other.fitness;
            if (other.root) {
                root = std::make_unique<NodeType>(*other.root);
            } else {
                root.reset();
            }
        }
        return *this;
    }

    // Move assignment
    Tree& operator=(Tree&&) noexcept = default;

    [[nodiscard]] double evaluate() const {
        if (root) {
            return root->evaluate();
        }
        return 0.0;
    }

    // Get tree depth
    [[nodiscard]] size_t depth() const {
        if (root) {
            return root->depth();
        }
        return 0;
    }

    // Get all nodes in the tree
    [[nodiscard]] std::vector<NodeType*> get_all_nodes() {
        if (!root) return {};
        return root->get_subtree_nodes();
    }

    // Get random node
    [[nodiscard]] NodeType* get_random_node(std::mt19937& rng) {
        auto nodes = get_all_nodes();
        if (nodes.empty()) return nullptr;
        
        std::uniform_int_distribution<size_t> dist(0, nodes.size() - 1);
        return nodes[dist(rng)];
    }

    // Get random function node
    [[nodiscard]] NodeType* get_random_function_node(std::mt19937& rng) {
        auto nodes = get_all_nodes();
        std::vector<NodeType*> function_nodes;
        std::copy_if(nodes.begin(), nodes.end(), std::back_inserter(function_nodes),
                    [](const NodeType* node) { return node->value.is_function; });
        
        if (function_nodes.empty()) return nullptr;
        
        std::uniform_int_distribution<size_t> dist(0, function_nodes.size() - 1);
        return function_nodes[dist(rng)];
    }

    // Get random terminal node
    [[nodiscard]] NodeType* get_random_terminal_node(std::mt19937& rng) {
        auto nodes = get_all_nodes();
        std::vector<NodeType*> terminal_nodes;
        std::copy_if(nodes.begin(), nodes.end(), std::back_inserter(terminal_nodes),
                    [](const NodeType* node) { return !node->value.is_function; });
        
        if (terminal_nodes.empty()) return nullptr;
        
        std::uniform_int_distribution<size_t> dist(0, terminal_nodes.size() - 1);
        return terminal_nodes[dist(rng)];
    }
};

// Main GP Engine class
template<typename T, typename FitnessFunction>
class GPEngine {
public:
    struct Parameters {
        std::size_t population_size = 500;
        std::size_t generations = 50;
        double crossover_rate = 0.7;
        double mutation_rate = 0.1;
        std::size_t tournament_size = 5;
        std::size_t max_depth = 17;
        std::size_t max_nodes = 100;
    };

private:
    Parameters params;
    std::vector<Tree<T>> population;
    FitnessFunction fitness_function;
    std::mt19937 rng;

public:
    explicit GPEngine(Parameters p, FitnessFunction f)
        : params(std::move(p))
        , fitness_function(std::move(f))
        , rng(std::random_device{}()) {}

    void initialize_population(std::function<Tree<T>()> tree_generator) {
        population.clear();
        population.reserve(params.population_size);
        
        for (std::size_t i = 0; i < params.population_size; ++i) {
            population.push_back(tree_generator());
        }
    }

    void evolve() {
        for (std::size_t gen = 0; gen < params.generations; ++gen) {
            // Evaluate fitness for all individuals
            for (auto& individual : population) {
                individual.fitness = fitness_function(individual);
            }

            // Sort population by fitness
            std::sort(population.begin(), population.end(),
                     [](const auto& a, const auto& b) {
                         return a.fitness > b.fitness;
                     });

            // Create new generation
            std::vector<Tree<T>> new_population;
            new_population.reserve(params.population_size);

            // Elitism: Keep best individual
            new_population.push_back(population.front());

            // Fill rest of population with crossover and mutation
            while (new_population.size() < params.population_size) {
                // Tournament selection
                auto parent1 = tournament_select();
                auto parent2 = tournament_select();

                // Crossover
                if (std::uniform_real_distribution<>(0, 1)(rng) < params.crossover_rate) {
                    auto [child1, child2] = crossover(parent1, parent2);
                    if (child1.depth() <= params.max_depth && child2.depth() <= params.max_depth) {
                        new_population.push_back(std::move(child1));
                        if (new_population.size() < params.population_size) {
                            new_population.push_back(std::move(child2));
                        }
                    } else {
                        // If children exceed max depth, keep parents
                        new_population.push_back(parent1);
                        if (new_population.size() < params.population_size) {
                            new_population.push_back(parent2);
                        }
                    }
                } else {
                    new_population.push_back(parent1);
                    if (new_population.size() < params.population_size) {
                        new_population.push_back(parent2);
                    }
                }
            }

            // Apply mutation
            for (auto& individual : new_population) {
                if (std::uniform_real_distribution<>(0, 1)(rng) < params.mutation_rate) {
                    mutate(individual);
                }
            }

            population = std::move(new_population);
        }
    }

    [[nodiscard]] const Tree<T>& get_best() const {
        return population.front();
    }

private:
    Tree<T> tournament_select() {
        std::vector<std::size_t> tournament_indices(params.tournament_size);
        std::uniform_int_distribution<std::size_t> dist(0, population.size() - 1);
        
        for (auto& idx : tournament_indices) {
            idx = dist(rng);
        }
        
        auto best_idx = *std::max_element(tournament_indices.begin(), 
                                        tournament_indices.end(),
                                        [this](auto a, auto b) {
                                            return population[a].fitness < population[b].fitness;
                                        });
        
        return population[best_idx];
    }

    std::pair<Tree<T>, Tree<T>> crossover(const Tree<T>& parent1, const Tree<T>& parent2) {
        Tree<T> offspring1 = parent1;
        Tree<T> offspring2 = parent2;

        // Get random crossover points
        auto* node1 = offspring1.get_random_node(rng);
        auto* node2 = offspring2.get_random_node(rng);

        if (!node1 || !node2) return {offspring1, offspring2};

        // Create deep copies of subtrees
        auto subtree1 = std::make_unique<Node<T>>(*node1);
        auto subtree2 = std::make_unique<Node<T>>(*node2);

        // Perform the swap
        node1->replace_with(std::move(subtree2));
        node2->replace_with(std::move(subtree1));

        return {std::move(offspring1), std::move(offspring2)};
    }

    // Generate a random subtree using the terminal and function set
    NodePtr generate_random_subtree(size_t max_depth, 
                                  const std::function<T()>& random_terminal,
                                  const std::function<T()>& random_function) {
        if (max_depth <= 1) {
            return std::make_unique<NodeType>(random_terminal());
        }

        std::uniform_int_distribution<int> dist(0, 1);
        if (dist(rng) == 0) {
            return std::make_unique<NodeType>(random_terminal());
        }

        auto node = std::make_unique<NodeType>(random_function());
        size_t num_children = node->value.children_count();

        for (size_t i = 0; i < num_children; ++i) {
            node->add_child(generate_random_subtree(max_depth - 1, random_terminal, random_function));
        }

        return node;
    }

    void mutate(Tree<T>& individual, 
                const std::function<T()>& random_terminal = nullptr,
                const std::function<T()>& random_function = nullptr,
                const std::function<T()>& point_mutate = nullptr) {
        auto* node = individual.get_random_node(rng);
        if (!node) return;

        // Different mutation types
        std::uniform_int_distribution<int> mut_type(0, 2);
        switch (mut_type(rng)) {
            case 0: // Point mutation: change node's value
                if (point_mutate) {
                    node->value = NodeType::NodeValue{point_mutate()};
                }
                break;

            case 1: // Subtree mutation: replace with new random subtree
                if (random_terminal && random_function) {
                    size_t remaining_depth = params.max_depth - node->depth();
                    if (remaining_depth > 0) {
                        auto new_subtree = generate_random_subtree(
                            remaining_depth,
                            random_terminal,
                            random_function
                        );
                        node->replace_with(std::move(new_subtree));
                    }
                }
                break;

            case 2: // Shrink mutation: replace function node with one of its children
                if (node->value.is_function && !node->children.empty()) {
                    std::uniform_int_distribution<size_t> child_dist(0, node->children.size() - 1);
                    auto child_idx = child_dist(rng);
                    auto child = std::make_unique<Node<T>>(*node->children[child_idx]);
                    node->replace_with(std::move(child));
                }
                break;
        }
    }
};

} // namespace gp

#endif // GP_ENGINE_HPP