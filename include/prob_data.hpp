// include/prob_data.hpp

#ifndef PROB_DATA_HPP
#define PROB_DATA_HPP

#include <vector>
#include <string>

// Forward declarations if needed
// class SomeOtherClass;

// Class definition for ProblemData
class ProblemData {
public:

    // Constants to represent polytopes
    static const int POLYTOPE_TYPE_A = 1; // Example constant for Polytope Type A
    static const int POLYTOPE_TYPE_B = 2; // Example constant for Polytope Type B

    // Constructor
    ProblemData();

    // Destructor
    ~ProblemData();

    // Member functions
    void loadData(const std::string& filename);
    void processData();
    void clearData();

    // Getters
    const std::vector<int>& getData() const;

private:
    // Member variables
    std::vector<int> data; // Example data container
    // Other member variables as needed
};

#endif // PROB_DATA_HPP