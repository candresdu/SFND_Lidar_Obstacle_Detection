#include <iostream>
#include <vector>
#include <algorithm>

// Test a function to check if a point (i.e.: vector) is in a list
void inList(std::vector<float> point, std::vector<std::vector<float>> list){
	if (std::find(list.begin(), list.end(), point) != list.end()) {
        std::cout << "Element found" << std::endl;
    }
    else {
        std::cout << "Element not found" << std::endl;
    }
}


int main () {
	std::vector<std::vector<float>> vec_list = {{1,2}, {3,4}, {1,3}, {2,4}, {5,5}};

	std::vector<std::vector<float>> test_list = {{1,2}, {3,4}, {3,3}, {2,4}, {1,5}};

	for (std::vector<float> test_vec : test_list){
		inList(test_vec, vec_list);
	}


}
