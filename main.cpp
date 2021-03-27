#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath> 
#include <Eigen/Dense>
#include "A_Star.h"

using namespace Eigen;


bool compare_f(Node* a, Node* b) {
	return a->getF() < b->getF();
};


A_Star::A_Star(int s[2], int e[2], int lenx, int leny, std::vector<Obstacle> obs) {
	start[0] = s[0];
	start[1] = s[1];
	goal[0] = e[0];
	goal[1] = e[1];

	len_x = lenx;
	len_y = leny;

	board = MatrixXi::Zero(len_y, len_x);
	board(len_y - s[1] - 1, s[0]) = 1;
	board(len_y - e[1] - 1, e[0]) = 2;

	obstacles = obs;
	for (Obstacle o : obs) {
		board(len_y - o.y - 1, o.x) = 5;
	};

	std::cout << board << std::endl;

	node_id_ctr = 0;
};

int A_Star::manhattan_dist(int x1, int x2, int y1, int y2) {
	return std::abs(x2 - x1) + std::abs(y2 - y1);
};

bool A_Star::node_in_closed(Node& check_node) {
	if (std::find(closed.begin(), closed.end(), check_node.id) != closed.end()) {
		return true;
	}
	else {
		return false;
	}
};

void A_Star::find_path(Node* next) {
	// Reverse linked list to find path to goal
	std::vector<int> path_ids;
	while (next->id != 0) {
		std::cout << "PATH NODE: " <<"X: "<< next->x << "  Y: "<< next->y << std::endl;
		next = next->parent;
		board(len_y - next->y - 1, next->x) = 1;
	}
};

bool A_Star::search_adjacent(Node* curr) {
	// Iterate over nodes that are adjacent to current
	int adjacent[4][2] = { {1, 0}, {0, 1}, {-1, 0}, {0, -1} };
	for (int i = 0; i < 4; i++) {
		int pos_x = curr->x + adjacent[i][1];
		int pos_y = curr->y + adjacent[i][0];
		if (pos_x >= 0 && pos_x < len_x && pos_y >= 0 && pos_y < len_y) {
			bool free_bool = board(len_y - pos_y - 1, pos_x) != 5;
			bool closed_bool = board(len_y - pos_y - 1, pos_x) == 3;
			if (free_bool && !closed_bool) {
				Node* node = new Node;
				node->x = pos_x;
				node->y = pos_y;

				if (board(len_y - pos_y - 1, pos_x) == 2) {
					std::cout << "*************** SOLVED ***************" << std::endl;
					node->parent = curr;
					find_path(node);
					return true;
				}

				node->g = curr->g + manhattan_dist(node->x, curr->x, len_y - node->y - 1, len_y - curr->y - 1);
				node->h = manhattan_dist(node->x, goal[0], len_y - node->y - 1, len_y - goal[1] - 1);
				node->f = node->g + node->h;
				node->id = node_id_ctr;
				node->parent = curr;

				open.push_back(node);
				all_nodes.push_back(node);
				node_id_ctr += 1;
			};
		};
	};
	return false;
}

void A_Star::solve() {
	Node goal_node;
	goal_node.x = goal[0];
	goal_node.y = goal[1];
	goal_node.terminal = true;

	Node* start_node = new Node;
	start_node->x = start[0];
	start_node->y = start[1];
	start_node->g = 0;
	start_node->h = 0; 
	start_node->f = 0;
	start_node->id = node_id_ctr;
	node_id_ctr += 1;

	open.push_back(start_node);
	all_nodes.push_back(start_node);
	
	bool found_goal = false;

	int ctr = 0;
	Node* curr_node;
	while (!open.empty() && !found_goal) {
		std::sort(open.begin(), open.end(), compare_f);
		curr_node = *open.begin();
		closed.push_back(curr_node->id);
		board(len_y - curr_node->y - 1, curr_node->x) = 3;

		found_goal = search_adjacent(curr_node);
		ctr += 1;
		open.erase(open.begin());
	};
	std::cout << std::endl;
	std::cout << board << std::endl;
};


int main() {
	const int num_rows = 15;
	const int num_cols = 15;
	int starting_pos[2] = { 7, 0 };
	int ending_pos[2] = { 7, 6 };

	std::vector<Obstacle> obstacles;
	Obstacle o1 = { 7, 4 };
	obstacles.push_back(o1);

	Obstacle o2 = { 6, 4 };
	obstacles.push_back(o2);

	Obstacle o3 = { 8, 4 };
	obstacles.push_back(o3);

	A_Star astar(starting_pos, ending_pos, num_rows, num_cols, obstacles);
	astar.solve();

	return 0;
}