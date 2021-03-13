#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath> 
#include <Eigen/Dense>
#include "A_Star.h"

using namespace Eigen;


bool compare_f(Node& a, Node& b) {
	return a.f < b.f;
};


A_Star::A_Star(int s[2], int e[2], int nrow, int ncol, std::vector<Obstacle> obs) {
	start[0] = s[0];
	start[1] = s[1];
	goal[0] = e[0];
	goal[1] = e[1];

	n_rows = nrow;
	n_cols = ncol;

	board = MatrixXi::Zero(n_rows, n_cols);
	board(s[0], s[1]) = 1;
	board(e[0], e[1]) = 2;

	obstacles = obs;
	for (Obstacle o : obs) {
		board(o.x, o.y) = 5;
	};

	std::cout << board << std::endl;
};

int A_Star::pos_to_id(int x, int y) {
	int i = y * n_rows + x;
	return i;
};

Node A_Star::id_to_node(int node_id) {
	for (Node& node : all_nodes) {
		if (node.id == node_id) {
			return node;
		}
	}
};

int A_Star::manhattan_dist(int x1, int x2, int y1, int y2) {
	return std::abs(x2 - x1) + std::abs(y2 - y1);
};

void A_Star::make_environment() {
	int id_ctr = 0;
	for (int row = 0; row < n_rows; row++) {
		for (int col = 0; col < n_cols; col++) {
			Node new_node;
			if (col == goal[0] && row == goal[1]) {
				new_node.terminal = true;
			}

			for (Obstacle o : obstacles) {
				if (col == o.x && row == o.y) {
					new_node.free = false;
				}
			}

			new_node.x = col;
			new_node.y = row;
			new_node.id = id_ctr;
			id_ctr++;
			all_nodes.push_back(new_node);
		};
	};
};

bool A_Star::node_in_closed(Node& check_node) {
	if (std::find(closed.begin(), closed.end(), check_node.id) != closed.end()) {
		return true;
	}
	else {
		return false;
	}
};

void A_Star::find_path(Node& next_node) {
	std::vector<int> path_ids;
	while (next_node.id != start_node.id) {
		path_ids.push_back(next_node.id);
		next_node = id_to_node(next_node.parent_id);
		board(next_node.x, next_node.y) = 1;
	}
};

bool A_Star::search_adjacent(Node& curr) {
	for (Node& node : all_nodes) {
		bool dist_bool = manhattan_dist(node.x, curr.x, node.y, curr.y) == 1;
		bool free_bool = node.free;
		bool self_bool = node.id != curr.id;
		bool closed_bool = node_in_closed(node);
		if (dist_bool && free_bool && self_bool && !closed_bool) {
			if (node.terminal) {
				std::cout << "*************** SOLVED ***************" << std::endl;
				node.parent_id = curr.id;
				node.parent_node = &curr;
				find_path(node);
				return true;
			}
			node.g = curr.g + manhattan_dist(node.x, curr.x, node.y, curr.y);
			node.h = manhattan_dist(node.x, goal_node.x, node.y, goal_node.y);
			node.f = node.g + node.h;
			node.parent_id = curr.id;
			open.push_back(node);
		};
	};
	return false;
}

void A_Star::solve() {
	make_environment();

	int goal_idx = pos_to_id(goal[0], goal[1]);
	goal_node = all_nodes[goal_idx];

	int start_idx = pos_to_id(start[0], start[1]);
	start_node = all_nodes[start_idx];
	start_node.g = 0;
	start_node.h = manhattan_dist(start_node.x, goal_node.x, start_node.y, goal_node.y);
	start_node.f = start_node.g + start_node.h;
	open.push_back(start_node);

	bool found_goal = false;

	while (!open.empty() && !found_goal) {
		std::sort(open.begin(), open.end(), compare_f);
		Node curr_node = open[0];
		open.erase(open.begin());
		closed.push_back(curr_node.id);

		found_goal = search_adjacent(curr_node);
	};
	std::cout << board << std::endl;
};


int main() {
	int starting_pos[2] = { 5, 0 };
	int ending_pos[2] = { 5, 6 };
	const int num_rows = 10;
	const int num_cols = 10;

	std::vector<Obstacle> obstacles;
	Obstacle o1 = { 5, 2 };
	obstacles.push_back(o1);

	Obstacle o2 = { 6, 2 };
	obstacles.push_back(o2);

	Obstacle o3 = { 4, 3 };
	obstacles.push_back(o3);

	A_Star astar(starting_pos, ending_pos, num_rows, num_cols, obstacles);
	astar.solve();

	return 0;
}