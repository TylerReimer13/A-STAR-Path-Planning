#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath> 
#include <string>
#include <Eigen/Dense>
#include "A_Star.h"

using namespace Eigen;
using namespace std;


bool compare_f(Node* a, Node* b) {
	return a->f < b->f;
};


A_Star::A_Star(int s[2], int e[2], int lenx, int leny, vector<Wall> ws, string h) {
	start[0] = s[0];
	start[1] = s[1];
	
	goal[0] = e[0];
	goal[1] = e[1];

	len_x = lenx;
	len_y = leny;

	board = MatrixXi::Zero(len_y, len_x);
	board(len_y - s[1] - 1, s[0]) = 1;
	board(len_y - e[1] - 1, e[0]) = 2;
	
	heur = h;
	
	for (auto& w : ws) {
		for (int xi = w.x1; xi < w.x1 + w.xlen; xi++) {
			for (int yi = w.y1; yi < w.y1 + w.ylen; yi++) {
				board(len_y - yi - 1, xi) = 5;
			};
		};
	};
	
	if (heur == "euclidean") {
		moves = { {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1} };
	}
	
	else {
		moves = { {1, 0}, {-1, 0}, {0, 1}, {0, -1} };
	};

	cout << board << endl;
};


int A_Star::heuristic(int x1, int x2, int y1, int y2) {
	if (heur == "euclidean") {
		return sqrt(pow((x2 - x1) * 10, 2) + pow((y2 - y1) * 10, 2));
	}
	else {
		return abs(x2 - x1) * 10 + abs(y2 - y1) * 10;
	}
};


bool A_Star::check_lower_f(Node* successor, Node* ref) {
	if (successor->f < ref->f) {
		return true;
	};
	
	return false;
};


Node* A_Star::find_node_by_xy(int p[2]) {
	for (auto& n : all_nodes) {
		if (p[0] == n->x && p[1] == n->y) {
			return n;
		};
	};
	return NULL;
};


void A_Star::find_path(Node* next) {
	// Reverse linked list to find path to goal
	vector<int> path_ids;
	while (next->parent != NULL) {
		cout << "PATH NODE: " <<"X: "<< next->x << "  Y: "<< next->y << std::endl;
		path_len += heuristic(next->x, next->parent->x, next->y, next->parent->y) / 10.;
		next = next->parent;
		board(len_y - next->y - 1, next->x) = 1;
	}
};


void A_Star::find_successors(Node* q) {
	for (auto& mov : moves) {
		int pos[2] = {q->x + mov[0], q->y + mov[1]};
		bool in_bounds = pos[0] >= 0 && pos[0] < len_x && pos[1] >= 0 && pos[1] < len_y;
		if (in_bounds) {
			
			if (board(len_y - pos[1] - 1, pos[0]) == 3 || board(len_y - pos[1] - 1, pos[0]) == 5) {
				continue;
			}; 
			
			int g = q->g + heuristic(pos[0], q->x, len_y - pos[1] - 1, len_y - q->y - 1);
			int h = heuristic(pos[0], goal[0], len_y - pos[1] - 1, len_y - goal[1] - 1);
			int f = g + h;
			Node* new_node = new Node(pos[0], pos[1], f, g, h);
			new_node->parent = q;
			
			if (board(len_y - pos[1] - 1, pos[0]) == 2) {
				cout << "**************SOLVED***************" << endl << endl;
				found_goal = true;
				find_path(new_node);
			};
			
			Node* ref_node = find_node_by_xy(pos);
			
			// If havent seen this cell before
			if (ref_node == NULL) {  

				open.push_back(new_node);
			}
			else {
				if (check_lower_f(new_node, ref_node)) {
					open.erase(remove(open.begin(), open.end(), ref_node), open.end());
					open.push_back(new_node);
				};
			};
			
			all_nodes.push_back(new_node);
		};
	};
};


void A_Star::solve() {
	Node* goal_node = new Node(goal[0], goal[1], 0., 0., 0.);
	
	Node* start_node = new Node(start[0], start[1], 0., 0., 0.);
	start_node->parent = NULL;
	
	all_nodes.push_back(start_node);
	open.push_back(start_node);
	
	cout << endl;
	
	while (!found_goal && open.size() != 0) {
		sort(open.begin(), open.end(), compare_f);  // Sort Nodes by f.
		Node* q = open[0];  // Node with smallest 'f' score.
		open.erase(open.begin());
		board(len_y - q->y - 1, q->x) = 3;  // This replaces using the closed list (faster)
		find_successors(q);
	};
	
	cout << endl;
	cout << board << endl;
	cout << endl;
	//cout << "LEN ALL NODES: " << all_nodes.size() << endl;
	cout << "PATH LENGTH: " << path_len << endl;
};


int main() {
	const int num_rows = 10;
	const int num_cols = 10;
	int starting_pos[2] = { 3, 0 };
	int ending_pos[2] = { 0, 6 };
	
	Wall w1(0, 2, 7, 2);
	vector<Wall> walls = {w1};

	A_Star astar(starting_pos, ending_pos, num_rows, num_cols, walls, "euclidean");
	astar.solve();
	
	//cout << astar.all_nodes.size() << endl;

	return 0;
}
