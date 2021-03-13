#pragma once

#include <vector>
#include <Eigen/Dense>

using namespace Eigen;

struct Node {
	int x, y;
	int f = 0, g = 0, h = 0;
	int id, parent_id = -1;
	bool free = true;
	bool terminal = false;
	Node* parent_node;
};

struct Obstacle {
	int x, y;
};

class A_Star {
private:
	int start[2];
	int goal[2];
	int n_rows;
	int n_cols;

	Node start_node;
	Node goal_node;

	std::vector<Obstacle> obstacles;
	MatrixXi board;

public:
	A_Star(int s[2], int e[2], int nrow, int ncol, std::vector<Obstacle> obs);
	int pos_to_id(int x, int y);
	Node id_to_node(int node_id);
	int manhattan_dist(int x1, int x2, int y1, int y2);
	void make_environment();
	bool node_in_closed(Node& check_node);
	void find_path(Node& next_node);
	bool search_adjacent(Node& curr);
	void solve();

	std::vector<Node> all_nodes;
	std::vector<Node> open;
	std::vector<int> closed;

};