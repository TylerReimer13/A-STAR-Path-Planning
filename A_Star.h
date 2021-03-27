#pragma once

#include <vector>
#include <Eigen/Dense>

using namespace Eigen;

struct Node {
	int x, y;
	int f = 0, g = 0, h = 0;
	int id;
	bool free = true;
	bool terminal = false;
	bool closed = false;
	Node* parent;

	int getF() {
		return f;
	}

};

struct Obstacle {
	int x, y;
};

class A_Star {
private:
	int start[2];
	int goal[2];
	int len_x;
	int len_y;

	Node start_node;
	Node goal_node;

	std::vector<Obstacle> obstacles;
	MatrixXi board;

	int node_id_ctr;

public:
	A_Star(int s[2], int e[2], int lenx, int leny, std::vector<Obstacle> obs);
	int manhattan_dist(int x1, int x2, int y1, int y2);
	bool node_in_closed(Node& check_node);
	void find_path(Node* next);
	bool search_adjacent(Node* curr);
	void solve();

	std::vector<Node*> all_nodes;
	std::vector<Node*> open;
	std::vector<int> closed;

};