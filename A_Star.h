#pragma once

#include <vector>
#include <string>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;


struct Node {
	int x, y;
	int f = 0, g = 0, h = 0;
	Node* parent;
	
	Node(int x_, int y_, int f_, int g_, int h_) {
		x = x_;
		y = y_;
		f = f_;
		g = g_;
		h = h_;
	};
	
	Node() {};
};


struct Wall {
	int x1, xlen, y1, ylen;
	Wall(int x1_, int y1_, int xlen_, int ylen_) {
		x1 = x1_;
		y1 = y1_;
		xlen = xlen_;
		ylen = ylen_;
	};
};


class A_Star {
	private:
		int start[2];
		int goal[2];
		
		int len_x;
		int len_y;

		Node start_node();
		Node goal_node();

		MatrixXi board;

		vector<vector<int>> moves;
		int heuristic(int x1, int x2, int y1, int y2);
		
		bool found_goal = false;
		Node* find_node_by_xy(int p[2]);
		void find_path(Node* next);
		double path_len = 0.;

	public:
		A_Star(int s[2], int e[2], int lenx, int leny, vector<Wall> ws, string h);
		bool check_lower_f(Node* successor, Node* ref);
		void find_successors(Node* q);
		void solve();

		vector<Node*> open;
		vector<Node*> closed;
		vector<Node*> all_nodes;
		
		vector<Wall> walls;

		string heur = "euclidean";

};
