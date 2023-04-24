#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <queue>
#include <stack>
#include <sstream>


using namespace std;

void print_node(struct node* current_node, ofstream &file);
struct node* initialize_node(string a, string b);
struct node* bfs(struct node* initial_node, struct node* goal_node, int& nodes_expanded);
struct node* dfs(struct node* initial_node, struct node* goal_node, int &nodes_expanded);
struct node* iddfs(struct node* initial_node, struct node* goal_node, int& nodes_expanded);
struct node* astar(struct node* initial_node, struct node* goal_node, int& nodes_expanded);
struct node* initialize_node(string a, string b);
bool compare_nodes(struct node* a, struct node* b);
bool in_list(vector<struct node*> &list, struct node* current);
bool is_fail_state(struct node* current);
void copy_child(struct node* a, struct node* b);
void expand(vector<struct node*> &list, struct node* current_node);
void solution_path(struct node* intial_node, struct node* current_node, char* arg);
void create_heuristic(struct node* current_node, struct node* goal_node);

struct node {
	int l_chick = 0;
	int l_wolf = 0;
	bool l_boat = 0;
	int r_chick = 0;
	int r_wolf = 0;
	bool r_boat = 0;
	struct node* parent = NULL; //default NULL, points to the node that expanded to create this node
	int depth = 0;
	int heuristic = 0;
	bool operator<(const node& b) {
		return heuristic > b.heuristic;
	}
};

int main(int argc, char** argv) {


	//initial state
	//goal state
	//mode
	//output

	if (argc != 5)
		return -1;

	//read initial state
	ifstream file;
	file.open(argv[1]);
	string a, b;
	file >> a >> b;

	//load initial state into node and add to list
	struct node* initial_node = initialize_node(a, b);
	//vector<struct node*> list;
	//list.push_back(initial_node);
	file.close();

	//read goal state
	file.open(argv[2]);
	file >> a >> b;

	//load goal state into node
	struct node* goal_node = initialize_node(a, b);

	//mode switch and call graph search function
	struct node* solution_node = NULL;
	string mode = string(argv[3]);
	int nodes_expanded = 0;
	if (mode == "bfs") {
		solution_node = bfs(initial_node, goal_node, nodes_expanded);
	}
	else if (mode == "dfs") {
		solution_node = dfs(initial_node, goal_node, nodes_expanded);
	}
	else if (mode == "iddfs") {
		solution_node = iddfs(initial_node, goal_node, nodes_expanded);
	}
	else if (mode == "astar") {
		solution_node = astar(initial_node, goal_node, nodes_expanded);
	}
	else {
		cout << "BAD MODE" << endl;
		return -1;
	}



	if (solution_node) {
		solution_path(initial_node, solution_node, argv[4]);
		cout << "nodes expanded: " << nodes_expanded << endl;
		cout << "Solution Depth: " << solution_node->depth << endl;
	}
	else {
		cout << "NULL SOLUTION" << endl;
		return -1;
	}

	return 0;

}

void print_node(struct node* current_node, ofstream &file) {
	cout << current_node->l_chick << ", " <<
		current_node->l_wolf << ", " <<
		current_node->l_boat << endl <<
		current_node->r_chick << ", " <<
		current_node->r_wolf << ", " <<
		current_node->r_boat << endl << endl;

	file << current_node->l_chick << ", " <<
		current_node->l_wolf << ", " <<
		current_node->l_boat << endl <<
		current_node->r_chick << ", " <<
		current_node->r_wolf << ", " <<
		current_node->r_boat << endl;
}

struct node* bfs(struct node* initial_node, struct node* goal_node, int& nodes_expanded) {

	int list_size;

	vector<struct node*> list;
	list.push_back(initial_node);
	queue<struct node*> frontier;
	frontier.push(initial_node);

	while (!frontier.empty()) {
		struct node* current_node = frontier.front();
		frontier.pop();

		//return if goal reached
		if (compare_nodes(current_node, goal_node)) {
			return current_node;
		}

		nodes_expanded++;
		list_size = list.size();
		expand(list, current_node);
		for (int i = list_size; i < (int)list.size(); i++) {
			frontier.push(list[i]);
		}
	}
	return NULL; //fail state

}

struct node* dfs(struct node* initial_node, struct node* goal_node, int &nodes_expanded) {

	int list_size;
	int depth_limit = 500;

	vector<struct node*> list;
	list.push_back(initial_node);
	stack<struct node*> frontier;
	frontier.push(initial_node);
	struct node* current_node = NULL;

	while (!frontier.empty()) {
		current_node = frontier.top();
		frontier.pop();
		if (current_node->depth > depth_limit)
			continue;

		//return if goal reached
		if (compare_nodes(current_node, goal_node)) {
			return current_node;
		}

		nodes_expanded++;
		list_size = list.size();
		expand(list, current_node);
		for (int i = list_size; i < (int)list.size(); i++) {
			frontier.push(list[i]);
		}
	}
	return NULL; //fail state

}

struct node* iddfs(struct node* initial_node, struct node* goal_node, int& nodes_expanded) {

	int list_size;

	for (int depth_limit = 0; depth_limit < 500; depth_limit++) {
		vector<struct node*> list;
		list.push_back(initial_node);
		stack<struct node*> frontier;
		frontier.push(initial_node);

		while (!frontier.empty()) {
			struct node* current_node = frontier.top();
			frontier.pop();

			//return if goal reached
			if (compare_nodes(current_node, goal_node)) {
				return current_node;
			}

			list_size = list.size();
			if (current_node->depth < depth_limit) {
				nodes_expanded++;
				expand(list, current_node);
			}
			for (int i = list_size; i < (int)list.size(); i++) {
				frontier.push(list[i]);
			}
		}
	}
	return NULL; //fail state

}

struct node* astar(struct node* initial_node, struct node* goal_node, int& nodes_expanded) {

	int list_size;

	vector<struct node*> list;
	create_heuristic(initial_node, goal_node);
	list.push_back(initial_node);
	priority_queue<struct node*, vector<struct node*>> frontier;
	frontier.push(initial_node);

	while (!frontier.empty()) {
		struct node* current_node = frontier.top();
		frontier.pop();

		//return if goal reached
		if (compare_nodes(current_node, goal_node)) {
			return current_node;
		}

		nodes_expanded++;
		list_size = list.size();
		expand(list, current_node);
		for (int i = list_size; i < (int)list.size(); i++) {
			create_heuristic(list[i], goal_node);
			frontier.push(list[i]);
		}
	}
	return NULL; //fail state

}

void create_heuristic(struct node* current_node, struct node* goal_node) {
	//int heuristic;
	//current_node->heuristic = goal_node->l_chick + goal_node->l_wolf - current_node->l_chick - current_node->l_wolf;
	//if (current_node->l_boat) {
	//	current_node->heuristic = (goal_node->l_chick + goal_node->l_wolf - current_node->l_chick - current_node->l_wolf) / 2;
	//}
	//if (current_node->r_boat) {
	//	current_node->heuristic = (goal_node->l_chick + goal_node->l_wolf - current_node->l_chick - current_node->l_wolf + 1) / 2;
	//}
	current_node->heuristic = 2 * (current_node->r_chick + current_node->r_wolf) - 1;
}

void solution_path(struct node* initial_node, struct node* current_node, char* arg) {

	stack<struct node*> stack;

	while (current_node->parent != NULL) {
		stack.push(current_node);
		current_node = current_node->parent;
	}

	ofstream file;
	file.open(arg);
	print_node(initial_node, file);
	while (!stack.empty()) {
		struct node* node = stack.top();
		stack.pop();
		print_node(node, file);
	}
	file.close();

}

struct node* initialize_node(string a, string b) {
	stringstream a_ss(a);
	stringstream b_ss(b);
	vector<string> temp;
	string str;

	while (getline(a_ss, str, ',')) {
		temp.push_back(str);
	}
	while (getline(b_ss, str, ',')) {
		temp.push_back(str);
	}

	struct node* current_node = new struct node;
	current_node->l_chick = stoi(temp[0]);
	current_node->l_wolf = stoi(temp[1]);
	current_node->l_boat = stoi(temp[2]);
	current_node->r_chick = stoi(temp[3]);
	current_node->r_wolf = stoi(temp[4]);
	current_node->r_boat = stoi(temp[5]);
	current_node->parent = NULL;

	return current_node;
}

bool compare_nodes(struct node* a, struct node* b) {
	if (a->l_chick == b->l_chick &&
		a->l_wolf == b->l_wolf &&
		a->l_boat == b->l_boat &&
		a->r_chick == b->r_chick &&
		a->r_wolf == b->r_wolf &&
		a->r_boat == b->r_boat) {
		return true;
	}
	return false;
}

bool in_list(vector<struct node*> &list, struct node* current) {
	for (int i = 0; i < (int)list.size(); i++) {
		if (compare_nodes(current, list[i]))
			return true;
	}
	return false;
}

bool is_fail_state(struct node* current) {
	if ((current->l_wolf > current->l_chick &&
		current->l_chick > 0) ||
		(current->r_wolf > current->r_chick &&
		current->r_chick > 0) ||
		current->l_wolf < 0 ||
		current->l_chick < 0 || 
		current->r_wolf < 0 || 
		current->r_chick < 0)
		return true;
	return false;
		
}

void copy_child(struct node* a, struct node* b) {
	a->l_chick = b->l_chick;
	a->l_wolf = b->l_wolf;
	a->l_boat = b->l_boat;
	a->r_chick = b->r_chick;
	a->r_wolf = b->r_wolf;
	a->r_boat = b->r_boat;
}

void expand(vector<struct node*> &list,/* stack<struct node*> &frontier,*/ struct node* current_node) {

	struct node* temp = new struct node;
	copy_child(temp, current_node);

	if (current_node->l_boat) {

		//1 chicken on boat
		temp->l_chick--;
		temp->r_chick++;
		temp->l_boat = false;
		temp->r_boat = true;
		if (!is_fail_state(temp) && !in_list(list, temp)) {
			struct node* case1 = new struct node;
			copy_child(case1, temp);
			case1->parent = current_node;
			case1->depth = current_node->depth + 1;
			//frontier.push(case1);
			list.push_back(case1);
		}
		copy_child(temp, current_node);

		//2 chickens on boat
		temp->l_chick -= 2;
		temp->r_chick += 2;
		temp->l_boat = false;
		temp->r_boat = true;
		if (!is_fail_state(temp) && !in_list(list, temp)) {
			struct node* case2 = new struct node;
			copy_child(case2, temp);
			case2->parent = current_node;
			case2->depth = current_node->depth + 1;
			//frontier.push(case2);
			list.push_back(case2);
		}
		copy_child(temp, current_node);

		//1 wolf on boat
		temp->l_wolf--;
		temp->r_wolf++;
		temp->l_boat = false;
		temp->r_boat = true;
		if (!is_fail_state(temp) && !in_list(list, temp)) {
			struct node* case3 = new struct node;
			copy_child(case3, temp);
			case3->parent = current_node;
			case3->depth = current_node->depth + 1;
			//frontier.push(case3);
			list.push_back(case3);
		}
		copy_child(temp, current_node);

		//1 wolf 1 chicken
		temp->l_chick--;
		temp->r_chick++;
		temp->l_wolf--;
		temp->r_wolf++;
		temp->l_boat = false;
		temp->r_boat = true;
		if (!is_fail_state(temp) && !in_list(list, temp)) {
			struct node* case5 = new struct node;
			copy_child(case5, temp);
			case5->parent = current_node;
			case5->depth = current_node->depth + 1;
			//frontier.push(case5);
			list.push_back(case5);
		}
		copy_child(temp, current_node);

		//2 wolf
		temp->l_wolf -= 2;
		temp->r_wolf += 2;
		temp->l_boat = false;
		temp->r_boat = true;
		if (!is_fail_state(temp) && !in_list(list, temp)) {
			struct node* case4 = new struct node;
			copy_child(case4, temp);
			case4->parent = current_node;
			case4->depth = current_node->depth + 1;
			//frontier.push(case4);
			list.push_back(case4);
		}



	}

	if (current_node->r_boat) {

		//1 chicken on boat
		temp->r_chick--;
		temp->l_chick++;
		temp->r_boat = false;
		temp->l_boat = true;
		if (!is_fail_state(temp) && !in_list(list, temp)) {
			struct node* case1 = new struct node;
			copy_child(case1, temp);
			case1->parent = current_node;
			case1->depth = current_node->depth + 1;
			//frontier.push(case1);
			list.push_back(case1);
		}
		copy_child(temp, current_node);

		//2 chickens on boat
		temp->r_chick -= 2;
		temp->l_chick += 2;
		temp->r_boat = false;
		temp->l_boat = true;
		if (!is_fail_state(temp) && !in_list(list, temp)) {
			struct node* case2 = new struct node;
			copy_child(case2, temp);
			case2->parent = current_node;
			case2->depth = current_node->depth + 1;
			//frontier.push(case2);
			list.push_back(case2);
		}
		copy_child(temp, current_node);

		//1 wolf on boat
		temp->r_wolf--;
		temp->l_wolf++;
		temp->r_boat = false;
		temp->l_boat = true;
		if (!is_fail_state(temp) && !in_list(list, temp)) {
			struct node* case3 = new struct node;
			copy_child(case3, temp);
			case3->parent = current_node;
			case3->depth = current_node->depth + 1;
			//frontier.push(case3);
			list.push_back(case3);
		}
		copy_child(temp, current_node);

		//1 wolf 1 chicken
		temp->r_chick--;
		temp->l_chick++;
		temp->r_wolf--;
		temp->l_wolf++;
		temp->r_boat = false;
		temp->l_boat = true;
		if (!is_fail_state(temp) && !in_list(list, temp)) {
			struct node* case5 = new struct node;
			copy_child(case5, temp);
			case5->parent = current_node;
			case5->depth = current_node->depth + 1;
			//frontier.push(case5);
			list.push_back(case5);
		}
		copy_child(temp, current_node);

		//2 wolf
		temp->r_wolf -= 2;
		temp->l_wolf += 2;
		temp->r_boat = false;
		temp->l_boat = true;
		if (!is_fail_state(temp) && !in_list(list, temp)) {
			struct node* case4 = new struct node;
			copy_child(case4, temp);
			case4->parent = current_node;
			case4->depth = current_node->depth + 1;
			//frontier.push(case4);
			list.push_back(case4);
		}



	}
}