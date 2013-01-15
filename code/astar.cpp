//**********************************************************************//
// 10/11/2012                                                           //
//                                                                      //
// Pathfinder Robot Simulator using A* with Manhattan Method heuristic. //
//                                                                      //
// Luca Da Rin Fioretto                            ldarinfi@cs.nmsu.edu //
//**********************************************************************//

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <unistd.h>
#include <algorithm>


#define max(a,b) (a>b?a:b)

using std::vector;
using std::cout;
using std::endl;
using std::string;
using std::ifstream;
using std::reverse;

size_t max_x, max_y;                // Grid dimensions.
vector<string> map;                 // Real map.
bool great, adaptive;               // Tie-breaking and AA* flags.
short x_dir[4] = {0, 1, 0, -1};     // Directions coordinates to check
short y_dir[4] = {1, 0, -1, 0};     // neighbours.


class Node
{
    int x;
    int y;
    int g;
    int h;
    int f;
    bool closed;
    bool opened;
    bool blocked;
    Node* parent;

    public:
        Node () {}
        Node (int x, int y, int g, Node* parent)
        : x(x), y(y), g(g), parent(parent), closed(0), opened(0), blocked(0) {}

        int get_x() const {return x;}
        int get_y() const {return y;}
        int get_g() const {return g;}
        int get_f() const {return f;}
        Node* get_p() const {return parent;}
        bool is_open() const {return opened;}
        bool is_closed() const {return closed;}
        bool is_blocked() const {return blocked;}

        void close() {closed = 1; opened = 0;}
        void open() {opened = 1; closed = 0;}
        void block() {blocked = 1;}
        void reset() {opened = 0; closed = 0;}

        void compute_h (const int & xG, const int & yG)
        {h = abs( xG - x ) + abs( yG - y );}

        void compute_f ()
        {f = g + h;}
        
        void update_h (const Node* goal)
        {h = max( h, (goal->get_g() - g) );}
        
        void update_g (Node* C)
        {g = C->get_g() + 1;}

        void update_p (Node* C)
        {parent = C;}
        
        vector<Node*> trace_path();
        
};

// Traces the path from a given node to the starting point of the
// search.
vector<Node*> Node::trace_path()
{
    vector<Node*> path;
    //cout << "Trace function" << endl;

    Node* tmp = this;
    path.push_back( tmp );
    
    while (tmp->parent != NULL)
    {
        //cout << "Node: "<< tmp->get_x() << " " << tmp->get_y();
        tmp = tmp->parent;
        //cout << " --> " << tmp->get_x() << " " << tmp->get_y() << endl;
        path.push_back( tmp );
    }
    return path;
}

// Order function for Node objects.
// Also implement tie breaking criteria in favor of larger/smaller g's
// decided by user.
bool min (const Node & a, const Node & b)
{
    if (a.get_f() != b.get_f())
        return a.get_f() < b.get_f();
    else if (great)
        return a.get_g() > b.get_g();
    else if (!great)
        return a.get_g() < b.get_g();
}

// Binary Heap to implement a priority queue.
class Heap
{
    vector<Node*> tree;
    int current_size;

    public:
        Heap () {}
        Heap (size_t max_size)
        : current_size(0) {
         tree.reserve( ++max_size );
        }
        void rebuild ();
        void push ( Node * element );
        Node* top ();
        void make_empty() {current_size = 0;}
        bool is_empty() const {return current_size == 0;}

    private:
        void percolate (int pos);
};

// Rebuilds the heap structure after a priority update
void Heap::rebuild ()
{
    for( int i = current_size / 2; i > 0; i-- )
        percolate( i );
}

// Push a Node* in the correct position in the tree.
void Heap::push ( Node * element )
{
    int hole = ++current_size;

    for( ; hole > 1 && min(*(element), *(tree[hole / 2])); hole /= 2 )
        tree[hole] = tree[hole / 2];
    tree[hole] = element;
}

// Return a Node* to the top element of the list, then delete it from
// the heap and recalculate the heap structure
Node* Heap::top ()
{
    Node* min = tree[1];
    tree[1] = tree[current_size--];
    percolate( 1 );
    return min;
}

// Percolate an element down in the tree
void Heap::percolate ( int pos )
{
    int child;
    Node* tmp = tree[pos];

    for( ; pos * 2 <= current_size; pos = child ) {
        child = pos * 2;
        if( child != current_size && min(*(tree[child + 1]), *(tree[child]) ))
            child++;
        if( min(*(tree[child]), *tmp) )
            tree[pos] = tree[child];
        else
            break;
    }
    tree[pos] = tmp;
}

// Initialize the node_map with NULL pointers.
void node_map_clean ( vector< vector<Node*> > & node_map )
{
    vector< vector<Node*> >::iterator row;
    vector<Node*>::iterator col;

    for (row = node_map.begin(); row != node_map.end(); row++)
        for (col = row->begin(); col != row->end(); col++)
            *col = NULL;
}

// Reset the node_map.
// All but the blocked nodes are deleted.
// If the search is in adaptive mode, closed nodes will be updated
// rather than deleted.
void node_map_reset ( vector< vector<Node*> > & node_map, Node* goal)
{
    vector< vector<Node*> >::iterator row;
    vector<Node*>::iterator col;

    for (row = node_map.begin(); row != node_map.end(); row++)
        for (col = row->begin(); col != row->end(); col++){
            if (*col != NULL && !((*col)->is_blocked())) {
                if (adaptive && (*col)->is_closed()) {
                    (*col)->update_h( goal );
                    (*col)->reset();
                }
                else{
                    delete *col;
                    *col = NULL;
                }
            }
        }
}

// Delete all the nodes in the map and set the pointers to NULL.
void garbage_collector ( vector< vector<Node*> > & node_map )
{
    vector< vector<Node*> >::iterator row;
    vector<Node*>::iterator col;

    for (row = node_map.begin(); row != node_map.end(); row++)
        for (col = row->begin(); col != row->end(); col++){
            if (*col != NULL) {
                delete *col;
                *col = NULL;
            }
        }
}

// Look for starting and goal point on the real map.
void find_s_g ( int & xS, int & yS, int & xG, int & yG )
{
    vector<string>::iterator line;
    int x = 0, s, g;

    for( line = map.begin(); line != map.end(); ++line, ++x ) {
        s = int( line->find( "s" ) );
        g = int( line->find( "g" ) );

        if (s != -1) {
            xS = x;
            yS = s;
        }
        if (g != -1) {
            xG = x;
            yG = g;
        }
    }
}

// Print real map.
void print_map ()
{
    vector<string>::iterator line;
    for( line = map.begin(); line != map.end(); ++line )
        cout << *line << endl;
    cout << "___________________" << endl;
}

// Load the real map from file and store it in map.
bool load_map ( const char* file )
{
    string line;
    char c;
    int i = 0;
    ifstream input( file );
    if( input.is_open() ) {

        getline( input, line );
        max_x = atoi(line.c_str());
        
        getline( input, line );
        max_y = atoi(line.c_str());

        while( input.good() ) {
            getline( input, line );
            line.erase( remove( line.begin(), line.end(), ' ' ), line.end() );
            map.push_back( line );
        }
        input.close();
        return 1;
    }
    else{
        cout << "Unable to read input file!" << endl;
        return 0;
    }
}

// Checks weather the neighbours cells of a given position are free or
// blocked and updates the node_map.
void check_blocked_neighbours ( vector< vector<Node*> > & node_map,
                                vector<string> & map, int & xN, int & yN,
                                int xS, int yS)
{
    for (int d = 0; d < 4; d++)
    {
        //cout << "for" << endl;
        xN = xS + x_dir[d];
        yN = yS + y_dir[d];
        
        //cout << "Check: " << xN << " " << yN << endl;
        
        if (xN >= max_x || yN >= max_y || xN < 0 || yN < 0) continue;

        else if (map[xN].compare( yN, 1, "x" ) == 0)
        {
            if (node_map[xN][yN] == NULL){
                //cout << "Create node: " << xN << " " << yN << endl;
                node_map[xN][yN] = new Node (xN, yN, 0, NULL);
            }
            //cout << "Block node: " << xN << " " << yN << endl;
            node_map[xN][yN]->block();
        }
    }
}

// A*/AA* search code.
// Returns an Node* vector containing the shortest path to the goal
// (based on the robot knowledge)
vector<Node*> a_star ( vector< vector<Node*> > & node_map, 
                        int xS, int yS, int xG, int yG, float & expanded_nodes)
{
    static Node *N;
    static Node *C;
    Heap open_list( max_x * max_y );
    vector<Node*> path;
    static int xN, yN, xC, yC;
    static bool initial_state = 1;

    //cout << "A*" << endl;

    // Create a new starting node
    if (node_map[xS][yS] != NULL)
        delete node_map[xS][yS];
    C = new Node( xS, yS, 0, NULL );
    node_map[xS][yS] = C;
    C->compute_h( xG, yG );
    C->compute_f();
    C->open();
    open_list.push( C );
    //cout << "Push: " << xS << " " << yS << endl;
    
    // A* main loop.
    while (!open_list.is_empty())
    {
        //cout << "while" << endl;
        
        // Get the top node from the open list and its position.
        C = open_list.top();
        C->close();
        expanded_nodes++;
        xC = C->get_x();
        yC = C->get_y();
        //cout << "top : " << xC << " " << yC << endl;
        
        // If the expanded node is the goal, returns the path to the
        // starting point then empty the open list.
        if (xC == xG && yC == yG)
        {
            //cout << "Trace path" << endl;
            path = C->trace_path();
            
            open_list.make_empty();
            
            return path;
        }
        //cout << "Mark: " << xC << " " << yC << endl;
        
        // Else mark the node on the node_map.
        //if (node_map[xC][yC] == NULL)
        //    node_map[xC][yC] = C;
        //if (C->is_closed()) cout << "Closed: " << xC << " " << yC << endl;


        // Check the neighbours of the current node (C).
        // If the node does not exist, generate it and push it in the
        // open list.
        // Else if the neighbour is not an obstacle update g and p.
        // Else if it contains information from a previous search (AA*
        // only) update g, p and push it into the open list.
        for (int d = 0; d < 4; d++)
        {
            xN = xC + x_dir[d];
            yN = yC + y_dir[d];
            //cout << "Expand: " << xN << " " << yN << endl;

            if (xN >= max_x || yN >= max_y || xN < 0 || yN < 0) continue;

            if (node_map[xN][yN] == NULL)
            {
                N = new Node(xN, yN, C->get_g() + 1, C );
                node_map[xN][yN] = N;
                //cout << "Create: " << xN << " " << yN << endl;
                N->compute_h( xG, yG );
                N->compute_f();
                N->open();
                open_list.push( N );
                //cout << "Push: " << xN << " " << yN << endl;
            }
            else
            {
                N = node_map[xN][yN];

                if (N->is_blocked()) {
                //    cout << xN << " " << yN << " is blocked" << endl;
                    continue;
                }
                else if ((C->get_g() + 1 < N->get_g()) && N->is_open())
                {
                    N->update_g(C);
                    N->update_p(C);
                    N->compute_f();
                    //cout << "Update: " << xN << " " << yN << endl;

                    //if (N->is_open()){
                        //cout << "Rebuild openlist" << endl;
                        open_list.rebuild();
                    //}
                }
                else if (!(N->is_open() || N->is_closed()))
                {
                    N->open();
                    N->update_g(C);
                    N->update_p(C);
                    N->compute_f();
                    open_list.push( N );
                }
            }
        }
    }
    return path;
}

int main (int argc, char** argv)
{
    vector<Node*> path;
    vector<Node*>::iterator i;
    
    static int xG, yG, xS, yS, xN, yN;
    float expanded_nodes = 0, searches = 0;
    static bool goal = 0;

    bool rev;
    char *input_file = NULL;
    char *direction = NULL;
    char *tie = NULL;

    int index;
    int c;

    
    opterr = 0;
    
    // Command line parameters processing.
    while ((c = getopt (argc, argv, "Ai:d:t:")) != -1)
      switch (c)
        {
        case 'A':
          adaptive = 1;
          break;
        case 'i':
          input_file = optarg;
          break;
        case 'd':
          direction = optarg;
          break;
        case 't':
          tie = optarg;
          break;
        case '?':
          if (optopt == 'c')
            fprintf (stderr, "Option -%c requires an argument.\n", optopt);
          else if (isprint (optopt))
            fprintf (stderr, "Unknown option `-%c'.\n", optopt);
          else
            fprintf (stderr,
                     "Unknown option character `\\x%x'.\n",
                     optopt);
          return 1;
        default:
          abort ();
        }
    
    //printf ("adaptive = %d, input = %s, direction = %s, tie = %s\n",
    //        adaptive, input_file, direction, tie);
    
    for (index = optind; index < argc; index++)
      printf ("Non-option argument %s\n", argv[index]);
    
    if (direction != NULL){
        if (*direction == 'f') rev = 0;
        else if (*direction == 'b') rev = 1;
        else{
            cout << "Wrong direction argument" << endl;
            return 1;
        }
    }
    else {
        cout << "Please specify a search direction: -d {f|b}" << endl;
        return 1;
    }

    if (tie != NULL){
        if (*tie == 's') great = 0;
        else if (*tie == 'l') great = 1;
        else{
            cout << "Wrong tie-breaking argument" << endl;
            return 1;
        }
    }
    else {
        cout << "Please specify a tie-breaking criteria: -t {s|l}" << endl;
        return 1;
    }

    // Load the real map from file.
    if(!load_map( input_file )) return 1;
    static vector< vector<Node*> > node_map( max_x, vector<Node*>(max_y) );
    node_map_clean( node_map );

    // Cool std output prints.
    cout << "= Map =" << endl;
    print_map();
    find_s_g( xS, yS, xG, yG );
     cout << "Start: " << xS << " " << yS << endl
          << "Goal: " << xG << " " << yG << endl;

    cout << "= Robot moves =" << endl;
    
    // Main robot loop.
    while (!goal)
    {
        // Look if there are obstacles around.
        check_blocked_neighbours( node_map, map, xN, yN, xS, yS );

        // Forward search
        if (!rev){
            searches++;
            path = a_star( node_map, xS, yS, xG, yG, expanded_nodes );
            reverse( path.begin(), path.end() );
        }
        // Backward search
        else{
            searches++;
            path = a_star( node_map, xG, yG, xS, yS, expanded_nodes );
        
        }

        //for (i = path.begin(); i != path.end(); i++)
        //    cout << (*i)->get_x() << " " << (*i)->get_y() << endl;

        if (path.size() == 0) {
            cout << endl << "!! No path !!" << endl <<endl;
            break;
        }
        

        // Robot moves loop.
        for (i = path.begin() + 1; i != path.end(); i++)
        {
            // Aquire coordinates of the following position.
            xN = (*i)->get_x();
            yN = (*i)->get_y();

            // If the cell is the goal, stay put and report the success.
            // If the cell is not blocked by obstacles, move into it.
            if (node_map[xN][yN]->is_blocked())
            {
                break;
            }
            else if (xN == xG && yN == yG)
            {
                goal = 1;
                // cout << (*i)->get_x() << " " << (*i)->get_y() << endl;
                
            }
            else {
                if (map[xN].compare( yN, 1, "s" ) != 0)
                    map[xN][yN] = 'o';
                
                // Print out your movements.
                cout << (*i)->get_x() << " " << (*i)->get_y() << endl;
                
                xS = xN;
                yS = yN;
                
                // Meanwhile you move check if there are obstacles along
                // the path.
                check_blocked_neighbours( node_map, map, xN, yN, xS, yS );
            }
        }
        // If the current cell is not the goal reset the node_map.
        // If an A* search is performed, this means, erase all nodes
        // informations but the obstacles.
        // If an AA* search is performed, leave the information about
        // the closed nodes and update their euristic.
        if (!goal)
            node_map_reset( node_map, node_map[xG][yG]);
    }
    // When the robot has finished his work delete all the remains
    // nodes.
    garbage_collector( node_map );
    
    // Other cool std output prints.
    cout << "= Visited cells map =" << endl;
    print_map();

    cout << "Grid dimensions: " << max_x << " x " << max_y << endl;
    cout << "Expanded nodes: " << expanded_nodes << endl;
    cout << "Searches: " << searches << endl;
    cout << "Ratio: " << expanded_nodes / searches << endl;

    return 0;
}

