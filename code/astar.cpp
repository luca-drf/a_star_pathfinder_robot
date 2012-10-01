#include <iostream>
#include <fstream>
#include <string>
#include <vector>

using std::vector;
using std::cout;
using std::endl;
using std::string;
using std::ifstream;

size_t max_x, max_y;
vector<string> map;
short x_dir[4] = {0, 1, 0, -1};
short y_dir[4] = {1, 0, -1, 0};


class Node
{
    int x;
    int y;
    int g;
    int f;
    bool closed;
    bool opened;
    bool blocked;
    Node* parent;

    public:
        Node () {}
        Node (int x, int y, int g, Node* parent)
        : x(x), y(y), g(g), parent(parent), closed(0), opened(1), blocked(0) {}

        //~Node();
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

        void compute_f (const int & xG, const int & yG)
        {f = g + (abs( xG - x ) + abs( yG - y ));}
        
        void update_g (Node* C)
        {g = C->get_g();}

        void update_p (Node* C)
        {parent = C->get_p();}
        
        vector<Node*> trace_path();

        bool operator < (const Node & comp)
        {
            if (f != comp.get_f())
                return f < comp.get_f();
            else
                return g > comp.get_g();
        }
};

vector<Node*> Node::trace_path()
{
    vector<Node*> path;
    
    Node* tmp = this;
    path.push_back( tmp );
    
    while (tmp->parent != NULL)
    {
        tmp = tmp->parent;
        path.push_back( tmp );
    }
    return path;
}


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
        Node* pop ();
        void make_empty() {current_size = 0;}
        bool is_empty() const {return current_size == 0;}

    private:
        void percolate (int pos);
};

void Heap::rebuild ()
{
    for( int i = current_size / 2; i > 0; i-- )
        percolate( i );
}

void Heap::push ( Node * element )
{
    int hole = ++current_size;

    for( ; hole > 1 && *(element) < *(tree[hole / 2]); hole /= 2 )
        tree[hole] = tree[hole / 2];
    tree[hole] = element;
}

Node* Heap::pop ()
{
    Node* min = tree[1];
    tree[1] = tree[current_size--];
    percolate( 1 );
    return min;
}

void Heap::percolate ( int pos )
{
    int child;
    Node* tmp = tree[pos];

    for( ; pos * 2 <= current_size; pos = child ) {
        child = pos * 2;
        if( child != current_size && *(tree[child + 1]) < *(tree[child]) )
            child++;
        if( *(tree[child]) < *tmp )
            tree[pos] = tree[child];
        else
            break;
    }
    tree[pos] = tmp;
}

void node_map_clean ( vector< vector<Node*> > & node_map )
{
    vector< vector<Node*> >::iterator row;
    vector<Node*>::iterator col;

    for (row = node_map.begin(); row != node_map.end(); row++)
        for (col = row->begin(); col != row->end(); col++)
            *col = NULL;
}

void node_map_reset ( vector< vector<Node*> > & node_map )
{
    vector< vector<Node*> >::iterator row;
    vector<Node*>::iterator col;

    for (row = node_map.begin(); row != node_map.end(); row++)
        for (col = row->begin(); col != row->end(); col++){
            if (*col != NULL && !((*col)->is_blocked())) {
                delete *col;
                *col = NULL;
            }
        }
}

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

void print_map ()
{
    vector<string>::iterator line;
    for( line = map.begin(); line != map.end(); ++line )
        cout << *line << endl;
    cout << "___________________" << endl;
}

void load_map ( const char* file )
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
    }
    else cout << "Unable to read input file!" << endl;
}


vector<Node*> a_star ( vector< vector<Node*> > & node_map, 
                        int xS, int yS, int xG, int yG )
{
    static Node *N;
    static Node *C;
    Heap open_list( max_x * max_y );
    vector<Node*> path;
    static int xN, yN, xC, yC;
    static bool initial_state = 1;

    cout << "A*" << endl;
    
    C = new Node( xS, yS, 0, NULL );
    C->compute_f( xG, yG );
    open_list.push( C );
    cout << "Push: " << xS << " " << yS << endl;
    
    while (!open_list.is_empty())
    {
        //cout << "while" << endl;
        // get the current node w/ the highest priority
        // from the list of open nodes
        C = open_list.pop();
        xC = C->get_x();
        yC = C->get_y();
        cout << "Pop : " << xC << " " << yC << endl;
        
        if (xC == xG && yC == yG)
        {
            cout << "Trace path" << endl;
            path = C->trace_path();
            
            // garbage collection
            //delete C;
            // empty the leftover nodes!!!
            open_list.make_empty();
            
            return path;
        }
        cout << "Mark: " << xC << " " << yC << endl;
        node_map[xC][yC] = C;
        C->close();
        if (C->is_closed()) cout << "Closed: " << xC << " " << yC << endl;


        // generate moves (child nodes) in all possible directions
        for (int d = 0; d < 4; d++)
        {
            xN = xC + x_dir[d];
            yN = yC + y_dir[d];
            cout << "Expand: " << xN << " " << yN << endl;

            if (xN >= max_x || yN >= max_y || xN < 0 || yN < 0) continue;

            if (node_map[xN][yN] == NULL)
            {
                N = new Node(xN, yN, C->get_g() + 1, C );
                node_map[xN][yN] = N;
                cout << "Create: " << xN << " " << yN << endl;


                N->compute_f( xG, yG );
                N->open();
                open_list.push( N );
                cout << "Push: " << xN << " " << yN << endl;

            }
            else
            {
                N = node_map[xN][yN];

                if (N->is_blocked()) {
                    cout << xN << " " << yN << " is blocked" << endl;
                    continue;
                }
                else if (C->get_g() < N->get_g())
                {
                    N->update_g(C);
                    N->update_p(C);
                    N->compute_f( xG, yG );
                    cout << "Update: " << xN << " " << yN << endl;

                    if (N->is_open()){
                        cout << "Rebuild openlist" << endl;
                        open_list.rebuild();
                    }
                }
                //else delete N;
            }
        }
        //delete C;
    }
    return path;
}


int main (int argc, char** argv)
{
    vector<Node*> path;
    static int xG, yG, xS, yS, xN, yN, expanded_nodes;
    static bool goal = 0;

    load_map( argv[1] );
    static vector< vector<Node*> > node_map( max_x, vector<Node*>(max_y) );
    node_map_clean( node_map );

    print_map();
    find_s_g( xS, yS, xG, yG );
    //cout << "Start: " << xS << " " << yS << endl
    //     << "Goal: " << xG << " " << yG << endl;

    while (!goal)
    {

    for (int d = 0; d < 4; d++)
    {
        //cout << "for" << endl;
        xN = xS + x_dir[d];
        yN = yS + y_dir[d];
        
        cout << "Check: " << xN << " " << yN << endl;

        if (xN >= max_x || yN >= max_y || xN < 0 || yN < 0) continue;

        else if (map[xN].compare( yN, 1, "x" ) == 0)
        {
            if (node_map[xN][yN] == NULL){
                cout << "Create node: " << xN << " " << yN << endl;
                node_map[xN][yN] = new Node (xN, yN, 0, NULL);
            }
            cout << "Block node: " << xN << " " << yN << endl;
            node_map[xN][yN]->block();
        }
    }

        path = a_star( node_map, xS, yS, xG, yG );
        
        vector<Node*>::iterator i;
        for (i = path.begin(); i != path.end(); i++)
            cout << (*i)->get_x() << " " << (*i)->get_y() << endl;
        
        if (path.size() == 0) {
            cout << "No path" << endl;
            break;
        }

        vector<Node*>::reverse_iterator rit;
        for (rit = path.rbegin() + 1; rit != path.rend(); rit++)
        {
            xN = (*rit)->get_x();
            yN = (*rit)->get_y();

            if (map[xN].compare( yN, 1, "x" ) == 0)
            {
                break;
            }
            else if (xN == xG && yN == yG) goal = 1;

            else {
                if (map[xN].compare( yN, 1, "s" ) != 0)
                    map[xN][yN] = 'o';
                xS = xN;
                yS = yN;
            }
        }
        node_map_reset( node_map );
    }
    garbage_collector( node_map );
    
    print_map();


    return 0;
}
