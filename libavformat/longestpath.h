#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>

#ifndef GRAPH_STACK
#define MAX_STACK_SIZE 10000

#ifndef FULL COV
#define FULL_COV 1472
#endif // !FULL COV

typedef struct gStack {
    int top;
    int buffer[MAX_STACK_SIZE];
} gStack;

void gstack_pop(gStack* s);
int gstack_emtpy(gStack s);
int gstack_full(gStack s);
void gstack_init(gStack* s);
int gstack_top(const gStack* s);
void gstack_push(gStack*s, int value);

void gstack_pop(gStack* s) { --s->top; }
int gstack_emtpy(gStack s) { return s.top == -1; }
int gstack_full(gStack s) { return s.top == MAX_STACK_SIZE; }
void gstack_init(gStack* s) {
    s->top = -1;
}

int gstack_top(const gStack* s) {
    return s->buffer[s->top];
}

void gstack_push(gStack*s, int value) {
    ++s->top;
    s->buffer[s->top] = value;
}
#endif // !GRAPH_STACK

#ifndef GRAPH_ALGO
#define MAX_VERTEX_NUMBER 100
typedef struct AdjListNode {
    int v;
    int w;
} AdjListNode;

typedef struct GraphNode {
    int b; //buffer size
    int z; //checksum coverage
    int k; //the step this node is represented for
} GraphNode;

typedef struct Graph {
    int V;
    GraphNode vnodes[MAX_VERTEX_NUMBER];
    AdjListNode adj[MAX_VERTEX_NUMBER][MAX_VERTEX_NUMBER];
    int tails[MAX_VERTEX_NUMBER];
} Graph;

Graph construct_graph(int V, const int *bs, const int *zs, const int *ks);
int destroy_graph(Graph* g);
void graph_add_edge(Graph* g, int u, int v, int weight);
void graph_clear_edges(Graph* g);
int longestPath(Graph g, int source, int dist[MAX_VERTEX_NUMBER], int pre[MAX_VERTEX_NUMBER]);
#endif // !GRAPH_ALGO

#define NINF INT_MIN

static int adj_list_alloc(AdjListNode adj[MAX_VERTEX_NUMBER][MAX_VERTEX_NUMBER], int V) {
    int i;
    for (i = 0; i < V; ++i) {
        adj[i][0].v = -1;
        adj[i][0].w = -1;
    }
    return 0;
}

static void adj_list_free(AdjListNode adj[MAX_VERTEX_NUMBER][MAX_VERTEX_NUMBER], int V) {
    memset(adj, 0, sizeof(adj));
    int i;
    for (i = 0; i < V; ++i) {
        adj[i][0].v = -1;
        adj[i][0].w = -1;
    }
}

Graph construct_graph(int V, const int* bs, const int *zs, const int *ks) {
    Graph g;
    int i;
    g.V = V;
    memset(g.vnodes, 0, sizeof(g.vnodes));
    memset(g.adj, 0, sizeof(g.adj));
    if (adj_list_alloc(g.adj, V) != 0) {
        printf("Failed to alloc memory for Linked List.");
        exit(1);
    }
    for (i = 0; i < V; ++i) {
        g.tails[i] = 0;
        g.vnodes[i].b = bs[i];
        g.vnodes[i].z = zs[i];
        g.vnodes[i].k = ks[i];
    }
    return g;
}

int destroy_graph(Graph* g) {
    int i = 0;
    adj_list_free(g->adj, g->V);
    memset(g, 0, sizeof(g));
    g->V = -1;
    return 0;
}

void graph_add_edge(Graph* g, int u, int v, int weight) {
    AdjListNode node;
    node.v = v;
    node.w = weight;
    g->adj[u][g->tails[u]] = node;
    g->adj[u][g->tails[u] + 1].v = -1;
    ++g->tails[u];
}

void graph_clear_edges(Graph* g) {
    adj_list_free(g->adj, g->V);
}

static void topsort(Graph* g, int v, int visited[], gStack* stack) {
    visited[v] = 1;
    int i = 0;
    for (i = 0; i < g->tails[v]; ++i) {
        AdjListNode node = g->adj[v][i];
        if (visited[node.v] == 0) {
            topsort(g, node.v, visited, stack);
        }
    }
    //printf("%d;", v);
    gstack_push(stack, v);
}

static int path_trace_back(const Graph* g, int pre[MAX_VERTEX_NUMBER], int v) {
    int u;
    if (v < 0 || v > g->V) {
        printf("Invalid start nodes for path_trace!\n");
        exit(1);
        return -1;
    }
    u = pre[v];
    if (u < 0 || u > g->V) {
        printf("Nodes has no precessor!\n");
        //exit(1);
        return v;
    }
    while (pre[u] != -1) {
        v = u;
        u = pre[u];
    }
    return v;
}

int longestPath(Graph g, int source, int dist[MAX_VERTEX_NUMBER], int pre[MAX_VERTEX_NUMBER]) {
    int visited[MAX_VERTEX_NUMBER];
    int u;
    int i;
    int maxDist, maxDistIndex;
    int nextVertex;
    AdjListNode inode;
    memset(visited, 0, sizeof(visited));
    for (i = 0; i < g.V; ++i) {
        dist[i] = NINF;
    }
    dist[source] = 0;
    pre[source] = -1;
    gStack stack;
    gstack_init(&stack);
    topsort(&g, 1, visited, &stack);

    //process the longest path in topological order
    while (!gstack_emtpy(stack)) {
        int u = gstack_top(&stack);

        gstack_pop(&stack);
        if (dist[u] != NINF) {
            for (i = 0; i < g.tails[u]; ++i) {
                inode = g.adj[u][i];
                if (dist[inode.v] < dist[u] + inode.w) {
                    dist[inode.v] = dist[u] + inode.w;
                    pre[inode.v] = u;
                }
            }
        }
    }

    //initialize the maxDist and maDistIndex
    maxDist = NINF;
    maxDistIndex = -1;
    //find the node with maxDist
    //which should be at the last layer
    for (i = 0; i < g.V; i++) {
        if (dist[i] > maxDist) {
            maxDist = dist[i];
            maxDistIndex = i;
        }
    }

    //use the precessor array to back_track the nextVertex
    //record the nextVertex to use in algorithm
    //use the buffer size and checksum coverage of nextVertex in next move
    nextVertex = -1;
    nextVertex = path_trace_back(&g, pre, maxDistIndex);

    return nextVertex;
}
